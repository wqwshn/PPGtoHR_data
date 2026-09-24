"""PPG enabled/disabled, each crossed with BLE normal/reset; no source edits."""
import json
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import analyze as base

OUT=base.OUT
BANDS={'total':(.5,50),'b20':(18,22),'b40':(38,42),'b33':(32.5,34.2)}


def run():
    base.load()
    records=[];spectra={};quality=[]
    for run,label in [(1,'PPG开启'),(2,'PPG关闭')]:
        item=base.DATA[('项目腕表','复位',run)];d=item['d'];e=item['e'];s=item['s']
        quality.append(dict(ppg=label,rows=len(d),duration_s=len(d)/100,
                            ppg_nonzero=int((d[['PPG_Green','PPG_Red','PPG_IR']]!=0).any(axis=1).sum()),
                            fifo_samples_delta=int(s.PpgFifoSampleTotalCounter.iloc[-1]-s.PpgFifoSampleTotalCounter.iloc[0]),
                            complete=int((e.Event=='COMPLETE').sum()),event_gap=int((e.EventId.diff().dropna()!=1).sum()),
                            queue_dropped=int(e.QueueDropped.max())))
        for cycle in [1,2,3]:
            events=[e[(e.Cycle==cycle)&(e.Event==code)].iloc[0] for code in ['NORMAL_START','RESET_ASSERT','RESET_RELEASE']]
            assert all(events[i+1]['McuTime(ms)']-events[i]['McuTime(ms)']==30000 for i in [0,1])
            for state,ev in zip(['连接','复位'],events[:2]):
                for trim in [5,10]:
                    lo=ev.SampleCounter+trim*100;hi=ev.SampleCounter+(30-trim)*100
                    seg=d[(d.sample_counter>=lo)&(d.sample_counter<hi)]
                    assert len(seg)==(30-2*trim)*100
                    for ch in ['Ut1','Ut2']:
                        x=seg[ch+'(mV)'].to_numpy();f,p=base.psd(x)
                        vals={k+'_rms_mV':base.rms(f,p,*v) for k,v in BANDS.items()}
                        vals['other_rms_mV']=float(np.sqrt(max(0,vals['total_rms_mV']**2-vals['b20_rms_mV']**2-vals['b40_rms_mV']**2)))
                        records.append(dict(ppg=label,state=state,cycle=cycle,trim_s=trim,channel=ch,
                            mcu_start_s=ev['McuTime(ms)']/1000+trim,mean_mV=float(x.mean()),**vals))
                        if trim==5:spectra[(label,state,cycle,ch)]=(f,p)
    r=pd.DataFrame(records);r.to_csv(OUT/'ppg_metrics.csv',index=False,encoding='utf-8-sig')
    pd.DataFrame(quality).to_csv(OUT/'ppg_quality.csv',index=False,encoding='utf-8-sig')
    summary=[]
    for keys,g in r[r.trim_s==5].groupby(['ppg','state','channel']):
        row=dict(zip(['ppg','state','channel'],keys))
        for col in [c for c in r if c.endswith('_rms_mV')]:
            row[col]=np.sqrt(np.mean(g[col]**2))
        summary.append(row)
    summary=pd.DataFrame(summary);summary.to_csv(OUT/'ppg_summary.csv',index=False,encoding='utf-8-sig')
    print(pd.DataFrame(quality).to_string(index=False));print(summary.to_string(index=False))
    print(r[r.trim_s==5][['ppg','state','cycle','channel','total_rms_mV','b20_rms_mV','b40_rms_mV']].to_string(index=False))
    plt.rcParams.update({'font.family':['Arial','Microsoft YaHei'],'font.size':8,'axes.spines.top':False,
        'axes.spines.right':False,'legend.frameon':False,'svg.fonttype':'none','pdf.fonttype':42,'axes.unicode_minus':False})
    colors={'PPG开启':'#BC7966','PPG关闭':'#477DA5'}
    fig,axs=plt.subplots(2,2,figsize=(7.2,5.5),layout='constrained',sharex=True,sharey=True)
    for i,state in enumerate(['连接','复位']):
        for j,ch in enumerate(['Ut1','Ut2']):
            ax=axs[i,j]
            for label in colors:
                arr=[spectra[(label,state,c,ch)] for c in [1,2,3]];f=arr[0][0];p=np.mean([a[1] for a in arr],axis=0)
                ax.semilogy(f[f>=.5],p[f>=.5],c=colors[label],ls='-' if label=='PPG开启' else '--',lw=1)
            for lo,hi in [(18,22),(38,42)]:ax.axvspan(lo,hi,color='.8',alpha=.3,zorder=0)
            ax.axvline(33.3,c='.6',ls=':',lw=.6)
            ax.set(xlim=(.5,50),xlabel='频率（Hz）',ylabel='PSD（mV²/Hz）')
            ax.set_title(f'{chr(97+i*2+j)}  蓝牙{state} · {ch}',loc='left',fontweight='bold')
    fig.legend(handles=[Line2D([],[],c=colors[l],ls='-' if l=='PPG开启' else '--',label=l) for l in colors],loc='outside upper center',ncol=2)
    save(fig,'07-ppg-spectrum')
    fig,axs=plt.subplots(1,2,figsize=(7.2,3.5),layout='constrained',sharey=True)
    for j,ch in enumerate(['Ut1','Ut2']):
        ax=axs[j]
        for i,state in enumerate(['连接','复位']):
            for di,label in enumerate(colors):
                v=r[(r.trim_s==5)&(r.channel==ch)&(r.state==state)&(r.ppg==label)].sort_values('cycle').total_rms_mV.to_numpy()
                x=i+(-.15 if di==0 else .15)
                ax.scatter(x+np.linspace(-.035,.035,3),v,c=colors[label],marker='o' if di==0 else '^',s=24)
                val=np.sqrt(np.mean(v*v));ax.plot([x-.07,x+.07],[val,val],c=colors[label],lw=2)
                ax.text(x,max(v)+.025,f'{val:.3f}',ha='center',fontsize=8)
        ax.set(xticks=[0,1],xticklabels=['蓝牙连接','保持复位'],ylim=(0,.95),ylabel='0.5–50 Hz RMS（mV）')
        ax.set_title(f'{chr(97+j)}  {ch}',loc='left',fontweight='bold')
    fig.legend(handles=[Line2D([],[],c=colors[l],marker='o' if l=='PPG开启' else '^',ls='',label=l+'：每点一轮') for l in colors]+[Line2D([],[],c='.3',lw=2,label='短横线：三轮平均功率开方')],loc='outside upper center',ncol=1)
    save(fig,'08-ppg-rms')
    # Match on/off by elapsed cycle only; these are separate boots, not randomized repeats.
    assert len(r)==48
    prior=pd.read_csv(OUT/'rf_metrics.csv')
    prior=prior[prior.device=='项目腕表']
    matched=r[r.ppg=='PPG开启'].merge(prior,on=['cycle','state','trim_s','channel'])
    assert len(matched)==24
    assert np.allclose(matched.total_rms_mV,matched.rms_mV,rtol=1e-10,atol=1e-12)
    assert quality[1]['ppg_nonzero']==0 and quality[1]['fifo_samples_delta']==0
    checks=dict(records=48,raw_continuity='passed',event_integrity='passed',
                metric_consistency='24 PPG-on windows match existing rf_metrics.csv',
                ppg_off_read_activity='zero PPG output and zero FIFO sample delta')
    (OUT/'ppg_validation.json').write_text(json.dumps(checks,indent=2),encoding='utf-8')


def save(fig,name):
    sys.path.insert(0,str(base.Path.home()/'.codex/skills/scipilot-figure-skill/scripts'))
    from visual_qa import audit_layout
    issues=[str(i) for i in audit_layout(fig)]
    (OUT/(name+'-qa.json')).write_text(json.dumps(issues,ensure_ascii=False),encoding='utf-8')
    fig.savefig(OUT/(name+'.png'),dpi=300)
    fig.savefig(OUT/(name+'.svg'));fig.savefig(OUT/(name+'.pdf'));plt.close(fig)


if __name__=='__main__':run()
