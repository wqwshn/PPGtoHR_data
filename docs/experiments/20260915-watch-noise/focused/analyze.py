"""Focused revision: 0.5–50 Hz RMS, fresh recordings, event-aligned RF comparison."""
from pathlib import Path
import hashlib
import json
import sys
import numpy as np
import pandas as pd
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]
SOURCE=ROOT/'recordings/20260914-热膜噪声蓝牙问题分析'
FS=100
COLORS={'论文腕表':'#477DA5','项目腕表':'#BC7966'}
CHANNELS=['Ut1','Ut2']
DATA={}
QUALITY=[]


def load():
    for p in sorted(SOURCE.glob('*/kaiji*_0915.csv')):
        dev='论文腕表' if p.parent.name.startswith('论文') else '项目腕表'
        phase='复位' if '关闭对比' in p.parent.name else '佩戴' if '佩戴' in p.parent.name else '静置' if '稳定后' in p.parent.name else '开机'
        run=2 if 'kaiji2' in p.stem else 1
        d=pd.read_csv(p);s=pd.read_csv(p.with_name(p.stem+'_status.csv'))
        assert d.ValidFlag.eq(1).all() and d.InterpFlag.eq(0).all()
        assert np.all(np.diff(d.SampleIndex)==1)
        assert np.all(np.diff(d.Seq.to_numpy(dtype=int))%65536==1)
        assert np.allclose(d['Time(s)'],d.SampleIndex/100)
        assert np.all(np.diff(s['McuTime(ms)'])>0)
        offset=float(np.median(s['McuTime(ms)']/1000-s.SampleCounter/100))
        sc0=int(d.Seq.iloc[0])+round((s.SampleCounter.iloc[0]-int(d.Seq.iloc[0]))/65536)*65536+1
        # Source firmware sends seq starting at zero; counter starts at one.
        assert (s.FrameCounter==s.SampleCounter).all()
        d['sample_counter']=np.arange(len(d))+sc0
        d['mcu_s']=d.sample_counter/100+offset
        item=dict(d=d,s=s,path=p,offset=offset,sc0=sc0)
        if phase=='复位':
            e=pd.read_csv(p.with_name(p.stem+'_rf_events.csv'))
            assert e.QueueDropped.max()==0 and np.all(np.diff(e.EventId)==1)
            assert not e.Event.isin(['NORMAL_ABORT','WAIT_WARNING','RESET_WARNING']).any()
            assert sum(e.Event=='COMPLETE')==1
            item['e']=e
        DATA[(dev,phase,run)]=item
        q=dict(device=dev,phase=phase,run=run,rows=len(d),duration_s=len(d)/100,
               mcu_start=float(d.mcu_s.iloc[0]),mcu_end=float(d.mcu_s.iloc[-1]),
               path=str(p.relative_to(ROOT)),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
        for c in ['TxBusyCounter','TxErrorCounter','AdcErrorCounter','ImuErrorCounter','PcMissingRaw','PcRawInvalidCandidates']:
            q[c+'_delta']=int(s[c].iloc[-1]-s[c].iloc[0]); assert q[c+'_delta']==0
        QUALITY.append(q)
    pd.DataFrame(QUALITY).to_csv(OUT/'inventory.csv',index=False,encoding='utf-8-sig')


def psd(x):
    return signal.welch(x,fs=100,nperseg=1000,noverlap=500,window='hann',detrend='linear',scaling='density')


def rms(f,p,lo=.5,hi=50):
    return float(np.sqrt(np.sum(p[(f>=lo)&(f<=hi)])*(f[1]-f[0])))


def metric(x):
    f,p=psd(x)
    return dict(rms_mV=rms(f,p),mean_mV=float(np.mean(x)),std_raw_mV=float(np.std(x,ddof=1)),
                slope_mV_min=float(np.polyfit(np.arange(len(x))/100,x,1)[0]*60))


def primary(dev,phase):
    run=2 if dev=='项目腕表' and phase in ['开机','静置'] else 1
    return DATA[(dev,phase,run)]


def compute():
    rows=[]; rfrows=[]; peakrows=[]; settling=[]
    for (dev,phase,run),item in DATA.items():
        if phase=='复位':continue
        d=item['d']
        windows={'前30秒':(0,30),'前60秒':(0,60),'末30秒':(len(d)/100-30,len(d)/100),
                 '末60秒':(len(d)/100-60,len(d)/100),'10–20秒':(10,20)}
        for label,(a,b) in windows.items():
            for ch in CHANNELS:
                x=d[ch+'(mV)'].to_numpy()[round(a*100):round(b*100)]
                rows.append(dict(device=dev,phase=phase,run=run,window=label,channel=ch,start_s=a,end_s=b,**metric(x)))
        if phase=='开机':
            # Rolling 60s slope of one-second means; sustained 120s of endpoint tests.
            for ch in CHANNELS:
                x=d[ch+'(mV)'].to_numpy(); x=x[:len(x)//100*100].reshape(-1,100).mean(1)
                slopes=np.array([np.polyfit(np.arange(60),x[a:a+60],1)[0]*60 for a in range(len(x)-59)])
                for threshold in [.5,1,2,5]:
                    ok=abs(slopes)<threshold
                    starts=[i for i in range(len(ok)-119) if ok[i:i+120].all()]
                    # Earliest ending time of first qualifying 60s window, retrospectively sustained.
                    t=starts[0]+60 if starts else None
                    settling.append(dict(device=dev,run=run,channel=ch,threshold_mV_min=threshold,
                                         recording_s=t,mcu_s=t+float(d.mcu_s.iloc[0]) if t is not None else None,
                                         tail_slope=float(slopes[-1])))
    for dev in COLORS:
        item=DATA[(dev,'复位',1)];d=item['d'];e=item['e']
        for cycle in [1,2,3]:
            transitions=[e[(e.Event==event)&(e.Cycle==cycle)].iloc[0] for event in ['NORMAL_START','RESET_ASSERT','RESET_RELEASE']]
            assert transitions[1]['McuTime(ms)']-transitions[0]['McuTime(ms)']==30000
            assert transitions[2]['McuTime(ms)']-transitions[1]['McuTime(ms)']==30000
            for phase,event in zip(['连接','复位'],transitions[:2]):
                for trim in [5,10]:
                    # +5..+25 seconds (main), +10..+20 sensitivity; no switch boundaries.
                    lo=int(event.SampleCounter)+trim*100;hi=int(event.SampleCounter)+(30-trim)*100
                    seg=d[(d.sample_counter>=lo)&(d.sample_counter<hi)]
                    assert len(seg)==(30-2*trim)*100
                    for ch in CHANNELS:
                        x=seg[ch+'(mV)'].to_numpy();f,p=psd(x)
                        rfrows.append(dict(device=dev,cycle=cycle,state=phase,trim_s=trim,channel=ch,
                            start_mcu_s=float(event['McuTime(ms)']/1000+trim),**metric(x)))
                        if trim==5:
                            for center in [6.6667,10,13.3333,20,26.6667,33.3333,40,46.6667]:
                                peakrows.append(dict(device=dev,cycle=cycle,state=phase,channel=ch,
                                                     peak_Hz=center,rms_mV=rms(f,p,center-.4,center+.4),band='窄带±0.4Hz'))
                            for lo,hi in [(18,22),(38,42)]:
                                mask=(f>=lo)&(f<=hi)
                                peakrows.append(dict(device=dev,cycle=cycle,state=phase,channel=ch,
                                    peak_Hz=(lo+hi)/2,rms_mV=rms(f,p,lo,hi),
                                    observed_peak_Hz=float(f[mask][np.argmax(p[mask])]),band=f'{lo}–{hi}'))
    for name,records in [('metrics',rows),('rf_metrics',rfrows),('rf_peaks',peakrows),('settling',settling)]:
        pd.DataFrame(records).to_csv(OUT/(name+'.csv'),index=False,encoding='utf-8-sig')
    print('PRIMARY');m=pd.DataFrame(rows)
    print(m[(m.window=='前30秒')&(m.phase!='开机')].to_string(index=False))
    print('RF');print(pd.DataFrame(rfrows).query('trim_s == 5').to_string(index=False))
    print('SETTLING');print(pd.DataFrame(settling).to_string(index=False))


def plot():
    plt.rcParams.update({'font.family':['Arial','Microsoft YaHei'],'font.size':8,
        'axes.spines.top':False,'axes.spines.right':False,'axes.linewidth':.7,
        'legend.frameon':False,'svg.fonttype':'none','pdf.fonttype':42,'axes.unicode_minus':False})
    sys.path.insert(0,str(Path.home()/'.codex/skills/scipilot-figure-skill/scripts'))
    from visual_qa import audit_layout
    qa={}
    def save(fig,name):
        fig.savefig(OUT/(name+'.png'),dpi=200)
        qa[name]=[str(i) for i in audit_layout(fig)]
        if '--export' in sys.argv:
            fig.savefig(OUT/(name+'.svg'));fig.savefig(OUT/(name+'.pdf'))
            fig.savefig(OUT/(name+'.png'),dpi=300)
        plt.close(fig)
    # Contract: equal-band comparison; dots are within-recording windows, not replicates.
    fig,axs=plt.subplots(1,2,figsize=(7.2,3.5),layout='constrained',sharey=True)
    for j,ch in enumerate(CHANNELS):
        ax=axs[j]
        for k,phase in enumerate(['静置','佩戴']):
            for di,dev in enumerate(COLORS):
                x=primary(dev,phase)['d'][ch+'(mV)'].to_numpy()[:3000]
                val=metric(x)['rms_mV'];vals=[metric(x[a:a+1000])['rms_mV'] for a in range(0,3000,1000)]
                xpos=k+(-.14 if di==0 else .14)
                ax.scatter(xpos+np.linspace(-.035,.035,3),vals,c=COLORS[dev],marker='o' if di==0 else '^',s=24,alpha=.65)
                ax.plot([xpos-.07,xpos+.07],[val,val],c=COLORS[dev],lw=2.5)
                ax.text(xpos,max(vals)+.035,f'{val:.3f}',ha='center')
        ax.set_xticks([0,1],['静置','佩戴']);ax.set_title(f'{chr(97+j)}  {ch}',loc='left',fontweight='bold');ax.set_ylabel('0.5–50 Hz RMS（mV）');ax.set_ylim(0,.9)
    fig.legend(handles=[Line2D([],[],color=COLORS[d],marker='o' if i==0 else '^',ls='',label=d+'：10秒窗 RMS') for i,d in enumerate(COLORS)]+[Line2D([],[],color='.3',lw=2.5,label='短横线：所选30秒 RMS')],loc='outside upper center',ncol=1,fontsize=8)
    save(fig,'01-rms')
    # Spectrum, peaks labelled on a dedicated top axis (no text over data).
    fig,axs=plt.subplots(2,2,figsize=(7.2,5.6),layout='constrained',sharex=True,sharey=True)
    for i,phase in enumerate(['静置','佩戴']):
        for j,ch in enumerate(CHANNELS):
            ax=axs[i,j]
            for dev in COLORS:
                f,p=psd(primary(dev,phase)['d'][ch+'(mV)'].to_numpy()[:3000])
                ax.semilogy(f[f>=.5],p[f>=.5],color=COLORS[dev],lw=.85,ls='-' if dev=='论文腕表' else '--')
            for c in np.arange(1,8)*100/15:ax.axvline(c,color='.75',lw=.5,ls=':',zorder=0)
            ax.set_title(f'{chr(97+i*2+j)}  {phase} · {ch}',loc='left',fontweight='bold')
            ax.set(xlim=(.5,50),ylabel='PSD（mV²/Hz）',xlabel='频率（Hz）')
            if i==0:
                top=ax.secondary_xaxis('top');top.set_xticks([6.7,13.3,20,26.7,33.3,40,46.7]);top.set_xticklabels(['6.7','13.3','20','26.7','33.3','40','46.7'],fontsize=6)
    fig.legend(handles=[Line2D([],[],c=COLORS[d],ls='-' if d=='论文腕表' else '--',label=d) for d in COLORS],loc='outside upper center',ncol=2)
    save(fig,'02-spectrum')
    # Raw absolute mV: separate devices to avoid concealing noise beneath DC offsets.
    fig,axs=plt.subplots(4,2,figsize=(7.2,7.9),layout='constrained',sharex=True)
    rawrows=[];rawstats=[]
    for i,(phase,dev) in enumerate([(p,d) for p in ['静置','佩戴'] for d in COLORS]):
        for j,ch in enumerate(CHANNELS):
            ax=axs[i,j];x=primary(dev,phase)['d'][ch+'(mV)'].to_numpy()[1000:1200]
            mu=x.mean();sd=x.std(ddof=1)
            rawstats.append(dict(device=dev,phase=phase,channel=ch,start_s=10,end_s=12,mean_mV=mu,std_raw_mV=sd))
            ax.plot(np.arange(200)/100+10,x,c=COLORS[dev],lw=.7)
            ax.axhline(mu,c='.2',lw=.7);ax.axhline(mu+sd,c='.3',ls='--',lw=.7);ax.axhline(mu-sd,c='.3',ls='--',lw=.7)
            ax.set_title(f'{chr(97+i*2+j)}  {dev} · {phase} · {ch}',loc='left',fontweight='bold',fontsize=8)
            ax.set_ylabel('原始电压（mV）');ax.ticklabel_format(axis='y',style='plain',useOffset=False)
            ax.text(.02,.96,f'μ={mu:.2f}，σ={sd:.2f} mV',transform=ax.transAxes,va='top',fontsize=7,
                    bbox=dict(facecolor='white',alpha=.8,edgecolor='none',pad=1))
            extent=max(abs(x-mu).max()*1.15,2)
            # Same y-span for each channel across all conditions assigned below.
            rawrows.append((ax,mu,extent))
    extent=max(r[2] for r in rawrows)
    for ax,mu,_ in rawrows:ax.set_ylim(mu-extent,mu+extent)
    for ax in axs[-1]:ax.set_xlabel('录制时间（秒）')
    fig.legend(handles=[Line2D([],[],c='.2',label='均值 μ'),Line2D([],[],c='.3',ls='--',label='μ ± 原始标准差 σ')],loc='outside upper center',ncol=2)
    save(fig,'03-raw')
    pd.DataFrame(rawstats).to_csv(OUT/'raw_display_stats.csv',index=False,encoding='utf-8-sig')
    # Startup absolute means plus slopes; stopping criterion documented, not a thermal-time fit.
    fig,axs=plt.subplots(2,2,figsize=(7.2,5.2),layout='constrained',sharex=True,sharey='row')
    for j,ch in enumerate(CHANNELS):
        for dev in COLORS:
            d=primary(dev,'开机')['d'];x=d[ch+'(mV)'].to_numpy();n=len(x)//100;x=x[:n*100].reshape(n,100).mean(1)
            t=np.arange(n)+.5+float(d.mcu_s.iloc[0])
            axs[0,j].plot(t,x,c=COLORS[dev],lw=1,label=dev)
            slopes=np.array([np.polyfit(np.arange(60),x[a:a+60],1)[0]*60 for a in range(n-59)])
            axs[1,j].plot(t[59:],slopes,c=COLORS[dev],lw=1)
        axs[0,j].set_title(f'{chr(97+j)}  {ch} · 开机趋势',loc='left',fontweight='bold');axs[0,j].set_ylabel('1秒原始均值（mV）')
        axs[1,j].axhspan(-1,1,color='.9',zorder=0);axs[1,j].set_title(f'{chr(99+j)}  60秒滚动斜率',loc='left',fontweight='bold')
        axs[1,j].set_ylabel('变化速率（mV/min）');axs[1,j].set_xlabel('MCU 上电后时间（秒）')
    axs[0,0].legend();save(fig,'04-startup')
    # RF paired cycles with all repeats and within-phase PSD, no pooled causal exaggeration.
    r=pd.read_csv(OUT/'rf_metrics.csv').query('trim_s == 5')
    fig,axs=plt.subplots(2,2,figsize=(7.2,5.3),layout='constrained',sharey=True)
    for i,dev in enumerate(COLORS):
        for j,ch in enumerate(CHANNELS):
            ax=axs[i,j]
            for cycle in [1,2,3]:
                vals=[float(r[(r.device==dev)&(r.channel==ch)&(r.cycle==cycle)&(r.state==state)].rms_mV.iloc[0]) for state in ['连接','复位']]
                ax.plot([0,1],vals,c=COLORS[dev],marker=['o','s','^'][cycle-1],ls=['-','--',':'][cycle-1],lw=1,label=f'第{cycle}轮')
            ax.set_xticks([0,1],['连接','保持复位']);ax.set_xlim(-.2,1.2);ax.set_ylim(0,.55)
            ax.set_title(f'{chr(97+i*2+j)}  {dev} · {ch}',loc='left',fontweight='bold');ax.set_ylabel('0.5–50 Hz RMS（mV）')
    axs[0,0].legend();save(fig,'05-rf-pairs')
    fig,axs=plt.subplots(2,2,figsize=(7.2,5.3),layout='constrained',sharex=True,sharey=True)
    for i,dev in enumerate(COLORS):
        for j,ch in enumerate(CHANNELS):
            ax=axs[i,j];item=DATA[(dev,'复位',1)]
            for state,eventname,color,ls in [('连接','NORMAL_START',COLORS[dev],'-'),('复位','RESET_ASSERT','.3','--')]:
                powers=[]
                for cycle in [1,2,3]:
                    ev=item['e'][(item['e'].Event==eventname)&(item['e'].Cycle==cycle)].iloc[0]
                    d=item['d'];x=d[(d.sample_counter>=ev.SampleCounter+500)&(d.sample_counter<ev.SampleCounter+2500)][ch+'(mV)'].to_numpy()
                    f,p=psd(x);powers.append(p)
                mean=np.mean(powers,axis=0);ax.semilogy(f[f>=.5],mean[f>=.5],c=color,ls=ls,lw=1,label=state)
            for center in [6.7,13.3,20,26.7,33.3,40,46.7]:ax.axvline(center,c='.8',lw=.4,ls=':')
            ax.set(xlim=(.5,50),ylabel='PSD（mV²/Hz）',xlabel='频率（Hz）');ax.set_title(f'{chr(97+i*2+j)}  {dev} · {ch}',loc='left',fontweight='bold');ax.legend()
    save(fig,'06-rf-spectrum')
    (OUT/'figure-qa.json').write_text(json.dumps(qa,ensure_ascii=False,indent=2),encoding='utf-8')


def validate():
    t=np.arange(3000)/100
    # 50 Hz is the Nyquist bin: include it, without incorrectly doubling its power.
    x=5+.1*t+np.sin(2*np.pi*2*t)+.4*np.sin(2*np.pi*33.3*t)+.2*np.cos(2*np.pi*50*t)
    f,p=psd(x);expected=np.sqrt(.5+.08+.04)
    assert np.isclose(rms(f,p),expected,rtol=1e-4)
    y=5+.1*t+np.sin(2*np.pi*.1*t)
    fy,py=psd(y)
    # Finite Hann windows plus detrending are not an ideal brick-wall filter.
    # 1 mV slow sine should leak less than 1% of its RMS into the metric.
    assert rms(fy,py)<.01/np.sqrt(2)
    r=pd.read_csv(OUT/'rf_metrics.csv')
    assert len(r)==48 and len(QUALITY)==11
    details=[]
    for dev in COLORS:
        item=DATA[(dev,'复位',1)];d=item['d'];e=item['e']
        for cycle in [1,2,3]:
            for ch in CHANNELS:
                for state,event in [('连接','NORMAL_START'),('复位','RESET_ASSERT')]:
                    ev=e[(e.Event==event)&(e.Cycle==cycle)].iloc[0]
                    values=[]
                    for shift in [-1,0,1]:
                        x=d[(d.sample_counter>=ev.SampleCounter+500+shift)&(d.sample_counter<ev.SampleCounter+2500+shift)][ch+'(mV)'].to_numpy()
                        values.append(metric(x)['rms_mV'])
                    details.append(dict(device=dev,cycle=cycle,channel=ch,state=state,
                                        max_alignment_change_pct=max(abs(np.array(values)/values[1]-1))*100))
    pd.DataFrame(details).to_csv(OUT/'alignment_sensitivity.csv',index=False,encoding='utf-8-sig')
    (OUT/'validation.json').write_text(json.dumps(dict(synthetic_rms=rms(f,p),expected_rms=expected,
        low_frequency_leakage_rms=rms(fy,py),quality_records=len(QUALITY),
        rf_phase_windows=24,alignment_max_pct=max(x['max_alignment_change_pct'] for x in details)),indent=2),encoding='utf-8')


if __name__=='__main__':
    load();compute();plot();validate()
