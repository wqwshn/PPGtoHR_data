"""1 uF hardware revision: raw HF startup, late windows and spectral comparison."""
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
ROOT=OUT.parents[2]
SOURCE=ROOT/'recordings'
SKILL=Path.home()/'.codex/skills/nature-figure/scripts'
sys.path.insert(0,str(SKILL))
from audit_panel_alignment import require_matplotlib_panel_alignment
FS=100
COLORS=['#d986a2','#eba9bc']
plt.rcParams.update({'font.family':['Arial','SimSun'],'font.size':10,'axes.titlesize':11,
    'axes.spines.top':False,'axes.spines.right':False,'legend.frameon':False,
    'svg.fonttype':'none','pdf.fonttype':42,'axes.unicode_minus':False})

def load(path):
    d=pd.read_csv(path);s=pd.read_csv(path.with_name(path.stem+'_status.csv'))
    assert d.ValidFlag.eq(1).all() and d.InterpFlag.eq(0).all()
    assert (np.diff(d.SampleIndex)==1).all() and (np.diff(d.Seq.astype(int))%65536==1).all()
    assert np.allclose(d['Time(s)'],d.SampleIndex/FS)
    offset=float(np.median(s['McuTime(ms)']/1000-s.SampleCounter/FS))
    assert np.ptp(s['McuTime(ms)']/1000-s.SampleCounter/FS)<.02
    sc0=int(d.Seq.iloc[0])+round((s.SampleCounter.iloc[0]-int(d.Seq.iloc[0]))/65536)*65536+1
    d['mcu_s']=(np.arange(len(d))+sc0)/FS+offset
    error_cols=['TxBusyCounter','TxErrorCounter','AdcErrorCounter','ImuErrorCounter','PcMissingRaw','PcRawInvalidCandidates','PpgFifoOverflowCounter']
    errors={c:int(s[c].iloc[-1]-s[c].iloc[0]) for c in error_cols}
    assert all(v==0 for v in errors.values())
    q=dict(file=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
           rows=len(d),duration_s=len(d)/FS,mcu_start_s=d.mcu_s.iloc[0],mcu_end_s=d.mcu_s.iloc[-1],
           errors=errors,ppg_nonzero_rows=int((d[['PPG_Green','PPG_Red','PPG_IR']]!=0).any(axis=1).sum()),
           fifo_samples_delta=int(s.PpgFifoSampleTotalCounter.iloc[-1]-s.PpgFifoSampleTotalCounter.iloc[0]))
    return d,q

def psd(x):
    return signal.welch(x,fs=FS,nperseg=1000,noverlap=500,window='hann',detrend='linear',scaling='density')

def rms(f,p,lo=.5,hi=50):
    return float(np.sqrt(np.sum(p[(f>=lo)&(f<=hi)])*(f[1]-f[0])))

def metrics(x):
    f,p=psd(x)
    peaks,_=signal.find_peaks(p)
    peaks=[i for i in peaks if f[i]>=.5]
    peaks=sorted(peaks,key=lambda i:p[i],reverse=True)[:8]
    return dict(mean_mV=float(x.mean()),sd_raw_mV=float(x.std(ddof=1)),
                slope_mV_min=float(np.polyfit(np.arange(len(x))/FS,x,1)[0]*60),
                rms_mV=rms(f,p),rms_20=rms(f,p,18,22),rms_40=rms(f,p,38,42),
                rms_33=rms(f,p,32.5,34.2),rms_05_5=rms(f,p,.5,5),
                peaks_Hz=';'.join(f'{f[i]:.1f}' for i in peaks))

def title(ax,label,text):
    ax.set_title(text,loc='center',fontsize=11)
    ax.annotate(label,xy=(0,1),xycoords='axes fraction',xytext=(-5,9),textcoords='offset points',
                ha='right',va='bottom',fontweight='bold',fontsize=11)

def save(fig,name):
    fig.canvas.draw()
    require_matplotlib_panel_alignment(fig,json_out=OUT/(name+'.alignment.json'),strict=True)
    fig.savefig(OUT/(name+'.png'),dpi=300)
    fig.savefig(OUT/(name+'.svg'))
    fig.savefig(OUT/(name+'.pdf'))
    plt.close(fig)

def main():
    new,nq=load(SOURCE/'20260922-提升容值的项目腕表开机及静息记录/kaiji1_LYX_0922.csv')
    oldroot=SOURCE/'20260914-热膜噪声蓝牙问题分析(蓝牙电容容值不足)'
    rest,rq=load(oldroot/'项目腕表开机稳定后记录/kaiji2_LYX_0915.csv')
    boot,bq=load(oldroot/'项目腕表开机记录（长时间）/kaiji2_LYX_0915.csv')
    rows=[];blocks=[];settling=[]
    datasets={'修订后':new,'修订前静置':rest,'修订前开机':boot}
    for name,d in datasets.items():
        windows={'前30秒':d.iloc[:3000],'末30秒':d.iloc[-3000:],'末60秒':d.iloc[-6000:]}
        if len(d)>=12000:windows['末120秒']=d.iloc[-12000:]
        windows['前60秒']=d.iloc[:6000]
        # Chronologically matched window entirely contained by the older startup capture.
        if name!='修订前静置':windows['上电540–600秒']=d[(d.mcu_s>=540)&(d.mcu_s<600)]
        if name!='修订前开机':windows['上电681.035–741.035秒']=d[(d.mcu_s>=681.035)&(d.mcu_s<741.035)]
        for win,seg in windows.items():
            for ch in ['Ut1','Ut2']:
                rows.append(dict(dataset=name,window=win,channel=ch,n=len(seg),
                    start_mcu_s=seg.mcu_s.iloc[0],end_mcu_s=seg.mcu_s.iloc[-1]+.01,
                    **metrics(seg[ch+'(mV)'].to_numpy())))
        for k in range(0,len(d)-2999,3000):
            seg=d.iloc[k:k+3000]
            for ch in ['Ut1','Ut2']:
                blocks.append(dict(dataset=name,channel=ch,start_mcu_s=seg.mcu_s.iloc[0],**metrics(seg[ch+'(mV)'].to_numpy())))
    for ch in ['Ut1','Ut2']:
        x=new[ch+'(mV)'].to_numpy();n=len(x)//FS;means=x[:n*FS].reshape(n,FS).mean(1)
        slopes=np.array([np.polyfit(np.arange(60),means[k:k+60],1)[0]*60 for k in range(n-59)])
        for threshold in [5,1,.5]:
            ok=abs(slopes)<threshold
            good=[k for k in range(len(ok)-119) if ok[k:k+120].all()]
            stable=[k for k in range(len(ok)-119) if ok[k:].all()]
            settling.append(dict(channel=ch,threshold_mV_min=threshold,
                first_sustained_mcu_s=float(new.mcu_s.iloc[0]+good[0]+60) if good else None,
                sustained_to_end_mcu_s=float(new.mcu_s.iloc[0]+stable[0]+60) if stable else None,
                tail60_slope_mV_min=metrics(x[-6000:])['slope_mV_min']))
    pd.DataFrame(rows).to_csv(OUT/'metrics.csv',index=False,encoding='utf-8-sig')
    pd.DataFrame(blocks).to_csv(OUT/'blocks_30s.csv',index=False,encoding='utf-8-sig')
    pd.DataFrame(settling).to_csv(OUT/'settling.csv',index=False,encoding='utf-8-sig')
    (OUT/'validation.json').write_text(json.dumps([nq,rq,bq],ensure_ascii=False,indent=2),encoding='utf-8')
    fig,axs=plt.subplots(1,2,figsize=(10,4.8),layout='constrained')
    insets=[];display=[]
    late=new.iloc[-6000:]
    for j,ch in enumerate(['Ut1','Ut2']):
        ax=axs[j];x=new[ch+'(mV)'].to_numpy();n=len(x)//FS
        t=(new.mcu_s.iloc[0]+np.arange(n)+.5)/60
        means=x[:n*FS].reshape(n,FS).mean(1)
        ax.plot(t,means,c=COLORS[j],lw=1.5)
        ax.set(xlabel='上电时间（min）',ylabel='原始电压1秒均值（mV）',
               xlim=(0,16),ylim=(1420+20*j,1480+20*j))
        title(ax,chr(97+j),f'HF{j+1}（{ch}）· 修订后')
        event=next(v['first_sustained_mcu_s'] for v in settling if v['channel']==ch and v['threshold_mV_min']==1)
        assert np.all(np.diff(t)>0)
        y=float(np.interp(event/60,t,means))
        ax.plot(event/60,y,'o',color=COLORS[j],markeredgecolor='.3',markersize=5,zorder=4)
        ax.annotate(f'持续 |斜率| < 1 mV/min\n首个合格窗结束\n{event:.1f} s（{event/60:.2f} min）',
                    xy=(event/60,y),xytext=(.18,.43),textcoords='axes fraction',
                    fontsize=9,ha='left',va='center',arrowprops=dict(arrowstyle='->',color='.4',lw=.7))
        seg=new.iloc[-3000:];v=seg[ch+'(mV)'].to_numpy();m=metrics(v)
        inset=ax.inset_axes([.40,.59,.56,.27]);insets.append(inset)
        inset.plot(seg.mcu_s,v,c=COLORS[j],lw=.8)
        inset.axhline(m['mean_mV'],c='.35',lw=.7)
        for value in [m['mean_mV']-m['sd_raw_mV'],m['mean_mV']+m['sd_raw_mV']]:
            inset.axhline(value,c='.5',lw=.6,ls='--')
        center=round(m['mean_mV']*2)/2
        inset.set(ylim=(center-2.5,center+2.5),xlim=(seg.mcu_s.iloc[0],seg.mcu_s.iloc[-1]+.01))
        inset.set_xlabel('上电时间（s）',fontsize=8,labelpad=2)
        inset.set_ylabel('mV',fontsize=8,labelpad=2)
        inset.tick_params(labelsize=8);inset.ticklabel_format(axis='y',style='plain',useOffset=False)
        inset.set_title(f'末30 s · mean={m["mean_mV"]:.3f} mV\nstd={m["sd_raw_mV"]:.3f} · RMS={m["rms_mV"]:.3f} mV',fontsize=8,pad=6)
        display.append(dict(channel=ch,event_mcu_s=event,event_y_mV=y,
                            inset_start_mcu_s=seg.mcu_s.iloc[0],inset_end_mcu_s=seg.mcu_s.iloc[-1]+.01,**m))
    fig.canvas.draw()
    require_matplotlib_panel_alignment(fig,axes=insets,panel_ids=['inset1','inset2'],
        row_groups=[['inset1','inset2']],json_out=OUT/'01-startup-noise.insets-alignment.json',strict=True)
    save(fig,'01-startup-noise')
    pd.DataFrame(display).to_csv(OUT/'combined_display.csv',index=False,encoding='utf-8-sig')
    fig,axs=plt.subplots(2,2,figsize=(8,6),layout='constrained',sharex=True,sharey=True)
    spectra=[]
    for i,(label,d) in enumerate([('修订前',rest.iloc[-6000:]),('修订后',late)]):
        for j,ch in enumerate(['Ut1','Ut2']):
            f,p=psd(d[ch+'(mV)'].to_numpy());mask=f>=.5
            ax=axs[i,j];ax.semilogy(f[mask],p[mask],c=COLORS[j],ls='--' if i==0 else '-',lw=.9)
            for lo,hi in [(18,22),(38,42)]:ax.axvspan(lo,hi,color='.93',zorder=0)
            ax.axvline(33.3,c='.45',ls=':',lw=.7)
            ax.set(xlim=(.5,50),xlabel='频率（Hz）',ylabel='PSD（mV²/Hz）')
            title(ax,chr(97+i*2+j),f'HF{j+1} · {label} · 末60秒')
            spectra.extend(dict(dataset=label,channel=ch,f_Hz=float(ff),psd_mV2_Hz=float(pp)) for ff,pp in zip(f,p))
    save(fig,'03-spectrum')
    pd.DataFrame(spectra).to_csv(OUT/'spectra.csv',index=False,encoding='utf-8-sig')
    print('QUALITY',json.dumps(nq,ensure_ascii=False))
    print('SETTLING',pd.DataFrame(settling).to_string(index=False))
    print('METRICS',pd.DataFrame(rows).to_string(index=False))

if __name__=='__main__':main()
