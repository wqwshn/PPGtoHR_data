"""Reproducible motion median comparison and paired wearing spectral check."""
from pathlib import Path
import sys, json, hashlib
import numpy as np
import pandas as pd
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent
BASE=OUT.parent/'focused'
ROOT=OUT.parents[3]
SRC=ROOT/'recordings/20260914-热膜噪声蓝牙问题分析'
sys.path.insert(0,str(BASE))
import analyze as base
plt.rcParams.update({'font.family':['Arial','SimSun'],'font.size':10,
 'axes.spines.top':False,'axes.spines.right':False,'legend.frameon':False,
 'svg.fonttype':'none','pdf.fonttype':42,'axes.unicode_minus':False})
C=['#477DA5','#BC7966']

def save(fig,name):
    from matplotlib.text import Text
    for artist in fig.findobj(Text):
        artist.set_text(artist.get_text().replace('论文腕表','旧器件').replace('项目腕表','新器件').replace('论文器件','旧器件').replace('项目器件','新器件').replace('旧器件（旧器件）','旧器件').replace('新器件（新器件）','新器件'))
    for ext in ['png','svg','pdf']:fig.savefig(OUT/(name+'.'+ext),dpi=300)
    plt.close(fig)

def compare(a,b):
    ac=a-a.mean();bc=b-b.mean()
    return dict(r=float(np.corrcoef(a,b)[0,1]),rmse_mV=float(np.sqrt(np.mean((a-b)**2))),
        nrmse_pct=float(100*np.sqrt(np.mean((a-b)**2))/np.std(a)),
        rms_ratio=float(np.sqrt(np.mean(b*b)/np.mean(a*a))),
        regression_gain=float(np.dot(ac,bc)/np.dot(ac,ac)))

def run():
    p=next((SRC/'项目腕表佩戴后运动产生响应').rglob('kaiji1_LYX_0915.csv'))
    d=pd.read_csv(p);s=pd.read_csv(p.with_name(p.stem+'_status.csv'))
    assert d.ValidFlag.eq(1).all() and d.InterpFlag.eq(0).all()
    assert (np.diff(d.SampleIndex)==1).all() and (np.diff(d.Seq.astype(int))%65536==1).all()
    assert np.allclose(d['Time(s)'],d.SampleIndex/100)
    assert d.Median3Valid.iloc[:2].eq(0).all() and d.Median3Valid.iloc[2:].eq(1).all()
    for ch in ['Uc1','Uc2','Ut1','Ut2']:
        expected=d[ch+'(mV)'].rolling(3).median()
        assert np.allclose(expected,d[ch+'_Median3(mV)'],equal_nan=True,atol=1e-8)
    errors={c:int(s[c].iloc[-1]-s[c].iloc[0]) for c in ['TxBusyCounter','TxErrorCounter','AdcErrorCounter','ImuErrorCounter','PcMissingRaw','PcRawInvalidCandidates']}
    assert all(v==0 for v in errors.values())
    t=d['Time(s)'].to_numpy()[2:];n=len(t)
    gyro=d[['GyroX(dps)','GyroY(dps)','GyroZ(dps)']].to_numpy()[2:]
    speed=np.linalg.norm(gyro-np.median(gyro,axis=0),axis=1)
    starts=np.arange(500,n-2500,100)
    motion=int(max(starts,key=lambda k:np.mean(speed[k:k+2000]**2)))
    quiet=int(min(starts,key=lambda k:np.mean(speed[k:k+1000]**2)))
    sos=signal.butter(4,[.5,5],btype='bandpass',fs=100,output='sos')
    series={};rows=[];bands=[]
    for ch in ['Ut1','Ut2']:
        x=d[ch+'(mV)'].to_numpy()[2:];m=d[ch+'_Median3(mV)'].to_numpy()[2:]
        a=signal.sosfiltfilt(sos,x);b=signal.sosfiltfilt(sos,m)
        series[ch]=(x,m,a,b)
        windows=[('全段去边缘5秒',500,n-500),('运动20秒',motion,motion+2000),('低运动10秒',quiet,quiet+1000),
                 ('全段去边缘3秒',300,n-300),('全段去边缘10秒',1000,n-1000)]
        windows += [(f'连续10秒_{k/100:.0f}',k,k+1000) for k in range(500,n-1500+1,1000)]
        for name,lo,hi in windows:
            for shift in [0,1]:
                rows.append(dict(channel=ch,window=name,start_s=t[lo],end_s=t[hi-1]+.01,
                    median_advance_samples=shift,**compare(a[lo:hi],b[lo+shift:hi+shift])))
            for kind,xx in [('raw',x),('median3',m)]:
                f,ps=base.psd(xx[lo:hi])
                bands.append(dict(channel=ch,window=name,signal=kind,
                    rms_05_50=base.rms(f,ps),rms_5_50=base.rms(f,ps,5.1,50),
                    rms_33=base.rms(f,ps,32.5,34.2)))
    pd.DataFrame(rows).to_csv(OUT/'motion_metrics.csv',index=False,encoding='utf-8-sig')
    pd.DataFrame(bands).to_csv(OUT/'motion_band_metrics.csv',index=False,encoding='utf-8-sig')
    pd.DataFrame({'time_s':t,**{ch+'_'+k:v for ch,vals in series.items() for k,v in zip(['raw','median3','raw_bp','median3_bp'],vals)}}).to_csv(OUT/'motion_processed.csv',index=False)
    fig,axs=plt.subplots(3,2,figsize=(7.2,7),layout='constrained')
    for j,ch in enumerate(series):
        x,m,a,b=series[ch]
        for k,(u,v,label) in enumerate([(x,m,'原始电压（mV）'),(a,b,'0.5–5 Hz（mV）')]):
            ax=axs[k,j];mask=(t>=t[motion])&(t<t[motion]+20)
            ax.plot(t[mask],u[mask],c=C[0],lw=.65,label='raw' if k==0 else 'raw → 带通')
            ax.plot(t[mask],v[mask],c=C[1],lw=.8,ls='-' if k==0 else '--',label='中值3' if k==0 else '中值3 → 带通')
            ax.set(xlabel='记录时间（s）',ylabel=label);ax.legend(loc='upper right' if j==0 else 'upper left',fontsize=9)
            ax.set_title(f'{chr(97+2*k+j)}  {ch} · 运动段',loc='left',fontsize=11)
        ax=axs[2,j];mask=(t>=t[motion])&(t<t[motion]+20)
        ax.plot(a[mask],b[mask],'.',c=C[0],ms=1,alpha=.3,rasterized=True)
        lim=[min(a[mask].min(),b[mask].min()),max(a[mask].max(),b[mask].max())]
        ax.plot(lim,lim,c='.4',lw=.7,ls=':');ax.set(xlabel='raw → 带通（mV）',ylabel='中值3 → 带通（mV）')
        rr=compare(a[mask],b[mask]);ax.set_title(f'{chr(101+j)}  {ch} · 运动段 r={rr["r"]:.4f}',loc='left',fontsize=11)
    save(fig,'01-motion-filter')
    fig,axs=plt.subplots(2,2,figsize=(7.2,4.6),layout='constrained')
    mask=(t>=t[motion])&(t<t[motion]+2)
    for j,ch in enumerate(series):
        x,m,a,b=series[ch]
        for k,(u,v) in enumerate([(x,m),(a,b)]):
            ax=axs[k,j];ax.plot(t[mask],u[mask],c=C[0],lw=1,label='raw' if k==0 else 'raw → 带通')
            ax.plot(t[mask],v[mask],c=C[1],ls='--',lw=1,label='中值3' if k==0 else '中值3 → 带通')
            ax.set(xlabel='记录时间（s）',ylabel='原始电压（mV）' if k==0 else '0.5–5 Hz（mV）')
            ax.set_title(f'{chr(97+k*2+j)}  {ch}',loc='left',fontsize=11);ax.legend(fontsize=9)
    save(fig,'02-motion-detail')
    base.load();wear=[]
    for dev in ['论文腕表','项目腕表']:
        for phase in ['静置','佩戴']:
            for runid in ([1,2] if dev=='项目腕表' and phase=='静置' else [1]):
                dd=base.DATA[(dev,phase,runid)]['d']
                for ch in ['Ut1','Ut2']:
                    for length in [30,60]:
                        f,ps=base.psd(dd[ch+'(mV)'].to_numpy()[:length*100])
                        wear.append(dict(device=dev,phase=phase,run=runid,channel=ch,seconds=length,rms_33=base.rms(f,ps,32.5,34.2)))
    w=pd.DataFrame(wear);w.to_csv(OUT/'wearing_33hz.csv',index=False,encoding='utf-8-sig')
    fig,axs=plt.subplots(1,2,figsize=(7.2,3),layout='constrained',sharey=True)
    for j,ch in enumerate(['Ut1','Ut2']):
        for i,dev in enumerate(['论文腕表','项目腕表']):
            q=w[(w.device==dev)&(w.channel==ch)&(w.run==1)&(w.seconds==30)].set_index('phase')
            v=q.loc[['静置','佩戴'],'rms_33'].to_numpy();axs[j].plot([0,1],v,c=C[i],marker='o' if i==0 else '^',label=dev)
            for xx,yy in zip([0,1],v):
                offset=(4,-14) if xx==0 and i==0 else (4,7)
                axs[j].annotate(f'{yy:.3f}',(xx,yy),xytext=offset,textcoords='offset points',fontsize=9,color=C[i])
        axs[j].set(xticks=[0,1],xticklabels=['静置','佩戴静息'],xlim=(-.2,1.3),ylim=(0,.65),ylabel='32.5–34.2 Hz RMS（mV）')
        axs[j].set_title(f'{chr(97+j)}  {ch} · 同次开机配对',loc='left',fontsize=11)
    handles,labels=axs[0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='outside upper center',ncol=2,fontsize=10)
    save(fig,'03-wearing-33hz')
    qa=dict(samples=len(d),duration_s=len(d)/100,sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
      source=str(p.relative_to(ROOT)),median_recomputation='all four channels passed',errors=errors,
      motion_start_s=float(t[motion]),quiet_start_s=float(t[quiet]),
      gyro_rms_motion=float(np.sqrt(np.mean(speed[motion:motion+2000]**2))),
      gyro_rms_quiet=float(np.sqrt(np.mean(speed[quiet:quiet+1000]**2))))
    (OUT/'validation.json').write_text(json.dumps(qa,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(qa,ensure_ascii=False));print(pd.DataFrame(rows).query('median_advance_samples==0').to_string(index=False))

if __name__=='__main__':run()
