"""Reproducible analysis of the two watches; source recordings stay untouched."""
from pathlib import Path
import hashlib
import json
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import numpy as np
import pandas as pd
from scipy import signal

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / 'recordings' / '20260914-热膜噪声蓝牙问题分析'
OUT = Path(__file__).resolve().parent
CHANNELS = ['Ut1', 'Ut2', 'Uc1', 'Uc2']
FS = 100.0
BANDS = {'slow': (.1, .5), 'signal': (.5, 5), 'fast': (5, 45),
         'edge': (45, 50.0001), 'line33': (32.5, 34.2)}


def psd(x, nperseg=1000, fs=FS):
    return signal.welch(x, fs=fs, window='hann', nperseg=nperseg,
                        noverlap=nperseg//2, detrend='linear', scaling='density')


def bandpower(f, p, lo, hi):
    return float(p[(f >= lo) & (f < hi)].sum()*(f[1]-f[0]))


def metrics(x, fs=FS):
    f,p = psd(x, min(int(10*fs), len(x)), fs)
    z = signal.detrend(x)
    out = dict(mean_mV=float(np.mean(x)), std_mV=float(np.std(x, ddof=1)),
               detrended_std_mV=float(np.std(z, ddof=1)),
               span95_mV=float(np.diff(np.percentile(z,[2.5,97.5]))[0]),
               peak_to_peak_mV=float(np.ptp(x)),
               slope_mV_min=float(np.polyfit(np.arange(len(x))/fs, x, 1)[0]*60),
               step_rms_mV=float(np.sqrt(np.mean(np.diff(x)**2)/2)))
    for name,(lo,hi) in BANDS.items():
        if hi <= fs/2+.001:
            out[name+'_rms_mV'] = np.sqrt(bandpower(f,p,lo,hi))
    if fs == FS:
        out['fast_ppm'] = out['fast_rms_mV']/abs(out['mean_mV'])*1e6
        out['line33_fraction_pct'] = 100*(out['line33_rms_mV']/out['fast_rms_mV'])**2
        out['fast_excluding33_rms_mV'] = np.sqrt(max(0,out['fast_rms_mV']**2-out['line33_rms_mV']**2))
        good = (f >= 5) & (f < 45)
        out['dominant_fast_Hz'] = float(f[good][np.argmax(p[good])])
        peaks,_ = signal.find_peaks(p)
        peaks = [i for i in peaks if 5 <= f[i] < 50]
        peaks = sorted(peaks,key=lambda i:p[i],reverse=True)[:5]
        out['top_peaks_Hz'] = ','.join(f'{f[i]:.1f}' for i in peaks)
    return out


def analyze():
    all_metrics, blocks, spectra, evidence = [], [], {}, []
    for path, device, phase, batch in sources():
        d = pd.read_csv(path)
        ident = f'{batch}_{device}_{phase}'
        # No gaps in this dataset: fail visibly if new data violates that premise.
        assert d.ValidFlag.eq(1).all() and d.InterpFlag.eq(0).all()
        assert np.all(np.diff(d.SampleIndex)==1)
        assert np.all(np.diff(d.Seq.to_numpy(dtype=int)) % 65536 == 1)
        duration = len(d)/FS
        slices = {'full': (0,len(d)), 'first60': (0,6000), 'first30': (0,3000),
                  'last60': (len(d)-6000,len(d)), 'seconds30to60': (3000,6000)}
        for label,(a,b) in slices.items():
            for ch in CHANNELS:
                # Uc values are held ~10 samples; block-average to 10 Hz for low-frequency context only.
                x = d[ch+'(mV)'].to_numpy()[a:b]
                fs = FS
                if ch.startswith('Uc'):
                    x = x[:len(x)//10*10].reshape(-1,10).mean(axis=1)
                    fs = 10.0
                m = metrics(x,fs)
                all_metrics.append(dict(id=ident,device=device,phase=phase,batch=batch,
                                        selection=label,channel=ch,start_s=a/FS,end_s=b/FS,**m))
                if label == 'first60' and fs == FS:
                    f,p = psd(x)
                    spectra[ident+'_'+ch] = dict(f=f.tolist(),p=p.tolist())
        for a in range(0,len(d)-999,1000):
            seg = d.iloc[a:a+1000]
            acc = seg[['AccX(g)','AccY(g)','AccZ(g)']].to_numpy()
            acc_rms = np.sqrt(np.mean(np.sum(signal.detrend(acc,axis=0)**2,axis=1)))
            for ch in CHANNELS:
                x=seg[ch+'(mV)'].to_numpy()
                fs=FS
                if ch.startswith('Uc'):
                    x=x.reshape(-1,10).mean(axis=1);fs=10.
                blocks.append(dict(id=ident,device=device,phase=phase,batch=batch,
                                   channel=ch,start_s=a/FS,acc_rms_g=acc_rms,**metrics(x,fs)))
        if batch == '0915':
            seg=d.iloc[:6000]
            x=seg['Ut1(mV)'].to_numpy();y=seg['Ut2(mV)'].to_numpy()
            sos=signal.butter(3,[5,45],fs=100,btype='bandpass',output='sos')
            xf=signal.sosfiltfilt(sos,x)[100:-100];yf=signal.sosfiltfilt(sos,y)[100:-100]
            f, coh=signal.coherence(x,y,fs=100,nperseg=1000,noverlap=500,detrend='linear')
            evidence.append(dict(id=ident,fast_correlation=float(np.corrcoef(xf,yf)[0,1]),
                                 coherence33=float(coh[np.argmin(abs(f-33.3))]),
                                 ut1_uc1_low_correlation=float(np.corrcoef(x.reshape(-1,100).mean(1),seg['Uc1(mV)'].to_numpy().reshape(-1,100).mean(1))[0,1]),
                                 ut2_uc2_low_correlation=float(np.corrcoef(y.reshape(-1,100).mean(1),seg['Uc2(mV)'].to_numpy().reshape(-1,100).mean(1))[0,1])))
    table=pd.DataFrame(all_metrics)
    table.to_csv(OUT/'metrics.csv',index=False,encoding='utf-8-sig')
    pd.DataFrame(blocks).to_csv(OUT/'blocks_10s.csv',index=False,encoding='utf-8-sig')
    pd.DataFrame(evidence).to_csv(OUT/'coupling.csv',index=False,encoding='utf-8-sig')
    (OUT/'spectra.json').write_text(json.dumps(spectra),encoding='utf-8')
    main=table[(table.selection=='first60') & table.channel.str.startswith('Ut')]
    print(main[['id','channel','mean_mV','std_mV','detrended_std_mV','span95_mV',
                'slow_rms_mV','signal_rms_mV','fast_rms_mV','edge_rms_mV',
                'line33_fraction_pct','dominant_fast_Hz','top_peaks_Hz']].to_string(index=False))


def figures():
    from PIL import Image
    skill_scripts = Path.home()/'.codex/skills/scipilot-figure-skill/scripts'
    sys.path.insert(0,str(skill_scripts))
    from visual_qa import audit_layout
    plt.rcParams.update({'font.family':'Microsoft YaHei', 'font.size':9,
                         'axes.unicode_minus':False, 'axes.spines.top':False,
                         'axes.spines.right':False, 'svg.fonttype':'none',
                         'axes.grid':True, 'grid.alpha':.18})
    colors={'论文腕表':'#0072B2','项目腕表':'#D55E00'}
    styles={'论文腕表':'-', '项目腕表':'--'}
    m=pd.read_csv(OUT/'metrics.csv',dtype={'batch':str})
    b=pd.read_csv(OUT/'blocks_10s.csv',dtype={'batch':str})
    spectra=json.loads((OUT/'spectra.json').read_text(encoding='utf-8'))
    data={(device,phase,batch):pd.read_csv(p) for p,device,phase,batch in sources()}
    qa={}

    def save(fig,name):
        fig.savefig(OUT/(name+'.png'),dpi=180)
        qa[name]=[str(i) for i in audit_layout(fig)]
        Image.open(OUT/(name+'.png')).convert('L').save(OUT/(name+'-gray.png'))
        # SVG export can be requested after the previews have been reviewed.
        if '--export' in sys.argv:
            fig.savefig(OUT/(name+'.svg'))
            fig.savefig(OUT/(name+'.png'),dpi=300)
        plt.close(fig)

    fig,axs=plt.subplots(1,2,figsize=(10,4.4),layout='constrained',sharey=True)
    for j,ch in enumerate(['Ut1','Ut2']):
        ax=axs[j]
        for k,phase in enumerate(['稳定静置','佩戴静息']):
            for di,device in enumerate(colors):
                rows=b[(b.batch=='0915')&(b.phase==phase)&(b.device==device)&(b.channel==ch)&(b.start_s<60)]
                x=k+(-.13 if di==0 else .13)
                ax.scatter(x+np.linspace(-.045,.045,len(rows)),rows.fast_rms_mV,
                           color=colors[device],alpha=.5,s=20,marker='o' if di==0 else '^')
                val=m[(m.batch=='0915')&(m.phase==phase)&(m.device==device)&(m.channel==ch)&(m.selection=='first60')].fast_rms_mV.iloc[0]
                ax.plot([x-.07,x+.07],[val,val],color=colors[device],lw=3,label=device if k==0 else None)
                ax.annotate(f'{val:.3f}',(x,val),xytext=(0,10),textcoords='offset points',ha='center',fontsize=9)
        ax.set_title(ch+'：5–45 Hz 快速抖动')
        ax.set_xticks([0,1],['稳定静置','佩戴静息']);ax.set_xlim(-.5,1.5);ax.set_ylim(0,.85)
        ax.set_ylabel('频带 RMS（mV）');ax.legend(loc='upper right')
    fig.suptitle('等长前60秒对比：点为6个连续10秒窗，短横线为整段 Welch RMS',fontsize=11)
    save(fig,'01-noise-comparison')

    fig,axs=plt.subplots(2,2,figsize=(10,7),layout='constrained',sharex=True,sharey=True)
    for i,phase in enumerate(['稳定静置','佩戴静息']):
        for j,ch in enumerate(['Ut1','Ut2']):
            ax=axs[i,j]
            for device in colors:
                sp=spectra[f'0915_{device}_{phase}_{ch}']
                ax.semilogy(sp['f'][1:],sp['p'][1:],color=colors[device],ls=styles[device],lw=1,label=device)
            ax.axvspan(32.5,34.2,color='gray',alpha=.12)
            ax.set_title(f'{phase} · {ch}');ax.set_xlim(0,50);ax.set_ylim(1e-5,100)
            ax.set_ylabel('功率谱密度（mV²/Hz，对数轴）');ax.set_xlabel('频率（Hz）')
            ax.legend(loc='upper right')
    fig.suptitle('原始信号频谱：同一尺度；灰带为32.5–34.2 Hz',fontsize=11)
    save(fig,'02-spectra')

    fig,axs=plt.subplots(2,2,figsize=(10,6.7),layout='constrained',sharex=True,sharey='row')
    for j,ch in enumerate(['Ut1','Ut2']):
        for device in colors:
            d=data[(device,'开机长记录','0915')]
            x=d[ch+'(mV)'].to_numpy();n=len(x)//100
            t=np.arange(n)+.5
            axs[0,j].plot(t,x[:n*100].reshape(-1,100).mean(1)-np.mean(x[:1000]),
                          color=colors[device],ls=styles[device],label=device)
            rows=b[(b.batch=='0915')&(b.phase=='开机长记录')&(b.device==device)&(b.channel==ch)]
            axs[1,j].plot(rows.start_s+5,rows.fast_rms_mV,color=colors[device],ls=styles[device],label=device)
        axs[0,j].set_title(ch+'：开机趋势');axs[0,j].set_ylabel('1秒均值相对首10秒（mV）')
        axs[1,j].set_title(ch+'：快速抖动随时间');axs[1,j].set_ylabel('10秒窗 5–45 Hz RMS（mV）')
        axs[1,j].set_xlabel('各自录制开始后的时间（秒）')
        axs[1,j].set_ylim(bottom=0);axs[0,j].legend(loc='lower left')
    fig.suptitle('开机漂移和快速噪声分开看：记录开始约在 MCU 上电后19–22秒',fontsize=11)
    save(fig,'03-startup')

    fig,axs=plt.subplots(3,2,figsize=(10,8),layout='constrained',sharex=True,sharey='row')
    for j,device in enumerate(colors):
        d=data[(device,'佩戴静息','0915')];n=len(d)//100;t=np.arange(n)+.5
        for i,ch in enumerate(['Ut1','Ut2']):
            x=d[ch+'(mV)'].to_numpy();uc=d[f'Uc{i+1}(mV)'].to_numpy()
            xm=x[:n*100].reshape(-1,100).mean(1);cm=uc[:n*100].reshape(-1,100).mean(1)
            axs[i,j].plot(t,xm-xm[0],color=colors[device],label=ch+' 1秒均值变化')
            axs[i,j].plot(t,(cm-cm[0])*np.mean(x)/np.mean(uc),color='gray',ls=':',label=f'Uc{i+1}变化 × 均值比')
            axs[i,j].set_ylabel('相对首秒变化（mV）');axs[i,j].legend(loc='upper left',fontsize=8)
        acc=d[['AccX(g)','AccY(g)','AccZ(g)']].to_numpy()[:n*100].reshape(n,100,3)
        acc_dev=np.sqrt(np.mean(np.sum((acc-acc.mean(1,keepdims=True))**2,axis=2),axis=1))
        axs[2,j].plot(t,acc_dev*1000,color=colors[device]);axs[2,j].set_ylabel('秒内三轴加速度波动（mg）')
        axs[2,j].set_xlabel('录制时间（秒）');axs[0,j].set_title(device);axs[2,j].set_xlim(0,100)
    fig.suptitle('佩戴段仍有工作点变化：慢漂移不能全部算成电子噪声',fontsize=11)
    save(fig,'04-wearing-drift')

    fig,axs=plt.subplots(2,2,figsize=(10,6),layout='constrained',sharex=True,sharey=True)
    for i,phase in enumerate(['稳定静置','佩戴静息']):
        for j,ch in enumerate(['Ut1','Ut2']):
            ax=axs[i,j]
            for device in colors:
                x=data[(device,phase,'0915')][ch+'(mV)'].to_numpy()[3000:3200]
                ax.plot(np.arange(200)/100,signal.detrend(x),color=colors[device],ls=styles[device],lw=.8,label=device)
            ax.set_title(f'{phase} · {ch}',pad=30);ax.set_xlabel('片段时间（秒）');ax.set_ylabel('去线性趋势的原始电压（mV）')
            ax.legend(loc='lower left',bbox_to_anchor=(0,1.01),ncol=2,fontsize=8,borderaxespad=0)
    fig.suptitle('固定取第30–32秒，无平滑、无择优：直观看快速抖动',fontsize=11)
    save(fig,'05-waveforms')
    (OUT/'figure-qa.json').write_text(json.dumps(qa,ensure_ascii=False,indent=2),encoding='utf-8')


def diagnostics():
    details=[]
    for path,device,phase,batch in sources():
        d=pd.read_csv(path)
        for ch in ['Ut1','Ut2']:
            x=d[ch+'(mV)'].to_numpy()
            n=len(x)//100; means=x[:n*100].reshape(-1,100).mean(1)
            f,p=psd(x[:6000])
            comb=np.zeros(len(f),dtype=bool)
            for center in np.arange(1,7)*100/15:
                comb |= abs(f-center) <= .4
            mask=(f>=5)&(f<45)&(~comb)
            j=int(np.argmax(abs(np.diff(means))))
            detail=dict(device=device,phase=phase,channel=ch,
                        first10_mean=float(np.mean(x[:1000])),last10_mean=float(np.mean(x[-1000:])),
                        mean_change_mV=float(np.mean(x[-1000:])-np.mean(x[:1000])),
                        max_1s_mean_step_mV=float(means[j+1]-means[j]),step_time_s=j+1,
                        off_comb_rms_mV=float(np.sqrt(np.sum(p[mask])*(f[1]-f[0]))),
                        median_off_comb_asd=float(np.sqrt(np.median(p[mask]))))
            f2,p2=psd(x[:6000],512)
            detail['fast_512_mV']=np.sqrt(bandpower(f2,p2,5,45))
            details.append(detail)
    pd.DataFrame(details).to_csv(OUT/'diagnostics.csv',index=False,encoding='utf-8-sig')
    t=np.arange(6000)/100
    test=10+np.sin(2*np.pi*33.3*t)+.4*np.sin(2*np.pi*2*t)
    a=metrics(test)
    assert np.isclose(a['fast_rms_mV'],1/np.sqrt(2),rtol=.002)
    assert np.isclose(a['signal_rms_mV'],.4/np.sqrt(2),rtol=.002)
    assert metrics(np.ones(6000))['fast_rms_mV']<1e-10
    # PSD normalization: integrated density equals Hann-weighted detrended power.
    z=signal.detrend(test[:1000]);w=signal.windows.hann(1000,sym=False)
    f,p=psd(test[:1000]);assert np.isclose(p.sum()*(f[1]-f[0]),np.sum((z*w)**2)/np.sum(w*w))
    (OUT/'validation.json').write_text(json.dumps({'synthetic_33Hz_rms':a['fast_rms_mV'],
        'synthetic_2Hz_rms':a['signal_rms_mV'],'source_recordings':len(list(sources())),
        'gap_duplicate_sequence_checks':'passed','psd_power_normalization':'passed',
        'python':sys.version,'numpy':np.__version__,'pandas':pd.__version__,
        'scipy':__import__('scipy').__version__},indent=2),encoding='utf-8')


def sources():
    for p in sorted(SOURCE.glob('*/*.csv')):
        if '蓝牙射频' in p.parent.name:
            continue  # RF experiments are analyzed separately in the focused revision.
        if p.stem.endswith(('_status', '_rf_events', '_markers')):
            continue
        if p.stem != 'kaiji1_LYX_0915':
            continue  # User scope: today's six recordings only.
        device = '论文腕表' if p.parent.name.startswith('论文') else '项目腕表'
        phase = '佩戴静息' if '佩戴' in p.parent.name else ('稳定静置' if '稳定后' in p.parent.name else '开机长记录')
        batch = '0915' if p.stem.startswith('kaiji') else '0914'
        yield p, device, phase, batch


def inventory():
    rows = []
    for p, device, phase, batch in sources():
        d = pd.read_csv(p)
        s = pd.read_csv(p.with_name(p.stem + '_status.csv'))
        idx = d['SampleIndex'].to_numpy()
        valid = d['ValidFlag'].eq(1) & d['InterpFlag'].eq(0)
        row = dict(id=f'{batch}_{device}_{phase}', path=str(p.relative_to(ROOT)),
                   sha256=hashlib.sha256(p.read_bytes()).hexdigest(), rows=len(d),
                   duration_s=float(d['Time(s)'].iloc[-1]+.01),
                   invalid=int((~valid).sum()), duplicate_indices=int(d['SampleIndex'].duplicated().sum()),
                   missing_indices=int(np.maximum(np.diff(np.unique(idx))-1, 0).sum()),
                   first_mcu_s=float(s['McuTime(ms)'].iloc[0]/1000),
                   last_mcu_s=float(s['McuTime(ms)'].iloc[-1]/1000),
                   time_error_s=float(np.max(np.abs(d['Time(s)']-d['SampleIndex']/100))),
                   reset_count=int((s['McuTime(ms)'].diff()<0).sum()))
        for c in ['TxBusyCounter','TxErrorCounter','AdcErrorCounter','ImuErrorCounter',
                  'PcMissingRaw','PcRawInvalidCandidates']:
            row[c+'_delta'] = int(s[c].iloc[-1]-s[c].iloc[0]) if c in s else None
        for ch in CHANNELS:
            x = d.loc[valid, ch+'(mV)']
            row[ch+'_mean'] = float(x.mean())
            row[ch+'_std'] = float(x.std())
            row[ch+'_unchanged_fraction'] = float(x.diff().eq(0).mean())
        rows.append(row)
    pd.DataFrame(rows).to_csv(OUT/'inventory.csv',index=False,encoding='utf-8-sig')
    print(pd.DataFrame(rows).drop(columns=['path','sha256']).to_string(index=False))


if __name__ == '__main__':
    inventory()
    analyze()
    figures()
    diagnostics()
