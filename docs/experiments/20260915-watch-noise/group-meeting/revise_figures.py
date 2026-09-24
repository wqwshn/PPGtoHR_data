"""Group-meeting figures with common Arial/SimSun typography."""
import analyze_motion as m
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib import font_manager

def run():
    for family in ['Arial','SimSun']:font_manager.findfont(family,fallback_to_default=False)
    b=m.base;b.load()
    plt.rcParams.update({'font.family':['Arial','SimSun'],'font.size':10,'axes.titlesize':11,'legend.fontsize':9})
    fig,axs=plt.subplots(1,2,figsize=(8,3.4),layout='constrained')
    for j,ch in enumerate(b.CHANNELS):
        for dev,c in b.COLORS.items():
            d=b.primary(dev,'开机')['d'];x=d[ch+'(mV)'].to_numpy();n=len(x)//100
            axs[j].plot(np.arange(n)+.5+d.mcu_s.iloc[0],x[:n*100].reshape(n,100).mean(1),c=c,label=dev,lw=1.1)
        axs[j].set(xlabel='上电时间（s）',ylabel='1秒均值（mV）',title=f'{chr(97+j)}  {ch}');axs[j].legend()
    m.save(fig,'04-startup-trend')
    fig,axs=plt.subplots(4,2,figsize=(8,9),layout='constrained',sharex=True)
    limits=[]
    for i,(phase,dev) in enumerate([(p,d) for p in ['静置','佩戴'] for d in b.COLORS]):
        for j,ch in enumerate(b.CHANNELS):
            ax=axs[i,j];x=b.primary(dev,phase)['d'][ch+'(mV)'].to_numpy()[1000:1200];mu=x.mean();sd=x.std(ddof=1)
            ax.plot(np.arange(200)/100+10,x,c=b.COLORS[dev],lw=.8)
            for v,ls in [(mu,'-'),(mu-sd,'--'),(mu+sd,'--')]:ax.axhline(v,c='.4',ls=ls,lw=.7)
            ax.set_title(f'{chr(97+i*2+j)}  {dev} · {phase} · {ch}',loc='left',fontsize=10)
            ax.set_ylabel('电压（mV）');ax.ticklabel_format(axis='y',style='plain',useOffset=False)
            ax.text(.02,.95,f'μ={mu:.2f}, σ={sd:.2f} mV',transform=ax.transAxes,va='top',fontsize=9,bbox=dict(facecolor='white',alpha=.85,edgecolor='none',pad=1))
            limits.append((ax,mu,max(abs(x-mu).max()*1.2,2)))
    extent=max(v[2] for v in limits)
    for ax,mu,_ in limits:ax.set_ylim(mu-extent,mu+extent)
    for ax in axs[-1]:ax.set_xlabel('记录时间（s）')
    fig.legend(handles=[Line2D([],[],c='.4',label='均值 μ'),Line2D([],[],c='.4',ls='--',label='μ ± 标准差 σ')],loc='outside upper center',ncol=2)
    m.save(fig,'05-raw')
    fig,axs=plt.subplots(1,2,figsize=(8,3.6),layout='constrained',sharey=True)
    for j,ch in enumerate(b.CHANNELS):
        for k,phase in enumerate(['静置','佩戴']):
            for di,(dev,c) in enumerate(b.COLORS.items()):
                x=b.primary(dev,phase)['d'][ch+'(mV)'].to_numpy()[:3000];v=b.metric(x)['rms_mV']
                vv=[b.metric(x[a:a+1000])['rms_mV'] for a in range(0,3000,1000)];pos=k+(-.15 if di==0 else .15)
                axs[j].scatter(pos+np.linspace(-.035,.035,3),vv,c=c,marker='o' if di==0 else '^',s=25)
                axs[j].plot([pos-.07,pos+.07],[v,v],c=c,lw=2);axs[j].text(pos,max(vv)+.035,f'{v:.3f}',ha='center',fontsize=9)
        axs[j].set(xticks=[0,1],xticklabels=['静置','佩戴静息'],ylim=(0,.9),ylabel='0.5–50 Hz RMS（mV）',title=f'{chr(97+j)}  {ch}')
    fig.legend(handles=[Line2D([],[],c=c,marker='o' if i==0 else '^',ls='',label=d+'：每点10秒') for i,(d,c) in enumerate(b.COLORS.items())]+[Line2D([],[],c='.4',label='短横线：30秒RMS')],loc='outside upper center',ncol=2)
    m.save(fig,'06-rms')
    fig,axs=plt.subplots(6,2,figsize=(8,11),layout='constrained',sharex=True,sharey=True)
    for i,(dev,run,title) in enumerate([('论文腕表',1,'旧器件 · PPG开'),('项目腕表',1,'新器件 · PPG开'),('项目腕表',2,'新器件 · PPG关')]):
        item=b.DATA[(dev,'复位',run)]
        for j,ch in enumerate(b.CHANNELS):
            for state_index,(event,c,ls) in enumerate([('NORMAL_START',b.COLORS[dev],'-'),('RESET_ASSERT',b.COLORS[dev],'--')]):
                ax=axs[2*i+state_index,j]
                powers=[]
                for cy in [1,2,3]:
                    ev=item['e'][(item['e'].Event==event)&(item['e'].Cycle==cy)].iloc[0];d=item['d']
                    x=d[(d.sample_counter>=ev.SampleCounter+500)&(d.sample_counter<ev.SampleCounter+2500)][ch+'(mV)'].to_numpy()
                    f,p=b.psd(x);powers.append(p)
                mask=f>=.5;ax.semilogy(f[mask],np.mean(powers,axis=0)[mask],c=c,ls=ls,lw=.9)
                for lo,hi in [(18,22),(38,42)]:ax.axvspan(lo,hi,color='.9',zorder=0)
                ax.axvline(33.3,c='.5',ls=':',lw=.7)
                ax.set(xlim=(.5,50),ylim=(1e-5,4),ylabel='PSD（mV²/Hz）',yticks=[1e-5,1e-3,1e-1])
                state='连接' if state_index==0 else '保持复位'
                ax.set_title(f'{chr(97+4*i+2*state_index+j)}  {title} · {ch} · {state}',loc='left',fontsize=9)
    for ax in axs[-1]:ax.set_xlabel('频率（Hz）')
    fig.legend(handles=[Line2D([],[],c=b.COLORS['论文腕表'],lw=2,label='论文腕表（旧器件）'),Line2D([],[],c=b.COLORS['项目腕表'],lw=2,label='项目腕表（新器件）'),Line2D([],[],c='.3',ls='-',label='蓝牙连接'),Line2D([],[],c='.3',ls='--',label='蓝牙保持复位')],loc='outside upper center',ncol=2)
    m.save(fig,'07-spectrum-composite')

if __name__=='__main__':run()
