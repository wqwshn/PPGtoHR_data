"""Create a portable report with embedded figures and selectable comparison windows."""
from pathlib import Path
import base64
import json
import re

import markdown
import pandas as pd

OUT=Path(__file__).resolve().parent
source=(OUT/'报告.md').read_text(encoding='utf-8')
body=markdown.markdown(source,extensions=['tables','toc'])
for path in OUT.glob('0[1-5]-*.png'):
    if '-gray' not in path.stem:
        uri='data:image/png;base64,'+base64.b64encode(path.read_bytes()).decode('ascii')
        body=body.replace('src="'+path.name+'"','src="'+uri+'"')
m=pd.read_csv(OUT/'metrics.csv')
cols=['device','phase','channel','selection','fast_rms_mV','signal_rms_mV']
data=m[(m.phase!='开机长记录')&m.channel.str.startswith('Ut')][cols].to_dict('records')
control='''<section class="explore"><h2>切换取段，检查结论是否稳定</h2>
<label>对比片段 <select id="window"><option value="first60">前60秒（主结果）</option><option value="first30">前30秒</option><option value="seconds30to60">30–60秒</option><option value="last60">各自末60秒</option><option value="full">各自全段</option></select></label>
<label>指标 <select id="metric"><option value="fast_rms_mV">5–45 Hz 快速抖动 RMS</option><option value="signal_rms_mV">0.5–5 Hz 较慢波动 RMS</option></select></label>
<div id="comparison" aria-live="polite"></div><p class="note">单位 mV。末60秒与全段不对应相同佩戴时长；低频波动包含真实响应及漂移。</p></section>'''
body=body.replace('</h1>','</h1>'+control,1)
script='''<script>
const data=DATA;
function refresh(){
const w=document.getElementById('window').value,k=document.getElementById('metric').value;
let rows='';
for(const phase of ['稳定静置','佩戴静息'])for(const channel of ['Ut1','Ut2']){
const get=device=>data.find(x=>x.phase===phase&&x.channel===channel&&x.device===device&&x.selection===w)[k];
const a=get('论文腕表'),b=get('项目腕表');
rows+=`<tr><td>${phase}</td><td>${channel}</td><td>${a.toFixed(3)}</td><td>${b.toFixed(3)}</td><td>${(b/a).toFixed(2)}×</td></tr>`;
}
document.getElementById('comparison').innerHTML='<table><thead><tr><th>状态</th><th>通道</th><th>论文腕表</th><th>项目腕表</th><th>项目/论文</th></tr></thead><tbody>'+rows+'</tbody></table>';
}
document.getElementById('window').addEventListener('change',refresh);
document.getElementById('metric').addEventListener('change',refresh);refresh();
</script>'''.replace('DATA',json.dumps(data,ensure_ascii=False))
html='''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>两套腕表热膜噪声分析 · 2026-09-15</title>
<style>
body{font:16px/1.8 "Microsoft YaHei",sans-serif;color:#243342;background:#fff;margin:0}
main{max-width:1060px;margin:auto;padding:28px 24px 70px}h1{font-size:28px;line-height:1.45}h2{font-size:23px;margin-top:44px;padding-top:16px;border-top:1px solid #d9e1e8}h3{font-size:19px;margin-top:28px}a{color:#08669b}p,li{max-width:94ch}li{margin:8px 0}table{border-collapse:collapse;width:100%;font-size:14px;margin:20px 0}th,td{text-align:left;padding:9px 10px;border-bottom:1px solid #d9e1e8}th{background:#edf3f7}img{width:100%;height:auto;margin:12px 0}code{font-size:.9em;background:#f1f4f7;padding:2px 4px;overflow-wrap:anywhere}strong{font-weight:650}.explore{padding:2px 0 10px}.explore h2{margin-top:18px}.note{font-size:14px;color:#4b5e6b}label{display:inline-block;margin:8px 24px 8px 0}select{font:inherit;padding:7px;border:1px solid #8d9eaa;border-radius:3px;background:white;max-width:100%}@media(max-width:640px){main{padding:16px 12px}table{font-size:12px}th,td{padding:6px 4px}h1{font-size:24px}}@media print{.explore{display:none}main{max-width:none;padding:0}h2{break-after:avoid}img{break-inside:avoid}body{font-size:11pt}}
</style></head><body><main>'''+body+'</main>'+script+'</body></html>'
assert 'src="0' not in html
assert len(data)==40
(OUT/'报告.html').write_text(html,encoding='utf-8')
print('Report written; 5 embedded figures; 40 metric records; size:',len(html.encode('utf-8')))
