"""Portable focused report; scientific figures are produced only by matplotlib."""
from pathlib import Path
import base64
import markdown

OUT=Path(__file__).resolve().parent
body=markdown.markdown((OUT/'报告.md').read_text(encoding='utf-8'),extensions=['tables'])
for path in sorted(OUT.glob('0*.png')):
    if path.stem.endswith('-preview'):
        continue
    uri='data:image/png;base64,'+base64.b64encode(path.read_bytes()).decode('ascii')
    body=body.replace('src="'+path.name+'"','src="'+uri+'"')
assert body.count('data:image/png;base64,')==8
style='''body{margin:0;background:#fff;color:#27333d;font:16px/1.85 "Microsoft YaHei",sans-serif}main{max-width:1040px;padding:28px 24px 60px;margin:auto}h1{font-size:28px}h2{font-size:22px;margin-top:42px;padding-top:12px;border-top:1px solid #d8e0e5}img{width:100%;height:auto}p,li{max-width:94ch}li{margin:8px 0}a{color:#477da5}table{border-collapse:collapse;font-size:14px;width:100%;margin:18px 0}th,td{padding:9px;border-bottom:1px solid #d8e0e5;text-align:left}th{background:#eef2f5}strong{font-weight:650}@media(max-width:620px){main{padding:16px 12px}h1{font-size:24px}table{font-size:12px}th,td{padding:5px}}@media print{main{max-width:none;padding:0}body{font-size:11pt}h2{break-after:avoid}img{break-inside:avoid}}'''
html='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>热膜噪声 · 聚焦修订与复位对照</title><style>'+style+'</style><main>'+body+'</main></html>'
(OUT/'报告.html').write_text(html,encoding='utf-8')
print('Portable report: eight figures embedded.')
