import base64, sys

html = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="GBK">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>R1 机器人控制代码框架文档</title>
</head>
<body>
<h1>R1 Project</h1>
</body>
</html>
"""
with open(r"e:\EK\R1\R1_Code_Framework.html", "wb") as f:
    f.write(html.encode("gbk"))
print("OK")
