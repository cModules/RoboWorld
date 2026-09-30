"""可替换边界层：后处理器/机器人库/真机链路/云通道/插件注册表。

依赖方向见 docs/develop/ARCHITECTURE.md：``app → service → domain ← adapter``。
本包刻意**不做 re-export**——公共 API 门面仍是 ``core/__init__.py``，
新代码请直接从本包的具体模块导入。
"""
