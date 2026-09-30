"""业务用例层：路径规划/工艺/抓放/标定/双机/程序/工程。

依赖方向见 docs/develop/ARCHITECTURE.md：``app → service → domain ← adapter``。
本包刻意**不做 re-export**——公共 API 门面仍是 ``core/__init__.py``，
新代码请直接从本包的具体模块导入。
"""
