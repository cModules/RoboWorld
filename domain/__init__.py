"""领域层：机器人模型/FK-IK/位姿/连杆/碰撞/动力学/包络/场景。

依赖方向见 docs/develop/ARCHITECTURE.md：``app → service → domain ← adapter``。
本包刻意**不做 re-export**——公共 API 门面仍是 ``core/__init__.py``，
新代码请直接从本包的具体模块导入。
"""
