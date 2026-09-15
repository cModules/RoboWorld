# orolp-plugin-acme-post

RobotWorld 的**第一方示例插件包**：一个真正可以 `pip install` 的插件，用于验证
`orolp.plugins` 从 `importlib.metadata` 发现插件的真实路径（而非测试注入的假入口点）。

```bash
pip install examples/acme_post_plugin
python - <<'PY'
from orolp.plugins import PluginRegistry
reg = PluginRegistry()
print(reg.load_entry_points())       # {'discovered': 1, 'registered': 1, ...}
print(reg.get("acme_v2").origin)     # entry_point
PY
```

装好后 `program.to_post("acme_v2")` 与内置 7 品牌走同一条出码通路。

要点：
- 入口点组 `orolp.plugins`，加载结果为提供函数（返回 `PluginSpec` 列表）
- 后处理工厂用 `"module:attr"` 字符串延迟导入
- `orolp` 由宿主环境提供，不写进 `dependencies`
