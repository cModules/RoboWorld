"""跨层共享支撑层（Support）：被各层依赖的最底层工具，**不含业务语义**。

当前内容：``i18n``（国际化设施）。它必须待在所有层的下面——Domain / Service /
Adapter 抛出的错误与提示消息也要本地化（20 处 ``from support.i18n import tr``），
放在任何一层都会造成反向依赖。

**本层不变量**（守护脚本 H8 强制）：
* 只允许依赖标准库与第三方；**禁止 import** app / domain / service / adapter / core；
* 新增内容必须同时满足"跨层共享 + 无业务语义"，单层专用的工具请放回该层；
* 本层不是回收站：拿不准就按 AGENTS.md #22 停下来问。
"""

#: 产品版本。**版本号唯一的代码来源**——`pyproject.toml` 用 `dynamic/attr` 从这里派生，
#: `packaging/app.spec` 的 CFBundleShortVersionString 也读它。禁止改成反查
#: `importlib.metadata`：那会破坏 setuptools 的静态解析，反过来要求先安装。
__version__ = "0.1.40"
