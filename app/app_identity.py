"""应用身份常量：三端互认的**同一个**身份。

Windows 的 AppUserModelID（任务栏分组 + Jump List 归属）、macOS 的
``CFBundleIdentifier``（LaunchServices 判定"这个文件归谁打开"）、以及本机
单实例 socket 名，必须是同一个字符串：

* 各写一份 → 任务栏把两个窗口分成两组、Jump List 的"最近"跑到别的条目下、
  文件关联指向一个不存在的身份——三种都是"看起来是随机的"事故；
* 真源在这里。``packaging/app.spec`` 的 ``bundle_identifier`` 由
  ``tests/test_packaging_spec.py`` 断言与 :data:`APPLICATION_ID` 一致，
  改一处忘了另一处时测试先报警（与 spec 里"仓库根算错"同一条纪律）。

为什么不放 ``support/``：support 是被各层依赖的跨层共享支撑（i18n/version），
而应用身份只有 App 层与打包脚本用；放下去等于让最底层替上层保管一个名字。
"""

from __future__ import annotations

__all__ = [
    "APPLICATION_ID",
    "PROJECT_MIME_TYPE",
    "PROJECT_PROG_ID",
    "PROJECT_TYPE_NAME",
    "PROJECT_UTI",
]


#: 应用标识（Windows AppUserModelID / macOS bundle id / 单实例 socket 名同源）
APPLICATION_ID = "com.openrobot.work.robotworld"

#: Windows 外壳里的工程文件类型（ProgID）。``.1`` 是版本号位：将来格式换了
#: 可以并存两个 ProgID，老文件仍指向老的那个
PROJECT_PROG_ID = "RoboWorld.Project.1"

#: macOS 的工程文件 UTI（LaunchServices 用它判定"这个文件归谁打开"）。
#:
#: 由 :data:`APPLICATION_ID` 派生而不是另起一个名字：UTI 要求反写域名前缀，
#: 而 bundle id 已经是了——两者分家的后果是"应用声明自己能开 A 类型，
#: 系统却把文件认成 B 类型"，双击仍会问"用什么打开"。
PROJECT_UTI = f"{APPLICATION_ID}.project"

#: Linux（XDG shared-mime-info）的 MIME 类型。桌面文件与 MIME XML 靠它对上
PROJECT_MIME_TYPE = "application/x-roboworld-project"

#: 文件管理器里显示的类型名（Windows「类型」列 / macOS「种类」/ Linux 属性）。
#:
#: **故意不走 ``tr()``**：这个值由安装器**一次性写进系统**（注册表 / plist /
#: MIME 数据库），而应用界面语言可在运行时切换。用 ``tr()`` 会把"安装那一刻的
#: 语言"永久冻在系统里，用户换了界面语言它也不会跟着变——它不是我们渲染的
#: 文案，而是交给操作系统的一段数据，语言中立最稳。
PROJECT_TYPE_NAME = "RoboWorld Project"
