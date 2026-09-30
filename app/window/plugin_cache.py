"""窗口级「插件覆盖层 + 结果缓存」操作（ADR-002 拆解第 3 步）。

从 ``main.py`` 的 ``MainWindow`` 原样搬出（方法体未改一字）。这一组是窗口的
**外部集成与可观测性**：插件覆盖层（v1.0.4）与三类结果缓存的命中情况（v1.2.4）
——两者都是"点开看一眼"的诊断入口，不参与编辑状态机。

两处与 ``main.py`` 版本的**唯一**差别（都不是行为改动）：

1. 依赖探测在本模块内独立完成（静默降级，不重复打印 ``[warning]``——``main.py``
   已就同一次缺失打过一条）；
2. 原 ``_HAS_WORLD and hasattr(self, "scene_panel")`` 简化为
   ``hasattr(self, "scene_panel")``：``main.py`` 只在对应标志为真时才赋这两个
   属性（见 setup_ui 里的 ``if _HAS_WORLD:`` / ``if _HAS_PROGRAM:``），从不置
   ``None``，因此 ``hasattr`` 蕴含该标志，短路结果完全一致。
"""

from __future__ import annotations

from PySide6 import QtWidgets

from app.logging import logger
from app.window.capabilities import (
    OverlayContext,
    _HAS_INCREMENTAL,
    _HAS_PLUGINS,
    cache_overview,
    clear_all_caches,
    load_entry_points,
    plugin_registry,
    reset_all_cache_stats,
    tr,
)

from support.i18n import tr


# 插件接口（v1.0.4）：渲染覆盖层 + 第三方后处理/运动学
# 缓存可观测性（v1.2.4）：三类结果缓存的统一快照
class PluginCacheMixin:
    """插件覆盖层与三类缓存的可观测性槽（宿主：``MainWindow``）。"""

    def _refresh_plugin_overlays(self, current_index: int = -1) -> None:
        """把当前轨迹交给渲染插件并重绘覆盖层。

        刻意**不**放在每帧回调里：``collect_overlays`` 每次都会构造插件对象，
        按帧调用等于每 16ms 构造一次第三方代码——插件里稍微有点初始化开销就会
        拖垮帧率。轨迹变化的时机（校验完成 / 加载演示 / 手动刷新）才是正确的
        刷新点。
        """
        if not _HAS_PLUGINS:
            return
        traj = None
        # 只判 hasattr：宿主窗口仅在对应依赖可用时才创建这两个面板（见其 setup_ui
        # 的 ``if _HAS_WORLD:`` / ``if _HAS_PROGRAM:``），从不置 None——所以
        # hasattr 已蕴含原 main.py 的 _HAS_WORLD / _HAS_PROGRAM 标志，短路结果一致。
        if hasattr(self, "program_panel"):
            traj = getattr(self.program_panel, "trajectory", None)
        tolerance = 1.0
        try:
            if hasattr(self, "scene_panel"):
                from service.planner import ENVELOPE_TOLERANCE_MM

                tolerance = float(ENVELOPE_TOLERANCE_MM)
        except Exception:
            pass
        robot_name = getattr(getattr(self, "olp_robot", None), "name", "") or ""
        try:
            ctx = OverlayContext.from_trajectory(
                traj,
                current_index=current_index,
                tolerance_mm=tolerance,
                robot_name=robot_name,
                extra={"has_scene": bool(self.gl_widget.scene is not None)},
            )
        except Exception as exc:
            logger.warning("overlay_context_failed: %s", exc)
            return
        self.gl_widget.set_overlay_context(ctx)
        self.gl_widget.refresh_plugin_overlays()

    def toggle_plugin_overlays(self, checked: bool) -> None:
        """开关插件覆盖层显示。"""
        self.gl_widget.show_plugin_overlays = bool(checked)
        if checked:
            self._refresh_plugin_overlays()
        self.gl_widget.update()

    def reload_plugins(self) -> None:
        """重新扫描已安装包的插件入口点（装了新插件不必重启）。"""
        if not _HAS_PLUGINS:
            return
        try:
            stats = load_entry_points()
        except Exception as exc:
            QtWidgets.QMessageBox.warning(
                self, tr("插件"), tr("重新扫描失败：{exc}").format(exc=exc)
            )
            return
        self._refresh_plugin_overlays()
        QtWidgets.QMessageBox.information(
            self,
            tr("插件"),
            tr(
                "重新扫描完成：发现 {discovered} 个入口点，登记 {registered} 个，"
                "跳过 {skipped} 个，失败 {failed} 个。"
            ).format(**stats),
        )

    def show_plugin_status(self) -> None:
        """显示插件注册表现状（含失败原因——插件生态里"为什么没生效"最要紧）。"""
        if not _HAS_PLUGINS:
            QtWidgets.QMessageBox.information(
                self, tr("插件"), tr("当前环境无插件层（adapter.plugins 不可用）。")
            )
            return
        text = plugin_registry.summary()
        if self.gl_widget.plugin_overlay_errors:
            text += tr("\n  最近一次渲染覆盖层的问题:\n")
            for msg in self.gl_widget.plugin_overlay_errors:
                text += f"    ! {msg}\n"
        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle(tr("插件状态"))
        box.setText(text)
        box.setDetailedText(text)
        box.exec()

    @staticmethod
    def _cache_status_text() -> str:
        """把 :func:`service.incremental.cache_overview` 排成可读文本。

        全局关闭的缓存仍占一行如实写"已关闭"——看不见的关闭最容易被
        当成"没这回事"。
        """
        lines = []
        for row in cache_overview():
            kind = row["kind"]
            if kind == "plan":
                lines.append(tr("规划与分段缓存（TrajectoryCache）"))
            elif kind == "collision":
                lines.append(tr("干涉检查结果缓存（CollisionResultCache）"))
            else:
                lines.append(tr("力矩校验结果缓存（DynamicsResultCache）"))
            if not row.get("enabled"):
                lines.append("  " + tr("已关闭"))
                lines.append("")
                continue
            if kind == "plan":
                lines.append(
                    "  "
                    + tr(
                        "整程序 {entries}/{max} 份（{wp} 路点） · "
                        "命中 {hits} / 未命中 {misses}（命中率 {rate:.0%}）"
                    ).format(
                        entries=row["plan_entries"],
                        max=row["max_entries"],
                        wp=row["plan_waypoints"],
                        hits=row["plan_hits"],
                        misses=row["plan_misses"],
                        rate=row["plan_hit_rate"],
                    )
                )
                lines.append(
                    "  "
                    + tr(
                        "分段 {entries}/{wp_max} 条（{wp} 路点） · "
                        "命中 {hits} / 未命中 {misses}（命中率 {rate:.0%}）"
                    ).format(
                        entries=row["segment_entries"],
                        wp_max=row["segment_max_waypoints"],
                        wp=row["segment_waypoints"],
                        hits=row["segment_hits"],
                        misses=row["segment_misses"],
                        rate=row["segment_hit_rate"],
                    )
                )
                lines.append("  " + tr("LRU 淘汰 {n} 次").format(n=row["evictions"]))
            else:
                lines.append(
                    "  "
                    + tr(
                        "条目 {entries}/{max} · 命中 {hits} / 未命中 {misses}"
                        "（命中率 {rate:.0%}）· 存储 {stores} · 淘汰 {evictions}"
                    ).format(
                        entries=row["entries"],
                        max=row["max_entries"],
                        hits=row["hits"],
                        misses=row["misses"],
                        rate=row["hit_rate"],
                        stores=row["stores"],
                        evictions=row["evictions"],
                    )
                )
            lines.append("")
        if not any(
            r.get("hits")
            or r.get("plan_hits")
            or r.get("segment_hits")
            or r.get("misses")
            or r.get("plan_misses")
            or r.get("segment_misses")
            for r in cache_overview()
        ):
            lines.append(tr("本轮还没有缓存活动（计数全为 0）。"))
        return "\n".join(lines).rstrip()

    def show_cache_status(self) -> None:
        """显示三类结果缓存的占用与命中情况（v1.2.4）。"""
        if not _HAS_INCREMENTAL:
            QtWidgets.QMessageBox.information(
                self, tr("缓存状态"), tr("当前环境无缓存层（service.incremental 不可用）。")
            )
            return
        text = self._cache_status_text()
        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle(tr("缓存状态"))
        box.setText(tr('内容寻址缓存不会给出过期结果；这里的数字解释"活少干了多少"。'))
        box.setInformativeText(text)
        box.setDetailedText(text)
        box.exec()

    def clear_caches(self) -> None:
        """清空三类缓存条目并把统计归零（用户要求的"强制全量重算"）。"""
        if not _HAS_INCREMENTAL:
            return
        clear_all_caches()
        reset_all_cache_stats()
        self.statusBar().showMessage(tr("已清空全部缓存并把统计归零——下次校验将全量真算。"), 8000)
