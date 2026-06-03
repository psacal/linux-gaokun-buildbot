#!/usr/bin/env python3
"""ssc-bridge 传感器桥接调参与状态监控 GUI

GTK4 + Adwaita, 提供:
  1. 运行状态 — ssc-bridge 是否运行、是否启用、自动启动
  2. 开关控制 — 启动/停止 ssc-bridge
  3. 参数调校 — 设备名、输出缩放、mount matrix
  4. 实时姿态 — 加速度计/陀螺仪/光照传感器数值与方向

参考: EGoTouchRev-Linux (chiyuki0325), linux-gaokun-buildbot (KawaiiHachimi)
"""

import math
import os
import re
import subprocess
import sys
import tempfile
import uuid
from pathlib import Path

import gi

gi.require_version("Gtk", "4.0")
gi.require_version("Adw", "1")
from gi.repository import Adw, GLib, Gtk, Pango

# ── Constants ────────────────────────────────────────────────────────

SERVICE_NAME = "ssc-bridge"
CONFIG_FILE = Path("/etc/default/ssc-bridge")
DATA_DIR = Path("/run/ssc-bridge")
ACCEL_FILE = DATA_DIR / "accel"
GYRO_FILE = DATA_DIR / "gyro"
LIGHT_FILE = DATA_DIR / "light"

POLL_INTERVAL_MS = 200

DEFAULT_CONFIG = {
    "SSCB_DEVICE_NAME": "ssc-imu",
    "SSCB_IMU_OUTPUT_SCALE": "1.0",
    "SSCB_ACCELEROMETER_MOUNT_MATRIX": "1,0,0,0,1,0,0,0,1",
    "SSCB_GYROSCOPE_MOUNT_MATRIX": "1,0,0,0,1,0,0,0,1",
}

WRITE_ENABLED = "--write-enabled" in sys.argv


def _arg_value(name):
    for idx, arg in enumerate(sys.argv):
        if arg == name and idx + 1 < len(sys.argv):
            return sys.argv[idx + 1]
        if arg.startswith(f"{name}="):
            return arg.split("=", 1)[1]
    return None


WRITE_READY_FILE = _arg_value("--ready-file")


# ── Localization ──────────────────────────────────────────────────────

_TR = {
    "app_title":            "ssc-bridge 传感器桥接",
    "tab_status":           "运行状态",
    "tab_imu":              "IMU 参数",
    "tab_monitor":          "实时姿态",
    "tab_about":            "关于",
    "status_running":       "运行中",
    "status_stopped":       "已停止",
    "status_enabled":       "开机自启：是",
    "status_disabled":      "开机自启：否",
    "status_unknown":       "未知",
    "btn_start":            "启动",
    "btn_stop":             "停止",
    "btn_restart":          "重启",
    "btn_enable_auto":      "启用开机自启",
    "btn_disable_auto":     "禁用开机自启",
    "btn_enable_edit":      "解锁配置修改",
    "btn_apply":            "应用",
    "btn_reset_defaults":   "恢复默认配置",
    "btn_close":            "关闭",
    "btn_open_link":        "打开链接",
    "device_name":          "设备名 (SSCB_DEVICE_NAME)",
    "output_scale":         "输出缩放 (SSCB_IMU_OUTPUT_SCALE)",
    "accel_mount":          "加速度计 Mount Matrix",
    "gyro_mount":           "陀螺仪 Mount Matrix",
    "orientation":          "设备方向",
    "pitch_roll":           "俯仰 / 横滚",
    "accel_raw":            "加速度计 (m/s²)",
    "gyro_raw":             "陀螺仪 (rad/s)",
    "light_lux":            "环境光照",
    "hint_no_data":         "等待传感器数据...",
    "hint_reset_done":      "已恢复默认配置",
    "hint_restart_warn":    "参数已更新，需重启 ssc-bridge 生效",
    "hint_not_running":     "ssc-bridge 未运行，传感器数据不可用",
    "hint_data_fresh":      "传感器数据已更新",
    "err_perm":             "权限不足，请以 root 运行",
    "err_config_read":      "无法读取配置文件：{err}",
    "err_config_write":     "无法写入配置文件：{err}",
    "err_service":          "systemd 操作失败：{err}",
    "err_open_link":        "打开链接失败：{err}",
    "err_helper_missing":   "未找到提权写入 helper：{path}",
    "orientation_normal":   "正常 (屏幕朝上)",
    "orientation_left":     "左侧朝下",
    "orientation_right":    "右侧朝下",
    "orientation_bottom":   "屏幕朝下",
    "orientation_tilted":   "倾斜",
}

ORIENTATION_LABELS = {
    "normal":     "orientation_normal",
    "bottom-up":  "orientation_bottom",
    "left-up":    "orientation_left",
    "right-up":   "orientation_right",
    "tilted":     "orientation_tilted",
}

_DESC = {
    "device_name":  "uinput 设备节点名。InputPlumber 用户可设为 "
                    "Sunshine gamepad (virtual) motion sensors 以绕过白名单。",
    "output_scale": "IMU 数据输出倍率。默认 1.0。"
                    "InputPlumber 会将所有 IMU 数据缩放 0.01，可设为 100.0 抵消。",
    "accel_mount":  "加速度计坐标系 → 屏幕坐标系的 3×3 变换矩阵。"
                    "9 个逗号分隔浮点数，row-major。",
    "gyro_mount":   "陀螺仪坐标系 → 屏幕坐标系的 3×3 变换矩阵。",
}

ABOUT_LINKS = [
    (
        "传感器修复",
        "right-0903/linux-gaokun",
        "https://github.com/right-0903/linux-gaokun",
        "首次在华为 MateBook E Go 上打通 SLPI DSP → hexagonrpcd → libssc → ssccli 全链路。",
    ),
    (
        "桥接参考",
        "gio3k/ssc-uinput",
        "https://github.com/gio3k/ssc-uinput",
        "Rust 版 SLPI → uinput 桥接，合并 IMU 设备、输出缩放、超时机制均受其启发。",
    ),
    (
        "libssc",
        "DylanVanAssche/libssc",
        "https://codeberg.org/DylanVanAssche/libssc",
        "Qualcomm SSC QMI 传感器协议的 GLib/GObject 封装库。",
    ),
    (
        "hexagonrpcd",
        "linux-msm/hexagonrpc",
        "https://github.com/linux-msm/hexagonrpc",
        "Qualcomm Hexagon DSP FastRPC 守护进程，负责文件映射与 DSP 通信。",
    ),
]


def t(key, **kwargs):
    s = _TR.get(key, key)
    return s.format(**kwargs) if kwargs else s


def desc(name):
    return _DESC.get(name, "")


# ── System helpers ────────────────────────────────────────────────────

def _run(*args):
    try:
        r = subprocess.run(args, capture_output=True, text=True, timeout=5)
        return r.returncode, r.stdout.strip()
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return -1, ""


def systemctl_is_active():
    code, _ = _run("systemctl", "is-active", "--quiet", SERVICE_NAME)
    return code == 0


def systemctl_is_enabled():
    code, _ = _run("systemctl", "is-enabled", "--quiet", SERVICE_NAME)
    return code == 0


def systemctl_start():
    code, out = _run("systemctl", "start", SERVICE_NAME)
    if code != 0:
        raise OSError(out or f"exit code {code}")


def systemctl_stop():
    code, out = _run("systemctl", "stop", SERVICE_NAME)
    if code != 0:
        raise OSError(out or f"exit code {code}")


def systemctl_enable():
    code, out = _run("systemctl", "enable", "--quiet", SERVICE_NAME)
    if code != 0:
        raise OSError(out or f"exit code {code}")


def systemctl_disable():
    code, out = _run("systemctl", "disable", "--quiet", SERVICE_NAME)
    if code != 0:
        raise OSError(out or f"exit code {code}")


# ── Config read/write ─────────────────────────────────────────────────

def read_config():
    cfg = dict(DEFAULT_CONFIG)
    if not CONFIG_FILE.is_file():
        return cfg
    try:
        text = CONFIG_FILE.read_text()
    except Exception:
        return cfg
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        m = re.match(r'^(\w+)\s*=\s*["\']?(.+?)["\']?\s*$', line)
        if m:
            cfg[m.group(1)] = m.group(2)
    return cfg


def write_config(cfg):
    lines = []
    for key in DEFAULT_CONFIG:
        lines.append(f'{key}="{cfg.get(key, DEFAULT_CONFIG[key])}"')
    try:
        CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
        CONFIG_FILE.write_text("\n".join(lines) + "\n")
    except PermissionError:
        raise
    except Exception as e:
        raise OSError(str(e))


# ── Sensor data readers ───────────────────────────────────────────────

def _read_float_file(path, count=3):
    try:
        text = path.read_text().strip()
    except (OSError, FileNotFoundError):
        return None
    parts = text.split(",")
    try:
        return [float(p) for p in parts[:count]]
    except ValueError:
        return None


def read_accel():
    v = _read_float_file(ACCEL_FILE, 3)
    return tuple(v) if v and len(v) == 3 else None


def read_gyro():
    v = _read_float_file(GYRO_FILE, 3)
    return tuple(v) if v and len(v) == 3 else None


def read_light():
    try:
        return float(LIGHT_FILE.read_text().strip())
    except (OSError, ValueError, FileNotFoundError):
        return None


def compute_orientation(x, y, z):
    """Determine device orientation from accelerometer vector.

    Returns (label_key, pitch_deg, roll_deg).
    """
    pitch = math.degrees(math.atan2(x, math.sqrt(y * y + z * z)))
    roll  = math.degrees(math.atan2(y, math.sqrt(x * x + z * z)))

    ax, ay, az = abs(x), abs(y), abs(z)
    threshold = 4.0  # m/s² — gravity is ~9.81

    if az > ay and az > ax and z > threshold:
        return ("normal", pitch, roll)
    elif az > ay and az > ax and z < -threshold:
        return ("bottom-up", pitch, roll)
    elif ax > ay and ax > az and x > threshold:
        return ("left-up", pitch, roll)
    elif ax > ay and ax > az and x < -threshold:
        return ("right-up", pitch, roll)
    else:
        return ("tilted", pitch, roll)


# ── GUI Window ────────────────────────────────────────────────────────

class BridgeTunerWindow(Gtk.Window):
    def __init__(self):
        super().__init__()
        self.set_title(t("app_title"))
        self.set_default_size(720, 760)
        self.set_size_request(500, 400)

        self._config = DEFAULT_CONFIG.copy()
        self._poll_source = None
        self._row_widgets = {}

        self._overlay = Adw.ToastOverlay()
        self._root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self._overlay.set_child(self._root)
        self.set_child(self._overlay)

        self._build_ui()
        self._start_polling()

    def show_toast(self, text, timeout=3):
        toast = Adw.Toast.new(text)
        toast.set_timeout(timeout)
        self._overlay.add_toast(toast)

    def _make_card(self):
        frame = Gtk.Frame()
        frame.add_css_class("card")
        return frame

    def _make_card_box(self, spacing=6, margins=12):
        return Gtk.Box(
            orientation=Gtk.Orientation.VERTICAL,
            spacing=spacing,
            margin_start=margins,
            margin_end=margins,
            margin_top=10,
            margin_bottom=10,
        )

    # ── Build ──────────────────────────────────────────────────────

    def _build_ui(self):
        outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL,
                        margin_top=8, margin_bottom=8)
        outer.set_vexpand(True)

        clamp = Adw.Clamp(maximum_size=760, tightening_threshold=420)
        content = Gtk.Box(orientation=Gtk.Orientation.VERTICAL,
                          spacing=10, margin_start=8, margin_end=8)
        clamp.set_child(content)
        outer.append(clamp)

        switcher = Gtk.StackSwitcher(halign=Gtk.Align.CENTER)
        stack = Gtk.Stack(vexpand=True, hexpand=True,
                          transition_type=Gtk.StackTransitionType.SLIDE_LEFT_RIGHT)
        switcher.set_stack(stack)
        content.append(switcher)
        content.append(stack)

        stack.add_titled(self._build_status_page(),  "status",  t("tab_status"))
        stack.add_titled(self._build_imu_page(),     "imu",     t("tab_imu"))
        stack.add_titled(self._build_monitor_page(), "monitor", t("tab_monitor"))
        stack.add_titled(self._build_about_page(),   "about",   t("tab_about"))

        content.append(self._build_action_row())
        self._root.append(outer)

    # ── Status page ────────────────────────────────────────────────

    def _build_status_page(self):
        scroll = Gtk.ScrolledWindow(hscrollbar_policy=Gtk.PolicyType.NEVER,
                                    vscrollbar_policy=Gtk.PolicyType.AUTOMATIC,
                                    vexpand=True)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL,
                      spacing=8, margin_start=4, margin_end=4)

        # Running status card
        card1 = self._make_card()
        cb1 = self._make_card_box(spacing=8)
        card1.set_child(cb1)

        title1 = Gtk.Label(label=t("tab_status"), xalign=0)
        title1.add_css_class("heading")
        cb1.append(title1)

        self._status_running_label = Gtk.Label(label=t("status_unknown"), xalign=0)
        self._status_running_label.add_css_class("title-1")
        cb1.append(self._status_running_label)

        self._status_enabled_label = Gtk.Label(label=t("status_unknown"), xalign=0)
        self._status_enabled_label.add_css_class("caption")
        cb1.append(self._status_enabled_label)

        self._status_data_age = Gtk.Label(label=t("hint_no_data"), xalign=0)
        self._status_data_age.add_css_class("caption")
        cb1.append(self._status_data_age)

        box.append(card1)

        # Control buttons card
        card2 = self._make_card()
        cb2 = self._make_card_box(spacing=8)
        card2.set_child(cb2)

        btn_box1 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self._btn_start = Gtk.Button(label=t("btn_start"))
        self._btn_start.connect("clicked", lambda *_: self._on_start())
        self._btn_stop = Gtk.Button(label=t("btn_stop"))
        self._btn_stop.connect("clicked", lambda *_: self._on_stop())
        self._btn_restart = Gtk.Button(label=t("btn_restart"))
        self._btn_restart.connect("clicked", lambda *_: self._on_restart())
        btn_box1.append(self._btn_start)
        btn_box1.append(self._btn_stop)
        btn_box1.append(self._btn_restart)
        cb2.append(btn_box1)

        self._btn_auto = Gtk.Button(label=t("btn_enable_auto"))
        self._btn_auto.connect("clicked", lambda *_: self._on_toggle_auto())
        btn_box2 = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        btn_box2.append(self._btn_auto)
        cb2.append(btn_box2)

        box.append(card2)
        scroll.set_child(box)
        return scroll

    # ── IMU page ───────────────────────────────────────────────────

    def _build_imu_page(self):
        scroll = Gtk.ScrolledWindow(hscrollbar_policy=Gtk.PolicyType.NEVER,
                                    vscrollbar_policy=Gtk.PolicyType.AUTOMATIC,
                                    vexpand=True)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL,
                      spacing=8, margin_start=4, margin_end=4)

        box.append(self._build_entry_card("device_name"))
        box.append(self._build_slider_card("output_scale", 0.1, 200.0, 0.5))
        box.append(self._build_matrix_card("accel_mount"))
        box.append(self._build_matrix_card("gyro_mount"))

        scroll.set_child(box)
        return scroll

    def _build_entry_card(self, key):
        card = self._make_card()
        cb = self._make_card_box()
        card.set_child(cb)

        title = Gtk.Label(label=t(key), xalign=0)
        title.add_css_class("heading")
        cb.append(title)

        d = desc(key)
        if d:
            sub = Gtk.Label(label=d, xalign=0, wrap=True)
            sub.set_wrap_mode(Pango.WrapMode.WORD_CHAR)
            sub.add_css_class("caption")
            cb.append(sub)

        entry_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        env_key = self._key_to_env(key)
        entry = Gtk.Entry(hexpand=True)
        entry.set_text(self._config.get(env_key, DEFAULT_CONFIG[env_key]))
        apply_btn = Gtk.Button(label=t("btn_apply"))
        apply_btn.connect("clicked", lambda *_, e=entry, k=key:
                          self._commit_entry(k, e))

        def on_activate(ent, k):
            self._commit_entry(k, ent)
        entry.connect("activate", on_activate)

        entry_box.append(entry)
        entry_box.append(apply_btn)
        cb.append(entry_box)

        entry.set_sensitive(WRITE_ENABLED)
        apply_btn.set_sensitive(WRITE_ENABLED)
        self._row_widgets[key] = (entry, apply_btn)
        return card

    def _build_slider_card(self, key, lo, hi, step):
        card = self._make_card()
        cb = self._make_card_box()
        card.set_child(cb)

        title = Gtk.Label(label=t(key), xalign=0)
        title.add_css_class("heading")
        cb.append(title)

        d = desc(key)
        if d:
            sub = Gtk.Label(label=d, xalign=0, wrap=True)
            sub.set_wrap_mode(Pango.WrapMode.WORD_CHAR)
            sub.add_css_class("caption")
            cb.append(sub)

        env_key = self._key_to_env(key)
        try:
            value = float(self._config.get(env_key, DEFAULT_CONFIG[env_key]))
        except ValueError:
            value = 1.0

        adj = Gtk.Adjustment(value=value, lower=lo, upper=hi,
                             step_increment=step,
                             page_increment=max(step * 10, step))
        scale_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=12)
        scale = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL, adjustment=adj)
        scale.set_draw_value(False)
        scale.set_hexpand(True)
        val_label = Gtk.Label(label=f"{value:.1f}", xalign=1)
        val_label.set_size_request(72, -1)

        def on_scale_changed(s, lbl):
            lbl.set_text(f"{s.get_value():.1f}")

        def on_scale_done(s, k):
            """Write config when user finishes dragging the scale."""
            self._set_config(k, f"{s.get_value():.1f}")

        scale.connect("value-changed", on_scale_changed, val_label)
        scale.connect("change-value", lambda s, scroll, new_val, k=key:
                      (self._set_config(k, f"{new_val:.1f}"), True)[1]
                      if not hasattr(self, '_dragging') else None)

        scale_box.append(scale)
        scale_box.append(val_label)
        cb.append(scale_box)

        scale.set_sensitive(WRITE_ENABLED)
        self._row_widgets[key] = scale
        return card

    def _build_matrix_card(self, key):
        card = self._make_card()
        cb = self._make_card_box()
        card.set_child(cb)

        title = Gtk.Label(label=t(key), xalign=0)
        title.add_css_class("heading")
        cb.append(title)

        d = desc(key)
        if d:
            sub = Gtk.Label(label=d, xalign=0, wrap=True)
            sub.set_wrap_mode(Pango.WrapMode.WORD_CHAR)
            sub.add_css_class("caption")
            cb.append(sub)

        env_key = self._key_to_env(key)
        raw = self._config.get(env_key, DEFAULT_CONFIG[env_key])
        values = [v.strip() for v in raw.split(",")]
        while len(values) < 9:
            values.append("0.0")

        grid = Gtk.Grid(column_homogeneous=True, row_homogeneous=True,
                        column_spacing=4, row_spacing=4, margin_top=4)
        entries = []
        for row in range(3):
            row_entries = []
            for col in range(3):
                idx = row * 3 + col
                e = Gtk.Entry(text=values[idx], hexpand=True,
                              input_purpose=Gtk.InputPurpose.NUMBER)
                e.set_sensitive(WRITE_ENABLED)
                grid.attach(e, col, row, 1, 1)
                row_entries.append(e)
            entries.append(row_entries)

        apply_btn = Gtk.Button(label=t("btn_apply"), margin_top=6)
        apply_btn.connect("clicked", lambda *_: self._commit_matrix(key, entries))
        apply_btn.set_sensitive(WRITE_ENABLED)

        cb.append(grid)
        cb.append(apply_btn)
        self._row_widgets[key] = (entries, apply_btn)
        return card

    # ── Monitor page ───────────────────────────────────────────────

    def _build_monitor_page(self):
        scroll = Gtk.ScrolledWindow(hscrollbar_policy=Gtk.PolicyType.NEVER,
                                    vscrollbar_policy=Gtk.PolicyType.AUTOMATIC,
                                    vexpand=True)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL,
                      spacing=8, margin_start=4, margin_end=4)

        # Orientation card
        card1 = self._make_card()
        cb1 = self._make_card_box(spacing=6)
        card1.set_child(cb1)

        orient_title = Gtk.Label(label=t("orientation"), xalign=0)
        orient_title.add_css_class("heading")
        cb1.append(orient_title)

        self._orient_label = Gtk.Label(label=t("hint_no_data"), xalign=0)
        self._orient_label.add_css_class("title-2")
        cb1.append(self._orient_label)

        self._pr_label = Gtk.Label(label="", xalign=0)
        self._pr_label.add_css_class("caption")
        cb1.append(self._pr_label)

        box.append(card1)

        # Sensor data cards
        for sensor_key, title_key in [
            ("accel", "accel_raw"),
            ("gyro",  "gyro_raw"),
            ("light", "light_lux"),
        ]:
            box.append(self._build_sensor_data_card(t(title_key), sensor_key))

        scroll.set_child(box)
        return scroll

    def _build_sensor_data_card(self, title_text, key):
        card = self._make_card()
        cb = self._make_card_box(spacing=4)
        card.set_child(cb)

        title = Gtk.Label(label=title_text, xalign=0)
        title.add_css_class("heading")
        cb.append(title)

        label = Gtk.Label(label=t("hint_no_data"), xalign=0)
        label.add_css_class("monospace")
        label.add_css_class("caption")
        cb.append(label)

        setattr(self, f"_monitor_{key}_label", label)
        return card

    # ── About page ─────────────────────────────────────────────────

    def _build_about_page(self):
        scroll = Gtk.ScrolledWindow(hscrollbar_policy=Gtk.PolicyType.NEVER,
                                    vscrollbar_policy=Gtk.PolicyType.AUTOMATIC,
                                    vexpand=True)
        shell = Gtk.Box(orientation=Gtk.Orientation.VERTICAL,
                        margin_start=4, margin_end=4)
        frame = self._make_card()
        shell.append(frame)
        scroll.set_child(shell)

        box = self._make_card_box(spacing=10)
        frame.set_child(box)

        for idx, (role, name, url, summary) in enumerate(ABOUT_LINKS):
            if idx:
                box.append(Gtk.Separator(orientation=Gtk.Orientation.HORIZONTAL))

            row = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
            head = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
            role_lbl = Gtk.Label(label=role, xalign=0)
            role_lbl.add_css_class("heading")
            role_lbl.set_hexpand(True)
            link = Gtk.Button(label=name, halign=Gtk.Align.START)
            link.connect("clicked", lambda *_a, u=url: self._open_link(u))
            head.append(role_lbl)
            head.append(link)

            summary_lbl = Gtk.Label(label=summary, xalign=0, wrap=True)
            summary_lbl.set_wrap_mode(Pango.WrapMode.WORD_CHAR)
            summary_lbl.add_css_class("caption")

            row.append(head)
            row.append(summary_lbl)
            box.append(row)

        return scroll

    # ── Action row ─────────────────────────────────────────────────

    def _build_action_row(self):
        row = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        buttons = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL,
                          spacing=8, halign=Gtk.Align.END)
        if not WRITE_ENABLED:
            enable_btn = Gtk.Button(label=t("btn_enable_edit"))
            enable_btn.connect("clicked", lambda *_: self._request_write_mode())
            buttons.append(enable_btn)
        reset_btn = Gtk.Button(label=t("btn_reset_defaults"))
        reset_btn.connect("clicked", lambda *_: self._reset_defaults())
        reset_btn.set_sensitive(WRITE_ENABLED)
        buttons.append(reset_btn)
        row.append(buttons)
        return row

    # ── Key mapping ────────────────────────────────────────────────

    def _key_to_env(self, gui_key):
        return {
            "device_name":  "SSCB_DEVICE_NAME",
            "output_scale": "SSCB_IMU_OUTPUT_SCALE",
            "accel_mount":  "SSCB_ACCELEROMETER_MOUNT_MATRIX",
            "gyro_mount":   "SSCB_GYROSCOPE_MOUNT_MATRIX",
        }.get(gui_key, gui_key)

    # ── Config write helpers ───────────────────────────────────────

    def _write_config(self):
        if not WRITE_ENABLED:
            return False
        try:
            write_config(self._config)
            return True
        except PermissionError:
            self.show_toast(t("err_perm"))
        except Exception as e:
            self.show_toast(t("err_config_write", err=e))
        return False

    def _set_config(self, gui_key, value):
        env_key = self._key_to_env(gui_key)
        self._config[env_key] = str(value)
        if self._write_config():
            self.show_toast(t("hint_restart_warn"))

    def _commit_entry(self, gui_key, entry):
        self._set_config(gui_key, entry.get_text().strip())

    def _commit_matrix(self, gui_key, entries):
        vals = []
        for row in entries:
            for e in row:
                vals.append(e.get_text().strip())
        self._set_config(gui_key, ",".join(vals))

    # ── Button handlers ────────────────────────────────────────────

    def _on_start(self):
        try:
            systemctl_start()
            self.show_toast(t("status_running"))
        except Exception as e:
            self.show_toast(t("err_service", err=e))
        self._refresh_status()

    def _on_stop(self):
        try:
            systemctl_stop()
            self.show_toast(t("status_stopped"))
        except Exception as e:
            self.show_toast(t("err_service", err=e))
        self._refresh_status()

    def _on_restart(self):
        try:
            systemctl_stop()
            systemctl_start()
            self.show_toast(t("status_running"))
        except Exception as e:
            self.show_toast(t("err_service", err=e))
        self._refresh_status()

    def _on_toggle_auto(self):
        try:
            if systemctl_is_enabled():
                systemctl_disable()
            else:
                systemctl_enable()
        except Exception as e:
            self.show_toast(t("err_service", err=e))
        self._refresh_status()

    def _reset_defaults(self):
        if not WRITE_ENABLED:
            return
        self._config = DEFAULT_CONFIG.copy()
        try:
            write_config(self._config)
            self._refresh_imu_fields()
            self.show_toast(t("hint_reset_done"))
        except Exception as e:
            self.show_toast(t("err_config_write", err=e))

    def _open_link(self, url):
        try:
            subprocess.Popen(
                ["xdg-open", url],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except Exception as e:
            self.show_toast(t("err_open_link", err=e))

    # ── Privilege escalation ───────────────────────────────────────

    def _request_write_mode(self):
        try:
            passthrough = [
                "DISPLAY", "XAUTHORITY", "WAYLAND_DISPLAY",
                "XDG_RUNTIME_DIR", "DBUS_SESSION_BUS_ADDRESS",
            ]
            env_args = [f"{k}={v}" for k in passthrough
                        if (v := os.environ.get(k))]
            ready_file = Path(tempfile.gettempdir()) / \
                f"ssc-bridge-tuner-ready-{uuid.uuid4().hex}"
            cmd = [
                "pkexec", "env", *env_args,
                sys.executable, os.path.abspath(__file__),
                "--write-enabled", "--ready-file", str(ready_file),
            ]
            subprocess.Popen(cmd, start_new_session=True)
            self._wait_for_write_ready(ready_file)
        except Exception as e:
            self.show_toast(t("err_service", err=e))

    def _wait_for_write_ready(self, ready_file):
        started_at = GLib.get_monotonic_time()

        def poll():
            if ready_file.exists():
                self.close()
                return False
            if GLib.get_monotonic_time() - started_at > 20 * 1_000_000:
                self.show_toast(t("err_helper_missing", path=str(ready_file)))
                return False
            return True

        GLib.timeout_add(200, poll)

    def _announce_write_ready(self):
        if not WRITE_ENABLED or not WRITE_READY_FILE:
            return False
        try:
            Path(WRITE_READY_FILE).write_text("ready")
        except Exception:
            return False
        return False

    # ── Refresh ────────────────────────────────────────────────────

    def _refresh_status(self):
        running = systemctl_is_active()
        enabled = systemctl_is_enabled()

        self._status_running_label.set_label(
            t("status_running") if running else t("status_stopped"))
        self._status_enabled_label.set_label(
            t("status_enabled") if enabled else t("status_disabled"))

        self._btn_start.set_sensitive(not running)
        self._btn_stop.set_sensitive(running)
        self._btn_restart.set_sensitive(running)

        self._btn_auto.set_label(
            t("btn_disable_auto") if enabled else t("btn_enable_auto"))

    def _refresh_imu_fields(self):
        self._config = read_config()

        widget = self._row_widgets.get("device_name")
        if widget:
            entry, _ = widget
            entry.set_text(self._config.get(
                "SSCB_DEVICE_NAME", DEFAULT_CONFIG["SSCB_DEVICE_NAME"]))

        widget = self._row_widgets.get("output_scale")
        if widget:
            try:
                val = float(self._config.get(
                    "SSCB_IMU_OUTPUT_SCALE",
                    DEFAULT_CONFIG["SSCB_IMU_OUTPUT_SCALE"]))
            except ValueError:
                val = 1.0
            widget.set_value(val)

        for key in ("accel_mount", "gyro_mount"):
            widget = self._row_widgets.get(key)
            if not widget:
                continue
            entries, _ = widget
            env_key = self._key_to_env(key)
            raw = self._config.get(env_key, DEFAULT_CONFIG[env_key])
            vals = [v.strip() for v in raw.split(",")]
            while len(vals) < 9:
                vals.append("0.0")
            for row in range(3):
                for col in range(3):
                    entries[row][col].set_text(vals[row * 3 + col])

    def _refresh_monitor(self):
        accel = read_accel()
        gyro = read_gyro()
        light = read_light()
        running = systemctl_is_active()

        if not running:
            self._orient_label.set_label(t("hint_not_running"))
            self._pr_label.set_label("")
            for k in ("accel", "gyro", "light"):
                lbl = getattr(self, f"_monitor_{k}_label", None)
                if lbl:
                    lbl.set_label(t("hint_not_running"))
            self._status_data_age.set_label(t("hint_not_running"))
            return

        if accel:
            orient_key, pitch, roll = compute_orientation(*accel)
            self._orient_label.set_label(
                t(ORIENTATION_LABELS.get(orient_key, "orientation_tilted")))
            self._pr_label.set_label(
                f"{t('pitch_roll')}: pitch={pitch:.1f}°  roll={roll:.1f}°")
        else:
            self._orient_label.set_label(t("hint_no_data"))
            self._pr_label.set_label("")

        al = getattr(self, "_monitor_accel_label", None)
        if al:
            al.set_label(
                f"X={accel[0]:.3f}  Y={accel[1]:.3f}  Z={accel[2]:.3f}"
                if accel else t("hint_no_data"))

        gl = getattr(self, "_monitor_gyro_label", None)
        if gl:
            gl.set_label(
                f"X={gyro[0]:.4f}  Y={gyro[1]:.4f}  Z={gyro[2]:.4f}"
                if gyro else t("hint_no_data"))

        ll = getattr(self, "_monitor_light_label", None)
        if ll:
            ll.set_label(f"{light:.1f} lux" if light is not None
                        else t("hint_no_data"))

        self._status_data_age.set_label(
            t("hint_data_fresh") if (accel or gyro or light is not None)
            else t("hint_no_data"))

    def _poll_tick(self):
        self._refresh_status()
        self._refresh_monitor()
        return True

    def _start_polling(self):
        if self._poll_source is None:
            self._poll_source = GLib.timeout_add(POLL_INTERVAL_MS, self._poll_tick)

    def _stop_polling(self):
        if self._poll_source is not None:
            GLib.source_remove(self._poll_source)
            self._poll_source = None


# ── Entry point ───────────────────────────────────────────────────────

def main():
    Adw.init()
    loop = GLib.MainLoop()
    win = BridgeTunerWindow()
    win.connect("close-request", lambda *_: (loop.quit(), False)[1])
    win.present()
    if WRITE_ENABLED and WRITE_READY_FILE:
        GLib.idle_add(win._announce_write_ready)
    loop.run()
    raise SystemExit(0)


if __name__ == "__main__":
    main()
