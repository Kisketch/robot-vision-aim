#!/usr/bin/env python3
"""Generate the README's explanatory diagrams (docs/diagrams/*.svg).

Same palette and conventions as tools/make_charts.py: pure stdlib, SVG
text so the files diff and render on GitHub.  Every number printed on a
diagram is quoted from the README / BASELINE.md / code (named beside it).
    python3 tools/make_diagrams.py
"""
import math
import os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "diagrams")

BG, PANEL, EDGE = "#0f172a", "#16213a", "#334155"
BLUE, TEAL, AMBER, RED, INK, MUTED = "#3b82f6", "#14b8a6", "#f59e0b", "#ef4444", "#e2e8f0", "#94a3b8"
FONT = "font-family=\"'Segoe UI', Helvetica, Arial, sans-serif\""


def svg(w, h, title, subtitle):
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" {FONT}>',
        "<defs>"
        f'<marker id="a" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{MUTED}"/></marker>'
        f'<marker id="at" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{TEAL}"/></marker>'
        f'<marker id="ar" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{RED}"/></marker>'
        f'<marker id="aa" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{AMBER}"/></marker>'
        "</defs>",
        f'<rect width="{w}" height="{h}" fill="{BG}"/>',
        f'<text x="24" y="30" font-size="17" font-weight="bold" fill="{INK}">{title}</text>',
        f'<text x="24" y="50" font-size="11.5" fill="{MUTED}">{subtitle}</text>',
    ]


def save(p, name):
    p.append("</svg>")
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name), "w", encoding="utf-8") as f:
        f.write("\n".join(p) + "\n")
    print("wrote", name)


def text(p, x, y, s, size=12, fill=INK, anchor="start", weight="normal"):
    s = s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    p.append(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{fill}" '
             f'text-anchor="{anchor}" font-weight="{weight}">{s}</text>')


def box(p, x, y, w, h, stroke=EDGE, fill=PANEL, r=8, dash=""):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    p.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{r}" fill="{fill}" stroke="{stroke}" stroke-width="1.5"{d}/>')


def arrow(p, x1, y1, x2, y2, colour=MUTED, marker="a", dash="", width=1.6):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    p.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{colour}" '
             f'stroke-width="{width}" marker-end="url(#{marker})"{d}/>')


# ------------------------------------------------------------- pipeline
def pipeline():
    W, H = 920, 360
    p = svg(W, H, "Pipeline: one frame, from photons to a fire command",
            "Per-stage cost single-threaded (README, Timing). Dashed: sentry-only feedback. Exposure stamps travel with every observation.")
    stages = [
        ("Camera", ["frame +", "exposure stamp"], "", BLUE),
        ("Detector", ["light bars → pairs", "corners → MLP digit"], "3–6 ms", AMBER),
        ("Pose", ["IPPE PnP, 2 poses", "→ yaw search"], "≈ 72 µs", AMBER),
        ("Tracker", ["13-state", "whole-robot EKF"], "< 0.5 ms*", TEAL),
        ("Aimer", ["ballistics RK4 +", "fire window"], "≈ 40 µs / solve", TEAL),
        ("MCU", ["yaw/pitch + rates", "fire flag"], "", BLUE),
    ]
    bw, bh, gap, x0, y0 = 124, 92, 23, 28, 92
    for i, (name, lines, cost, colour) in enumerate(stages):
        x = x0 + i * (bw + gap)
        box(p, x, y0, bw, bh, stroke=colour)
        text(p, x + bw / 2, y0 + 26, name, 14, colour, "middle", "bold")
        for j, ln in enumerate(lines):
            text(p, x + bw / 2, y0 + 50 + j * 16, ln, 11.5, INK, "middle")
        if cost:
            text(p, x + bw / 2, y0 + bh + 18, cost, 11, MUTED, "middle")
        if i < len(stages) - 1:
            arrow(p, x + bw + 2, y0 + bh / 2, x + bw + gap - 2, y0 + bh / 2)
    # attitude path: MCU -> PoseBuffer -> Pose
    px, py, pw, ph = x0 + 2 * (bw + gap) - 30, 252, 3 * bw + 2 * gap + 60, 70
    box(p, px, py, pw, ph, stroke=BLUE, dash="0")
    text(p, px + pw / 2, py + 24, "PoseBuffer — gimbal attitude ring buffer", 13, BLUE, "middle", "bold")
    text(p, px + pw / 2, py + 44, "quaternion SLERP at each frame's exposure instant;", 11.5, INK, "middle")
    text(p, px + pw / 2, py + 60, "no bracketing sample → frame dropped, never aimed from a wrong pose", 11.5, INK, "middle")
    mcu_x = x0 + 5 * (bw + gap) + bw / 2
    p.append(f'<line x1="{mcu_x}" y1="{y0 + bh + 26}" x2="{mcu_x}" y2="{py + ph / 2 + 2}" stroke="{MUTED}" stroke-width="1.6"/>')
    p.append(f'<line x1="{mcu_x}" y1="{py + ph / 2 + 2}" x2="{px + pw + 2}" y2="{py + ph / 2 + 2}" stroke="{MUTED}" stroke-width="1.6" marker-end="url(#a)"/>')
    text(p, mcu_x - 8, py - 12, "17-byte attitude frames", 10.5, MUTED, "end")
    pose_x = x0 + 2 * (bw + gap) + bw / 2
    arrow(p, pose_x, py - 2, pose_x, y0 + bh + 26)
    # sentry ROI feedback: Tracker -> Detector (above)
    tx = x0 + 3 * (bw + gap) + bw / 2
    dx = x0 + 1 * (bw + gap) + bw / 2
    p.append(f'<path d="M{tx},{y0 - 2} C{tx},{y0 - 30} {dx},{y0 - 30} {dx},{y0 - 4}" fill="none" stroke="{TEAL}" '
             f'stroke-width="1.5" stroke-dasharray="5 4" marker-end="url(#at)"/>')
    text(p, (tx + dx) / 2, y0 - 26, "predicted ROI windows (sentry): detection 1.3–1.6 ms", 10.5, TEAL, "middle")
    text(p, 24, H - 10, "* tracker + aimer + shot judge combined. Detector: 6.3 ms in the Timing table, 3.4 ms on the Linux box used for Quick start.", 10, MUTED)
    save(p, "pipeline.svg")


# -------------------------------------------------------- time alignment
def time_alignment():
    W, H = 920, 380
    p = svg(W, H, "Time first: look up the gimbal attitude at the exposure instant",
            "Receipt-time stamping is late by the whole transport + processing delay D, always in the same direction.")
    x0, x1 = 60, 880
    ty = 300                                     # time axis
    arrow(p, x0, ty, x1, ty)
    text(p, x1, ty + 20, "time", 11, MUTED, "end")
    # attitude samples (MCU), every ~1 ms
    ay = 130
    text(p, x0, ay - 26, "gimbal attitude samples (MCU, serial)", 11.5, BLUE)
    xs = list(range(x0 + 20, x1 - 20, 34))
    for x in xs:
        p.append(f'<line x1="{x}" y1="{ay - 8}" x2="{x}" y2="{ay + 8}" stroke="{BLUE}" stroke-width="2"/>')
    p.append(f'<line x1="{x0}" y1="{ay}" x2="{x1 - 20}" y2="{ay}" stroke="{EDGE}" stroke-width="1"/>')
    # exposure window
    te, D = 300, 330
    box(p, te - 28, ty - 44, 56, 30, stroke=AMBER, fill="#3a2a0a", r=4)
    text(p, te, ty - 24, "exposure", 10.5, AMBER, "middle")
    p.append(f'<line x1="{te}" y1="{ty - 10}" x2="{te}" y2="{ty + 6}" stroke="{AMBER}" stroke-width="2"/>')
    text(p, te, ty + 24, "t_e  (exposure centre)", 11.5, AMBER, "middle", "bold")
    # arrival
    ta = te + D
    p.append(f'<line x1="{ta}" y1="{ty - 10}" x2="{ta}" y2="{ty + 6}" stroke="{MUTED}" stroke-width="2"/>')
    text(p, ta, ty + 24, "t_e + D  (frame reaches host)", 11.5, MUTED, "middle")
    p.append(f'<line x1="{te}" y1="{ty + 40}" x2="{ta}" y2="{ty + 40}" stroke="{MUTED}" stroke-width="1.2" '
             f'marker-start="url(#a)" marker-end="url(#a)"/>')
    text(p, (te + ta) / 2, ty + 56, "D = transport + processing delay (measured on the robot by DelayCalibrator)", 11, MUTED, "middle")
    # correct lookup
    arrow(p, te, ty - 48, te, ay + 12, TEAL, "at", width=2.2)
    text(p, te + 10, (ty + ay) / 2 - 8, "this system: SLERP between", 11.5, TEAL)
    text(p, te + 10, (ty + ay) / 2 + 8, "the two bracketing samples", 11.5, TEAL)
    # wrong lookup
    arrow(p, ta, ty - 12, ta, ay + 12, RED, "ar", dash="6 4", width=2.0)
    text(p, ta + 10, (ty + ay) / 2 - 8, "receipt-time lookup (old code):", 11.5, RED)
    text(p, ta + 10, (ty + ay) / 2 + 8, "attitude already moved by ω_g · D", 11.5, RED)
    # error callout
    box(p, 560, 62, 330, 40, stroke=RED, fill="#2a1418", r=6)
    text(p, 725, 80, "6 rad/s × 10 ms = 3.4° of aim error", 12, INK, "middle", "bold")
    text(p, 725, 95, "≈ 30 cm at 5 m — larger than a plate", 11, MUTED, "middle")
    save(p, "time_alignment.svg")


# ---------------------------------------------------------- robot model
def robot_model():
    W, H = 920, 440
    p = svg(W, H, "Track the robot, not the plate: the whole-robot EKF",
            "Top view. Four plates ride on one spinning body, so one state predicts every plate — including the one about to rotate into view.")
    cx, cy = 230, 235
    r1, r2 = 118, 104                           # drawn in proportion to r1 0.239 m / r2 0.214 m
    yaw = math.radians(-25)
    p.append(f'<circle cx="{cx}" cy="{cy}" r="{r2 - 6}" fill="#1b2944" stroke="{EDGE}" stroke-width="1.5"/>')
    p.append(f'<circle cx="{cx}" cy="{cy}" r="4" fill="{INK}"/>')
    text(p, cx - 10, cy + 22, "centre (xc, yc, zc)", 11, INK, "end")
    plates = []
    for k in range(4):
        a = yaw + k * math.pi / 2
        r = r1 if k % 2 == 0 else r2
        px, py = cx + r * math.cos(a), cy + r * math.sin(a)
        tx, ty = -math.sin(a), math.cos(a)
        L = 34
        colour = AMBER if k in (0, 1) else "#7c5a1a"
        p.append(f'<line x1="{px - tx * L / 2:.1f}" y1="{py - ty * L / 2:.1f}" x2="{px + tx * L / 2:.1f}" '
                 f'y2="{py + ty * L / 2:.1f}" stroke="{colour}" stroke-width="7" stroke-linecap="round"/>')
        p.append(f'<line x1="{cx}" y1="{cy}" x2="{px:.1f}" y2="{py:.1f}" stroke="{EDGE}" stroke-width="1" stroke-dasharray="3 3"/>')
        plates.append((px, py, a, r))
    # radius labels
    for k, lab in ((0, "r1"), (1, "r2")):
        px, py, a, r = plates[k]
        mx, my = cx + 0.55 * r * math.cos(a), cy + 0.55 * r * math.sin(a)
        text(p, mx + 6, my - 4, lab, 12, INK, "start", "bold")
    text(p, plates[2][0] + 14, plates[2][1] + 30, "plate k = 2", 10.5, MUTED)
    text(p, plates[3][0] + 26, plates[3][1] - 6, "plate k = 3", 10.5, MUTED)
    # yaw arc + omega
    p.append(f'<path d="M{cx + 60},{cy} A60,60 0 0,0 {cx + 60 * math.cos(yaw):.1f},{cy + 60 * math.sin(yaw):.1f}" '
             f'fill="none" stroke="{TEAL}" stroke-width="1.5"/>')
    p.append(f'<line x1="{cx}" y1="{cy}" x2="{cx + 150}" y2="{cy}" stroke="{EDGE}" stroke-width="1"/>')
    text(p, cx + 66, cy - 10, "yaw θ", 11.5, TEAL)
    R = 168
    a0, a1 = math.radians(150), math.radians(205)
    p.append(f'<path d="M{cx + R * math.cos(a0):.1f},{cy + R * math.sin(a0):.1f} A{R},{R} 0 0,1 '
             f'{cx + R * math.cos(a1):.1f},{cy + R * math.sin(a1):.1f}" fill="none" stroke="{TEAL}" '
             f'stroke-width="2" marker-end="url(#at)"/>')
    text(p, cx - R - 24, cy + 6, "ω", 16, TEAL, "end", "bold")
    # camera
    camx, camy = cx, 425
    p.append(f'<polygon points="{camx - 14},{camy} {camx + 14},{camy} {camx + 8},{camy - 14} {camx - 8},{camy - 14}" fill="{BLUE}"/>')
    text(p, camx + 20, camy - 4, "camera", 11, BLUE)
    for k in (0, 1):
        px, py, _, _ = plates[k]
        p.append(f'<line x1="{camx}" y1="{camy - 14}" x2="{px:.1f}" y2="{py:.1f}" stroke="{BLUE}" stroke-width="1" stroke-dasharray="4 4"/>')
    # state panel
    sx, sy = 470, 78
    box(p, sx, sy, 426, 236)
    text(p, sx + 16, sy + 26, "13 states (core/include/autoaim/tracker.hpp)", 13, INK, "start", "bold")
    rows = [
        ("centre position", "xc  yc  zc", BLUE),
        ("centre velocity", "vxc  vyc  vzc   (Singer-damped, τ = 0.3 s)", BLUE),
        ("spin", "yaw θ,  ω", TEAL),
        ("body geometry", "r1, r2 (two plate radii),  dz (2nd-pair height)", AMBER),
        ("CA extension", "axc  ay   (off by default — rejected on video)", MUTED),
    ]
    for i, (k, v, c) in enumerate(rows):
        y = sy + 58 + i * 34
        text(p, sx + 16, y, k, 11.5, MUTED)
        text(p, sx + 140, y, v, 12, c)
    text(p, sx + 16, sy + 222, "Plate k sits at yaw θ + k·π/2 on radius r1 (k even) or r2 (k odd).", 11, INK)
    # estimated values
    box(p, sx, 330, 426, 82, stroke=TEAL)
    text(p, sx + 16, 354, "Estimated on the reference videos", 12, TEAL, "start", "bold")
    text(p, sx + 16, 376, "3.avi   r1 0.243 m   r2 0.210 m   dz 0.050 m   ω ≈ 2.6 rad/s", 11.5, INK)
    text(p, sx + 16, 396, "4.avi   r1 0.239 m   r2 0.214 m   dz 0.046 m   ω ≈ 8.3 rad/s", 11.5, INK)
    save(p, "robot_model.svg")


# --------------------------------------------- aim timing principle (zh)
def aim_timing_principle():
    """Latency chain + whole-robot prediction + flight-time iteration.

    Numbers: sentry/config/sentry_165hz.yaml (team-reported latency values),
    core/src/aimer.cpp (6 iterations, 1e-4 s tolerance, fire gates),
    sentry/src/serial_protocol.cpp (21-byte frame).  Flight times: 25 m/s,
    k = 0.019 (tools/latency_study.json).
    """
    W, H = 1180, 900
    p = svg(W, H, "自瞄原理：延迟计算 → 整车预测 → 飞行时间迭代 → 开火与下发",
            "165 Hz 哨兵链路（sentry_165hz.yaml）。时间以曝光中心 t_exp 为原点；上车时前两段每帧实测，后两段用配置值。")
    text(p, W - 24, 70, "数据来源：延迟数值 = 队伍提供；165 Hz 结论 = 仿真推演（真实视频仅 25 fps，165 Hz 实拍待补）",
         11, AMBER, "end")

    # ---------------- 1. time axis -------------------------------------
    text(p, 24, 84, "① 时间轴：一帧画面从曝光到命中", 14, INK, weight="bold")
    x0, y = 60, 150
    # (label, model ms, range text, colour, how it is obtained)
    segs = [
        ("传输", 10, "6–14 ms", BLUE, "曝光 2.5 ms + 读出 + USB 4.4 ms"),
        ("处理", 5, "每帧实测", TEAL, "检测/PnP/EKF/瞄准"),
        ("串口+MCU", 20, "10–30 ms", AMBER, "21 B 帧 + MCU 控制周期"),
        ("发射", 30, "30 ms", RED, "开火 → 出膛（拨弹/摩擦轮）"),
        ("弹丸飞行", 210, "3 m 124 / 5 m 210 / 7 m 300 ms", "#a78bfa", "RK4 + 二次阻力，逐帧解算"),
    ]
    # piecewise scale: the four short segments get room, flight is compressed
    px = [150, 120, 170, 190, 330]
    x = x0
    ticks = [(x0, "t_exp")]
    for (name, ms, rng, colour, how), w in zip(segs, px):
        p.append(f'<rect x="{x}" y="{y}" width="{w - 3}" height="34" rx="4" fill="{colour}" opacity="0.88"/>')
        text(p, x + w / 2, y + 22, f"{name} {ms} ms", 12, BG, "middle", "bold")
        text(p, x + w / 2, y + 52, rng, 11, INK, "middle")
        text(p, x + w / 2, y + 68, how, 10.5, MUTED, "middle")
        x += w
        ticks.append((x, None))
    names = ["t_exp\n曝光中心戳", "t_host\n帧到内存", "t_cmd = now\n指令发出", "t_mcu\n云台执行", "t_muzzle\n出膛", "t_impact\n命中"]
    for (tx, _), nm in zip(ticks, names):
        p.append(f'<line x1="{tx}" y1="{y - 22}" x2="{tx}" y2="{y + 40}" stroke="{MUTED}" stroke-width="1"/>')
        a, b = nm.split("\n")
        text(p, tx, y - 34, a, 11.5, INK, "middle", "bold")
        text(p, tx, y - 22 + 0, "", 1)
        text(p, tx + 4, y - 8, b, 10, MUTED)
    # brackets
    def bracket(xa, xb, yy, label, colour):
        p.append(f'<path d="M{xa},{yy - 6} L{xa},{yy} L{xb},{yy} L{xb},{yy - 6}" fill="none" stroke="{colour}" stroke-width="1.6"/>')
        text(p, xa + 4, yy + 16, label, 11.5, colour)
    bracket(x0, x0 + 270, y + 92, "elapsed = now − t_exp（上车每帧实测；离线用 transfer_delay + process_delay）", TEAL)
    bracket(x0 + 270, x0 + 630, y + 128, "comm_delay + fire_delay（配置值，暂无逐帧测量手段）", AMBER)
    bracket(x0, x0 + 630, y + 164, "出膛前延迟 L = latency + elapsed + comm_delay + fire_delay ≈ 65 ms", INK)
    bracket(x0, x0 + 960, y + 200, "瞄准时刻 T = t_exp + L + t_fly（总提前量 ≈ 190–365 ms，随距离变）", "#a78bfa")

    # ---------------- 2. pipeline --------------------------------------
    yb = 420
    text(p, 24, yb - 14, "② 每帧计算流程", 14, INK, weight="bold")
    bw, bh, gap = 176, 168, 14
    blocks = [
        ("1 打戳与姿态", BLUE, ["曝光中心戳 t_exp", "PoseBuffer 在 t_exp", "SLERP 插值云台姿态", "→ 世界系（重力对齐）", "无姿态样本 → 丢帧"]),
        ("2 检测 + PnP", AMBER, ["灯条 → 装甲板四角点", "IPPE 双解 + yaw 搜索", "板位置 p、朝向 yaw", "距离 d = |p|", "（距离也下发 MCU）"]),
        ("3 整车 EKF", TEAL, ["状态：中心 x,y,z 与速度", "yaw、ω、r1、r2、dz", "在 t_exp 融合观测", "predict(t)：Singer 阻尼", "速度 + 匀角速转动"]),
        ("4 飞行时间迭代", TEAL, ["t_fly ← 初值", "x̂ = predict(t_exp+L+t_fly)", "选板 k → 瞄准点 P", "solve(P) → pitch, t_fly′", "|Δt| < 0.1 ms 或 6 次"]),
        ("5 开火判定", RED, ["TRACKING 且已收敛", "|朝向角| < 30°", "σ 门控（可选，哨兵关）", "云台在板角宽内", "|ω| > 12 → 中心模式"]),
        ("6 串口下发", BLUE, ["yaw, pitch (+补偿)", "yaw_vel 前馈", "distance_mm", "found, fire", "21 B 帧 → MCU"]),
    ]
    xs = 24
    for i, (title, colour, lines) in enumerate(blocks):
        bx = xs + i * (bw + gap)
        box(p, bx, yb, bw, bh, stroke=colour)
        text(p, bx + bw / 2, yb + 24, title, 13, colour, "middle", "bold")
        for j, ln in enumerate(lines):
            text(p, bx + 10, yb + 50 + j * 22, ln, 11.2, INK)
        if i < len(blocks) - 1:
            arrow(p, bx + bw + 1, yb + bh / 2, bx + bw + gap - 1, yb + bh / 2)
    # loop arrow on block 4
    b4 = xs + 3 * (bw + gap)
    p.append(f'<path d="M{b4 + bw - 14},{yb + bh - 8} C{b4 + bw + 6},{yb + bh + 30} {b4 - 6},{yb + bh + 30} {b4 + 14},{yb + bh - 8}" '
             f'fill="none" stroke="{TEAL}" stroke-width="1.6" marker-end="url(#at)"/>')
    text(p, b4 + bw / 2, yb + bh + 34, "t_fly 依赖 P 的距离，P 又依赖 t_fly：不动点迭代", 10.5, TEAL, "middle")

    # ---------------- 3. prediction geometry ---------------------------
    yg = 660
    text(p, 24, yg - 14, "③ 综合预测：瞄准的是“子弹到达时”装甲板的位置", 14, INK, weight="bold")
    box(p, 24, yg, 560, 210)
    # top view: robot at time t_exp (grey) and at T (teal)
    cx0, cy0, cx1, cy1, r = 120, yg + 105, 235, yg + 95, 50
    for (cx, cy, col, yaw0, lab) in ((cx0, cy0, MUTED, 0.3, "t_exp 时"), (cx1, cy1, TEAL, 1.25, "T 时（预测）")):
        p.append(f'<circle cx="{cx}" cy="{cy}" r="{r}" fill="none" stroke="{col}" stroke-dasharray="4 3"/>')
        for k in range(4):
            a = yaw0 + k * math.pi / 2
            ax, ay = cx - r * math.cos(a), cy - r * math.sin(a)
            dx, dy = 14 * math.sin(a), -14 * math.cos(a)
            p.append(f'<line x1="{ax - dx:.1f}" y1="{ay - dy:.1f}" x2="{ax + dx:.1f}" y2="{ay + dy:.1f}" stroke="{col}" stroke-width="4"/>')
        p.append(f'<circle cx="{cx}" cy="{cy}" r="3" fill="{col}"/>')
        text(p, cx, cy + r + 22, lab, 11, col, "middle")
    arrow(p, cx0 + 4, cy0 - 2, cx1 - 6, cy1 - 1, TEAL, "at")
    text(p, (cx0 + cx1) / 2, cy0 - 12, "v·Δt", 11, TEAL, "middle")
    # aimed plate at T: the one facing the barrel
    a = 1.25 + math.pi / 2 * 2
    px_, py_ = cx1 - r * math.cos(a), cy1 - r * math.sin(a)
    p.append(f'<circle cx="{px_:.1f}" cy="{py_:.1f}" r="8" fill="none" stroke="{RED}" stroke-width="2"/>')
    text(p, px_ + 12, py_ + 4, "P：被选中的板", 11, RED)
    # barrel at the left
    p.append(f'<circle cx="44" cy="{yg + 190}" r="5" fill="{INK}"/>')
    text(p, 54, yg + 194, "枪口", 10.5, MUTED)
    p.append(f'<line x1="44" y1="{yg + 190}" x2="{px_:.1f}" y2="{py_:.1f}" stroke="{RED}" stroke-dasharray="5 4"/>')
    for j, ln in enumerate([
        "中心 c(T) = c + v·τ(1 − e^(−Δt/τ))",
        "（Singer 速度阻尼，τ = 0.3 s）",
        "第 k 块板角度 θk = yaw + ω·Δt + k·π/2",
        "板位置 Pk = c(T) − r_k·(cos θk, sin θk) + dz_k",
        "Δt = T − t_exp = L + t_fly",
    ]):
        text(p, 318, yg + 36 + j * 22, ln, 10.8, INK)

    box(p, 600, yg, 556, 210)
    for j, (ln, col) in enumerate([
        ("为什么每一段都要算进去", INK),
        ("10 rad/s 小陀螺：Δt 每错 10 ms，板的相位错 5.7°（半径 0.25 m 处约 2.5 cm）", MUTED),
        ("仿真 165 Hz：漏算 30 ms 发射延迟 → 10 rad/s 命中率 98.9% → 65.3%", AMBER),
        ("串口+MCU 抖动 ±10 ms → 8 场景平均只掉约 0.5 pp", MUTED),
        ("估短比估长代价大：拿不准时宁可让模型略长", INK),
        ("云台滞后补偿 gimbal_lag：仿真 +1 pp，真实视频退化 → 默认 0", RED),
        ("帧率无关：门限按时间（tracking 80 ms、丢帧宽限 20 ms）", TEAL),
        ("数据：docs/studies/latency_chain_165hz.md", MUTED),
    ]):
        text(p, 616, yg + 28 + j * 23, ln, 11.5 if j == 0 else 10.8, col, weight="bold" if j == 0 else "normal")
    save(p, "aim_timing_principle.svg")


if __name__ == "__main__":
    pipeline()
    time_alignment()
    robot_model()
    aim_timing_principle()
