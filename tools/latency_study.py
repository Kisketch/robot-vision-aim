#!/usr/bin/env python3
"""Latency chain + 165 Hz study (docs/studies/latency_chain_165hz.md).

Runs autoaim_sim (ground-truth simulator) for
  1. 25 Hz vs 165 Hz, per scenario;
  2. latency model error sweep at 165 Hz (true departure = model + error),
     with and without +-10 ms jitter (the measured serial + MCU spread);
  3. named what-if cases (fire delay forgotten, old lumped 60 ms, ...);
writes tools/latency_study.json and draws three SVG charts into docs/charts/
in the same style as tools/make_charts.py.

    python3 tools/latency_study.py                 # run sims + draw
    python3 tools/latency_study.py --plot-only     # redraw from the JSON

Simulator absolute values drift a few points between machines (BASELINE.md);
compare rows of one run, not runs across machines.
"""
import argparse
import json
import math
import re
import subprocess
from pathlib import Path

import make_charts as mc

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "tools" / "latency_study.json"
CFG_165 = "sentry/config/sentry_165hz.yaml"
CFG_OLD = "sentry/config/sentry.yaml"

# Team measurements, 2026-11-17 (ms): (label, low, model, high)
CHAIN = [
    ("曝光中心→主机内存", 6, 10, 14),
    ("检测+解算+瞄准", 2.5, 5, 7),
    ("串口+MCU", 10, 20, 30),
    ("发射机构", 30, 30, 30),
]


def run_sim(exe, *args):
    out = subprocess.run([str(exe), *map(str, args)], capture_output=True, text=True, check=True).stdout
    rows = []
    for line in out.splitlines():
        if not line.startswith("| ") or line.startswith("| scenario"):
            continue
        cells = [c.strip() for c in line.strip("|").split("|")]
        rows.append({"scenario": cells[0], "hit": float(cells[3]), "hits_s": float(cells[4])})
    if len(rows) != 8:
        raise RuntimeError(f"unexpected simulator output:\n{out}")
    return rows


def mean(rows, key="hit"):
    return sum(r[key] for r in rows) / len(rows)


def flight_time(dist, v0=25.0, k=0.019, g=9.8):
    """Flat-fire flight time with quadratic drag (same model as Ballistics)."""
    x, z, vx, vz, t, dt = 0.0, 0.0, v0, 0.0, 0.0, 1e-4
    while x < dist:
        v = math.hypot(vx, vz)
        vx, vz = vx - k * v * vx * dt, vz - (g + k * v * vz) * dt
        x, z, t = x + vx * dt, z + vz * dt, t + dt
    return t


def collect(exe):
    d = {"fps": {}, "sweep": {}, "cases": {}}
    d["fps"]["25 Hz (sentry.yaml, lumped 60 ms)"] = run_sim(exe, "--config", CFG_OLD, "--fps", 25)
    d["fps"]["165 Hz (sentry_165hz.yaml)"] = run_sim(exe, "--config", CFG_165, "--fps", 165)
    d["fps"]["165 Hz + 10 ms jitter"] = run_sim(exe, "--config", CFG_165, "--fps", 165, "--latency-jitter", 0.010)
    for jit in (0, 10):
        pts = []
        for err in range(-40, 41, 10):
            rows = run_sim(exe, "--config", CFG_165, "--fps", 165, "--latency-error", err / 1000,
                           "--latency-jitter", jit / 1000)
            pts.append({"err_ms": err, "mean": mean(rows), "spin10": rows[2]["hit"],
                        "spin2": rows[0]["hit"], "approach": rows[4]["hit"]})
        d["sweep"][str(jit)] = pts
    # What-if: the aimer's model vs the real chain (model 65 ms everywhere;
    # the error is what the real chain adds on top of what the model knows).
    cases = [
        ("链路实测 + 每帧测处理耗时（本次方案）", 0, 10),
        ("沿用旧的 60 ms 打包值", 5, 12),
        ("漏算发射延迟 30 ms", 30, 12),
        ("串口按 10 ms 下限估", 10, 10),
        ("串口按 30 ms 上限估", -10, 10),
    ]
    for name, err, jit in cases:
        rows = run_sim(exe, "--config", CFG_165, "--fps", 165, "--latency-error", err / 1000,
                       "--latency-jitter", jit / 1000)
        d["cases"][name] = {"err_ms": err, "jit_ms": jit, "mean": mean(rows), "spin10": rows[2]["hit"],
                            "hits_s": mean(rows, "hits_s")}
    d["t_fly_ms"] = {str(m): round(flight_time(m) * 1000, 1) for m in (3, 5, 7)}
    return d


# ------------------------------------------------------------------ charts
def chart_chain(d):
    w, h = 900, 330
    tf = d["t_fly_ms"]["5"]
    p = mc.open_svg(w, h, "延迟链：从曝光到弹丸命中（165 Hz 相机；数值为队伍提供，未经实拍验证）",
                    "上：出膛前延迟（条宽 = 模型值，细线 = 范围）；下：它与弹丸飞行时间（5 m，25 m/s，阻力 k=0.019）的占比")
    colours = [mc.BLUE, mc.TEAL, mc.AMBER, mc.RED]
    lead = sum(m for _, _, m, _ in CHAIN)
    x0, x1 = 40, w - 40
    # row 1: the pre-muzzle chain, full width
    y, bh = 92, 40
    scale = (x1 - x0) / lead
    x = x0
    for (name, lo, m, hi), c in zip(CHAIN, colours):
        bw = m * scale
        p.append(f'<rect x="{x:.1f}" y="{y}" width="{bw - 2:.1f}" height="{bh}" fill="{c}" opacity="0.9"/>')
        p.append(f'<text x="{x + bw / 2:.1f}" y="{y + 25}" font-size="12" font-weight="bold" '
                 f'text-anchor="middle" fill="#0f172a">{m:g} ms</text>')
        rng = f"{lo:g}–{hi:g} ms" if hi > lo else "固定"
        p.append(f'<text x="{x + bw / 2:.1f}" y="{y + bh + 18}" font-size="11" text-anchor="middle" '
                 f'fill="{mc.INK}">{name}</text>')
        p.append(f'<text x="{x + bw / 2:.1f}" y="{y + bh + 33}" font-size="10.5" text-anchor="middle" '
                 f'fill="#94a3b8">{rng}</text>')
        x += bw
    p.append(f'<text x="{x0}" y="{y - 8}" font-size="11.5" fill="{mc.INK}">出膛前 ≈ {lead:g} ms'
             f'（前两段上车时每帧实测，取代模型值）</text>')
    # row 2: pre-muzzle vs flight
    y2 = 210
    total = lead + tf
    scale2 = (x1 - x0) / total
    p.append(f'<rect x="{x0}" y="{y2}" width="{lead * scale2 - 2:.1f}" height="28" fill="{mc.AMBER}"/>')
    p.append(f'<rect x="{x0 + lead * scale2:.1f}" y="{y2}" width="{tf * scale2:.1f}" height="28" fill="#a78bfa"/>')
    p.append(f'<text x="{x0 + lead * scale2 / 2:.1f}" y="{y2 + 19}" font-size="11.5" text-anchor="middle" '
             f'fill="#0f172a">出膛前 {lead:g} ms</text>')
    p.append(f'<text x="{x0 + lead * scale2 + tf * scale2 / 2:.1f}" y="{y2 + 19}" font-size="11.5" '
             f'text-anchor="middle" fill="#0f172a">弹丸飞行 {tf:g} ms（3 m {d["t_fly_ms"]["3"]:g} / 7 m {d["t_fly_ms"]["7"]:g}）</text>')
    p.append(f'<text x="{x0}" y="{y2 + 56}" font-size="11.5" fill="{mc.INK}">总提前量 ≈ {total:.0f} ms：瞄准点 = '
             f'目标在「曝光时刻 + {lead:g} ms + 飞行时间」的预测位置；飞行时间随距离由弹道解算迭代求出</text>')
    p.append(f'<text x="{x0}" y="{y2 + 76}" font-size="11" fill="#94a3b8">10 rad/s 小陀螺上，提前量每错 10 ms，'
             f'装甲板相位就错 5.7°（半径 0.25 m 处约 2.5 cm）</text>')
    mc.close_svg(p, "latency_chain.svg")


def chart_error(d):
    w, h = 900, 360
    p = mc.open_svg(w, h, "延迟估错的代价〔仿真推演 165 Hz，非实拍〕",
                    "横轴：真实出膛时刻 − 模型时刻；实线 = 10 rad/s 快速小陀螺，虚线 = 8 场景平均；橙 = 无抖动，蓝 = ±10 ms 抖动")
    x0, x1, y0, y1 = 70, 860, 70, 310
    vmin, vmax = 40, 100
    for t in range(40, 101, 10):
        y = y1 - (t - vmin) / (vmax - vmin) * (y1 - y0)
        p.append(f'<line x1="{x0}" y1="{y:.0f}" x2="{x1}" y2="{y:.0f}" stroke="{mc.GRID}"/>')
        p.append(f'<text x="{x0 - 6}" y="{y + 4:.0f}" font-size="10.5" text-anchor="end" fill="#94a3b8">{t}%</text>')
    errs = [pt["err_ms"] for pt in d["sweep"]["0"]]
    def X(e):
        return x0 + (e - errs[0]) / (errs[-1] - errs[0]) * (x1 - x0)
    for e in errs:
        p.append(f'<text x="{X(e):.0f}" y="{y1 + 18}" font-size="10.5" text-anchor="middle" fill="#94a3b8">{e:+d} ms</text>')
    p.append(f'<line x1="{X(0):.0f}" y1="{y0}" x2="{X(0):.0f}" y2="{y1}" stroke="{mc.GRAY}" stroke-dasharray="3 3"/>')
    for jit, colour in (("0", mc.AMBER), ("10", mc.BLUE)):
        for key, dash in (("spin10", ""), ("mean", ' stroke-dasharray="6 4"')):
            pts = " ".join(f"{X(pt['err_ms']):.1f},{y1 - (pt[key] - vmin) / (vmax - vmin) * (y1 - y0):.1f}"
                           for pt in d["sweep"][jit])
            p.append(f'<polyline points="{pts}" fill="none" stroke="{colour}" stroke-width="2.2"{dash}/>')
    p.append(f'<text x="{X(20):.0f}" y="{y0 + 14}" font-size="11" fill="{mc.INK}">真实链路比模型长 → 打在板后</text>')
    p.append(f'<text x="{x0 + 10}" y="{y1 - 10}" font-size="11" fill="#94a3b8">左侧（比模型短）偏高来自仿真云台的 30 ms 一阶滞后；'
             f'按此加补偿在真实视频上反而退化，未采用</text>')
    mc.close_svg(p, "latency_error.svg")


def chart_fps(d):
    w, h = 900, 380
    names = list(d["fps"].keys())
    p = mc.open_svg(w, h, "25 Hz vs 165 Hz〔两者均为仿真；真实视频只有 25 fps〕",
                    "165 Hz 配置：延迟链 65 ms、按时间计的门限、射频上限 20 发/s；同机同种子")
    rows0 = d["fps"][names[0]]
    x0, y0, y1 = 70, 70, 300
    mc.y_axis(p, x0, y0, y1, 100, [0, 25, 50, 75, 100], "%")
    colours = [mc.GRAY, mc.TEAL, mc.BLUE]
    group = (820) / len(rows0)
    bw = group / (len(names) + 1)
    short = ["2.1 rad/s", "6 rad/s+横移", "10 rad/s", "强横移", "接近 7→3 m", "变速", "摇摆", "3 rad/s+轻横移"]
    for i in range(len(rows0)):
        for j, n in enumerate(names):
            v = d["fps"][n][i]["hit"]
            x = x0 + i * group + (j + 0.5) * bw
            hh = v / 100 * (y1 - y0)
            p.append(f'<rect x="{x:.1f}" y="{y1 - hh:.1f}" width="{bw * 0.9:.1f}" height="{hh:.1f}" fill="{colours[j]}"/>')
        p.append(f'<text x="{x0 + (i + 0.5) * group:.0f}" y="{y1 + 16}" font-size="10.5" text-anchor="middle" '
                 f'fill="#94a3b8">{short[i]}</text>')
    for j, n in enumerate(names):
        p.append(f'<rect x="{x0 + j * 270}" y="{h - 34}" width="12" height="12" fill="{colours[j]}"/>')
        p.append(f'<text x="{x0 + j * 270 + 18}" y="{h - 24}" font-size="11" fill="{mc.INK}">{n}: 平均 '
                 f'{mean(d["fps"][n]):.1f}%</text>')
    mc.close_svg(p, "latency_fps.svg")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", default=str(ROOT / "build" / "apps" / "autoaim_sim"))
    ap.add_argument("--plot-only", action="store_true")
    a = ap.parse_args()
    if a.plot_only:
        d = json.loads(DATA.read_text(encoding="utf-8"))
    else:
        d = collect(a.sim)
        DATA.write_text(json.dumps(d, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
        print("wrote", DATA.relative_to(ROOT))
    chart_chain(d)
    chart_error(d)
    chart_fps(d)
    for name, c in d["cases"].items():
        print(f"{name:24s} err {c['err_ms']:+d} ms jit +-{c['jit_ms']} ms: mean {c['mean']:.1f}%  "
              f"10 rad/s {c['spin10']:.1f}%  {c['hits_s']:.2f} hits/s")


if __name__ == "__main__":
    main()
