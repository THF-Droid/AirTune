#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AirTune CLI 采集闭环：一条命令 -> 数据 -> CSV -> 图。

固件命令（两种流，脚本自动识别）：
  log <n> <ms>     AirTune 推理遥测
      LOG,<tick_ms>,<in_k>,<pred_mode>,<cycles>,<tx_dbm>,<sf> ... LOG_END
  pwrlog <n> <ms>  INA226 功率遥测
      PWR,<tick_ms>,<vbus_mV>,<current_mA>,<power_mW>,<vshunt_uV> ... PWR_END

用法（一条命令出一张图）：
    python capture.py "pwrlog 300 20"      # 功率曲线：CSV + PNG
    python capture.py "log 200 20"         # 推理延迟 + LoRa 模式图
    python capture.py "log 300 10" --port COM5
    python capture.py --demo               # 无板子自测（可带命令名，如 --demo pwrlog 100 20）
"""
import sys, os, csv, argparse, time

MODE_NAME = {0: "SF7/BW500", 1: "SF7/BW125", 2: "SF10/BW125", 3: "SF12/BW125"}


def kind_of(cmd):
    """按命令名判断数据流类型：pwrlog -> pwr，其余按 log 处理。"""
    return "pwr" if cmd.strip().split(" ")[0].startswith("pwr") else "log"


# ---------------------------------------------------------------- LOG (推理)
def parse_log(lines):
    rows = []
    for ln in lines:
        ln = ln.strip()
        if ln.startswith("LOG,"):
            p = ln.split(",")
            if len(p) >= 7:
                try:
                    rows.append({"tick_ms": int(p[1]), "in_k": int(p[2]),
                                 "pred_mode": int(p[3]), "cycles": int(p[4]),
                                 "tx_dbm": int(p[5]), "sf": int(p[6])})
                except ValueError:
                    pass
    return rows


def write_csv_log(rows, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tick_ms", "in_k", "pred_mode", "cycles", "latency_us", "tx_dbm", "sf"])
        for r in rows:
            w.writerow([r["tick_ms"], r["in_k"], r["pred_mode"], r["cycles"],
                        round(r["cycles"] / 100.0, 2), r["tx_dbm"], r["sf"]])


def plot_log(rows, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    idx = list(range(len(rows)))
    lat = [r["cycles"] / 100.0 for r in rows]
    mode = [r["pred_mode"] for r in rows]
    fig, ax1 = plt.subplots(figsize=(9.5, 4.8))
    for i, r in enumerate(rows):
        ax1.axvspan(i - 0.5, i + 0.5, color=plt.cm.tab10(r["in_k"] % 10), alpha=0.07)
    ax1.plot(idx, lat, color="#1f77b4", lw=1.5, label="inference latency (us)")
    ax1.set_xlabel("sample #")
    ax1.set_ylabel("latency (us)  [DWT CYCCNT @100MHz]", color="#1f77b4")
    ax1.tick_params(axis="y", labelcolor="#1f77b4")
    ax1.grid(alpha=0.3)
    ax2 = ax1.twinx()
    ax2.step(idx, mode, where="mid", color="#d62728", lw=1.6, label="predicted LoRa mode")
    ax2.set_ylabel("predicted LoRa mode", color="#d62728")
    ax2.set_yticks([0, 1, 2, 3])
    ax2.set_yticklabels(["SF7/BW500", "SF7/BW125", "SF10/BW125", "SF12/BW125"], fontsize=8)
    ax2.set_ylim(-0.5, 3.5)
    ax2.tick_params(axis="y", labelcolor="#d62728")
    avg = sum(lat) / len(lat) if lat else 0.0
    ax1.set_title(f"{title}   |   n={len(rows)}  avg={avg:.2f}us  min={min(lat):.2f}us  max={max(lat):.2f}us")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------- PWR (功率)
def parse_pwr(lines):
    rows = []
    for ln in lines:
        ln = ln.strip()
        if ln.startswith("PWR,"):
            p = ln.split(",")
            if len(p) >= 6:
                try:
                    rows.append({"tick_ms": int(p[1]), "vbus_mV": int(p[2]),
                                 "current_mA": int(p[3]), "power_mW": int(p[4]),
                                 "vshunt_uV": int(p[5])})
                except ValueError:
                    pass
    return rows


def write_csv_pwr(rows, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tick_ms", "vbus_mV", "current_mA", "power_mW", "vshunt_uV"])
        for r in rows:
            w.writerow([r["tick_ms"], r["vbus_mV"], r["current_mA"],
                        r["power_mW"], r["vshunt_uV"]])


def plot_pwr(rows, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    idx = list(range(len(rows)))
    cur = [r["current_mA"] for r in rows]
    pwr = [r["power_mW"] for r in rows]
    vbs = [r["vbus_mV"] for r in rows]
    fig, ax1 = plt.subplots(figsize=(9.5, 4.8))
    ax1.plot(idx, cur, color="#1f77b4", lw=1.4, label="current (mA)")
    ax1.set_xlabel("sample #")
    ax1.set_ylabel("current (mA)  [INA226 shunt]", color="#1f77b4")
    ax1.tick_params(axis="y", labelcolor="#1f77b4")
    ax1.grid(alpha=0.3)
    ax2 = ax1.twinx()
    ax2.plot(idx, pwr, color="#ff7f0e", lw=1.2, alpha=0.85, label="power (mW)")
    ax2.set_ylabel("power (mW)", color="#ff7f0e")
    ax2.tick_params(axis="y", labelcolor="#ff7f0e")
    avg_i = sum(cur) / len(cur) if cur else 0.0
    avg_v = sum(vbs) / len(vbs) if vbs else 0.0
    ax1.set_title(f"{title}   |   n={len(rows)}  I: avg={avg_i:.1f}mA "
                  f"min={min(cur)}mA max={max(cur)}mA   Vbus avg={avg_v:.0f}mV")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------- 串口 / demo
def capture_serial(port, baud, cmd, timeout):
    import serial
    ser = serial.Serial(port, baud, timeout=0.2)
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()
    lines = []
    t0 = time.time()
    done = False
    while (time.time() - t0) < timeout and not done:
        raw = ser.readline()
        if not raw:
            continue
        ln = raw.decode(errors="ignore").strip()
        if ln:
            lines.append(ln)
            if ln.endswith("_END"):        # LOG_END 或 PWR_END
                done = True
    ser.close()
    return lines


def demo_lines(cmd, n=200):
    import random
    random.seed(1)
    parts = cmd.split()                       # 从命令解析样本数，如 "pwrlog 100 20" -> 100
    if len(parts) >= 2:
        try:
            n = max(1, int(parts[1]))
        except ValueError:
            pass
    if kind_of(cmd) == "pwr":
        lines = [f"PWR_BEGIN {n} 20"]
        for i in range(n):
            burst = (i % 10) < 2                       # LoRa 周期性发射突发
            cur = 4 + (105 if burst else 0) + random.randint(-2, 2)
            vbs = 3300 + random.randint(-3, 3)
            lines.append(f"PWR,{i*20},{vbs},{cur},{vbs*cur//1000},{cur*0}")  # 演示数据
        lines.append("PWR_END")
        return lines
    lines = [f"LOG_BEGIN {n} 20"]
    for i in range(n):
        k = i & 3
        cyc = 2100 + random.randint(-50, 50)
        lines.append(f"LOG,{i*20},{k},{k},{cyc},17,7")
    lines.append("LOG_END")
    return lines


def main():
    ap = argparse.ArgumentParser(description="AirTune CLI capture: command -> data -> CSV -> chart")
    ap.add_argument("cmd", nargs="*", default=None, help='CLI command, e.g. "pwrlog 300 20"')
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=25.0)
    ap.add_argument("--csv", default=None)
    ap.add_argument("--png", default=None)
    ap.add_argument("--demo", action="store_true", help="no-board self test with synthetic data")
    a = ap.parse_args()

    cmd = " ".join(a.cmd) if a.cmd else "log 200 20"
    kind = kind_of(cmd)
    tag = cmd.replace(" ", "_").replace("/", "-")

    if a.demo:
        lines = demo_lines(cmd)
    else:
        print(f"[..] 打开 {a.port} @{a.baud}，发送命令: {cmd}")
        lines = capture_serial(a.port, a.baud, cmd, a.timeout)

    rows = parse_pwr(lines) if kind == "pwr" else parse_log(lines)
    if not rows:
        print(f"[!] 没采集到 {'PWR' if kind == 'pwr' else 'LOG'} 数据。排查："
              "① 串口工具是否占用了 COM4（先关掉）；② 板子是否烧了带该命令的固件；③ 命令是否以换行结束。")
        print("    收到的原始行(前 20 条):")
        for ln in lines[:20]:
            print("      ", ln)
        sys.exit(2)

    csv_path = a.csv or f"capture_{tag}.csv"
    png_path = a.png or f"capture_{tag}.png"
    if kind == "pwr":
        write_csv_pwr(rows, csv_path)
        plot_pwr(rows, png_path, f"AirTune INA226 power capture: '{cmd}'")
    else:
        write_csv_log(rows, csv_path)
        plot_log(rows, png_path, f"AirTune CLI capture: '{cmd}'")

    print(f"[OK] 命令 '{cmd}' -> {len(rows)} 样本")
    if kind == "pwr":
        cur = [r["current_mA"] for r in rows]
        vbs = [r["vbus_mV"] for r in rows]
        pwr = [r["power_mW"] for r in rows]
        print(f"     电流  min={min(cur)}mA  avg={sum(cur)/len(cur):.1f}mA  max={max(cur)}mA")
        print(f"     电压  min={min(vbs)}mV  avg={sum(vbs)/len(vbs):.0f}mV  max={max(vbs)}mV")
        print(f"     功率  min={min(pwr)}mW  avg={sum(pwr)/len(pwr):.1f}mW  max={max(pwr)}mW")
    else:
        lat = [r["cycles"] / 100.0 for r in rows]
        modes = [r["pred_mode"] for r in rows]
        dist = ", ".join(f"{MODE_NAME.get(m, m)}={modes.count(m)}" for m in sorted(set(modes)))
        print(f"     推理延迟  min={min(lat):.2f}us  avg={sum(lat)/len(lat):.2f}us  max={max(lat):.2f}us")
        print(f"     预测模式分布: {dist}")
    print(f"     CSV -> {os.path.abspath(csv_path)}")
    print(f"     PNG -> {os.path.abspath(png_path)}")


if __name__ == "__main__":
    main()
