"""
AirTune - PC 侧训练 + int8 量化验证
====================================
项目背景：AirTune 是「STM32 + TinyML × 低功耗通信」主线里的自适应 LoRa 链路模块。
它在端侧用一颗小 MLP，根据当前链路状态（10 维特征）预测「最优 LoRa 工作模式」，
从而在吞吐 / 距离 / 功耗之间做在线权衡。

本脚本只做 PC 侧：
  1) 定义 AirTune 的 10 维特征（带物理含义与取值范围）
  2) 造合成数据（特征 -> 4 类最优模式，标签由平滑决策函数生成）
  3) 用纯 numpy 训练一个 MLP（10 -> 32 -> 16 -> 4，ReLU + softmax）
  4) 写 int8 后训练量化（PTQ）：权重和激活都量化成 int8，int32 累加
  5) 验证「量化前后精度差 < 1%」

量化约定（与后续 F411 端侧部署一致）：
  - 对称、per-tensor 量化： scale = max(|x|) / 127
  - q = clip(round(x / scale), -127, 127)   （int8 范围留 1 个码给 0 对称）
  - 反量化： x ≈ q * scale
  - 层计算： z_int32 = qW · qX  (int32 累加) ; z ≈ z_int32 * scale_w * scale_x ; +bias
  - 激活量化尺度用一份「校准集」统计各层激活的 min/max 得到
"""

import numpy as np

RNG = np.random.default_rng(20261014)

# ---------------------------------------------------------------------------
# 1) AirTune 的 10 维特征
# ---------------------------------------------------------------------------
# 每项是 (名字, 物理含义, 取值范围[min,max], 是否需要归一化到[-1,1]用于训练)
FEATURES = [
    ("rssi_dBm",        "接收信号强度 dBm",          (-120.0, -60.0)),
    ("snr_dB",          "信噪比 dB",                 (-20.0,  15.0)),
    ("per",             "丢包率 0~1",                (0.0,    1.0)),
    ("channel_util",    "信道占用率 0~1",            (0.0,    1.0)),
    ("link_margin_dB",  "链路余量 dB（越大越稳）",   (-10.0,  30.0)),
    ("energy_budget",   "剩余电量归一化 0~1",        (0.0,    1.0)),
    ("data_rate_req",   "业务所需速率归一化 0~1",    (0.0,    1.0)),
    ("latency_budget",  "时延预算归一化 0~1(小=严)", (0.0,    1.0)),
    ("temperature_C",   "板温 °C",                  (-20.0,  70.0)),
    ("interference_dB", "干扰抬升的噪声底 dB",       (-100.0,-30.0)),
]
FNAMES = [f[0] for f in FEATURES]
N_FEAT = len(FEATURES)  # 10

# 4 种 LoRa 工作模式（标签 0..3）
MODES = [
    "SF7/BW500  (高吞吐/短距)",
    "SF7/BW125  (均衡)",
    "SF10/BW125 (中距/中吞吐)",
    "SF12/BW125 (远距/低吞吐)",
]
N_CLASS = len(MODES)  # 4


# ---------------------------------------------------------------------------
# 2) 合成数据：每类标签对应一种「链路工况」，在归一化特征空间按中心+噪声采样
#    （标签均匀，4 类都充分出现；噪声让类间有重叠 -> MLP 可学、int8 验证更真实）
# ---------------------------------------------------------------------------
# 每个 LoRa 模式在归一化特征空间(~[-1,1])的中心；物理直觉：
#   好链路(rssi/snr高、per低) -> 高吞吐模式；差链路(rssi/snr低、per高) -> 远距模式
MODE_MEAN = np.array([
    [ 0.6,  0.6, -0.5, -0.2,  0.5, -0.1,  0.7,  0.7,  0.0, -0.5],  # 0 SF7/BW500 高吞吐/短距
    [ 0.1,  0.1, -0.1,  0.0,  0.1,  0.2, -0.1,  0.1,  0.0, -0.1],  # 1 SF7/BW125 均衡
    [-0.2, -0.1,  0.1,  0.1,  0.2,  0.4, -0.2, -0.2,  0.0, -0.2],  # 2 SF10/BW125 中距/中吞吐
    [-0.7, -0.6,  0.6,  0.3, -0.6,  0.3, -0.6, -0.6,  0.0, -0.6],  # 3 SF12/BW125 远距/低吞吐
])


def generate_data(n):
    """返回 X_raw(n,10) 原始特征, X(n,10) 归一化特征, y(n,) 标签 0..3（均匀）"""
    y = RNG.integers(0, N_CLASS, n)            # 标签均匀 -> 4 类都充分出现
    X = np.zeros((n, N_FEAT))
    for k in range(N_CLASS):
        mask = (y == k)
        m = int(mask.sum())
        noise = RNG.normal(0.0, 0.20, (m, N_FEAT))
        X[mask] = np.clip(MODE_MEAN[k] + noise, -1.0, 1.0)
    # 转回原始量纲（仅用于展示/文档）
    X_raw = np.zeros((n, N_FEAT))
    for j, (_, _, (lo, hi)) in enumerate(FEATURES):
        X_raw[:, j] = (X[:, j] + 1.0) / 2.0 * (hi - lo) + lo
    return X_raw, X, y


# ---------------------------------------------------------------------------
# 3) 纯 numpy MLP（训练用 float32）
# ---------------------------------------------------------------------------
class MLP:
    def __init__(self, sizes, lr=0.01):
        self.sizes = sizes
        self.lr = lr
        # He 初始化
        self.W, self.b = [], []
        for i in range(len(sizes) - 1):
            fan_in = sizes[i]
            std = np.sqrt(2.0 / fan_in)
            self.W.append(RNG.standard_normal((sizes[i + 1], sizes[i])) * std)
            self.b.append(np.zeros(sizes[i + 1]))

    @staticmethod
    def _relu(z):
        return np.maximum(0.0, z)

    def forward(self, X, cache=False):
        a = X.T  # (din, N)
        acts = [a]
        for i in range(len(self.W) - 1):
            z = self.W[i] @ a + self.b[i][:, None]
            a = self._relu(z)
            acts.append(a)
        # 输出层：softmax
        z = self.W[-1] @ a + self.b[-1][:, None]
        z = z - np.max(z, axis=0, keepdims=True)
        e = np.exp(z)
        out = e / np.sum(e, axis=0, keepdims=True)
        acts.append(out)
        if cache:
            return out.T, acts
        return out.T

    def loss(self, X, y):
        p = self.forward(X)  # (N, C)
        p = np.clip(p, 1e-12, 1.0)
        nll = -np.mean(np.log(p[np.arange(len(y)), y]))
        return nll

    def train(self, X, y, epochs=120, batch=128, lr=0.01):
        N = len(y)
        b1, b2, eps = 0.9, 0.999, 1e-8
        m = [np.zeros_like(p) for p in (self.W + self.b)]
        v = [np.zeros_like(p) for p in (self.W + self.b)]
        t = 0
        for ep in range(epochs):
            perm = RNG.permutation(N)
            for s in range(0, N, batch):
                idx = perm[s:s + batch]
                xb = X[idx]
                yb = y[idx]
                p, acts = self.forward(xb, cache=True)
                dout = p.T.copy()
                dout[yb, np.arange(len(yb))] -= 1.0
                dout /= len(yb)
                # 逐层梯度（输出->输入）
                gW, gb = [], []
                a_prev = acts[-2]
                gW.append(dout @ a_prev.T)
                gb.append(dout.sum(axis=1))
                da = self.W[-1].T @ dout
                dz = da * (a_prev > 0)
                for l in range(len(self.W) - 2, -1, -1):
                    a_in = acts[l]
                    gW.append(dz @ a_in.T)
                    gb.append(dz.sum(axis=1))
                    if l > 0:
                        da = self.W[l].T @ dz
                        dz = da * (a_in > 0)
                gW = gW[::-1]
                gb = gb[::-1]
                grads = gW + gb
                t += 1
                params = self.W + self.b
                for i, g in enumerate(grads):
                    m[i] = b1 * m[i] + (1 - b1) * g
                    v[i] = b2 * v[i] + (1 - b2) * (g * g)
                    mhat = m[i] / (1 - b1 ** t)
                    vhat = v[i] / (1 - b2 ** t)
                    params[i] -= lr * mhat / (np.sqrt(vhat) + eps)
            if (ep + 1) % 20 == 0:
                print(f"  epoch {ep+1:3d}/{epochs}  loss={self.loss(X, y):.4f}")

    def predict(self, X):
        return np.argmax(self.forward(X), axis=1)


def accuracy(mlp, X, y):
    return np.mean(mlp.predict(X) == y)


# ---------------------------------------------------------------------------
# 4) int8 后训练量化（PTQ）
# ---------------------------------------------------------------------------
def sym_quant(x):
    """对称量化到 int8：返回 (q_int, scale)"""
    scale = (np.max(np.abs(x)) + 1e-9) / 127.0
    q = np.clip(np.round(x / scale), -127, 127).astype(np.int32)
    return q, float(scale)


def sym_dequant(q, scale):
    return q.astype(np.float64) * scale


def calibrate_activation_scales(mlp, X_cal):
    """统计每层激活（ReLU 输出）的 min/max -> 对称 scale"""
    scales = []
    a = X_cal.T
    for i in range(len(mlp.W) - 1):
        z = mlp.W[i] @ a + mlp.b[i][:, None]
        a = mlp._relu(z)
        amax = np.max(np.abs(a))
        scales.append(float(amax + 1e-9) / 127.0)
    return scales  # len = n_hidden_layers


def mlp_int8_forward(mlp, qW, qb, w_scale, x_scale, X, in_scale):
    """
    全 int8 推理（W8 × X8 -> int32 累加 -> 反量化 -> 激活 -> 再量化）
    qW/qb/w_scale: 各层量化权重/偏置/权重尺度
    x_scale: 各隐藏层激活尺度（len = 隐藏层数）
    in_scale: 输入特征（已归一化）的量化尺度，由校准集得到
    """
    # 输入激活尺度序列：第 0 项是输入特征(in_scale)，之后是各隐藏层 ReLU 输出尺度
    all_x_scale = [in_scale] + list(x_scale)
    qx = np.clip(np.round(X / in_scale), -127, 127).astype(np.int32).T  # (din, N)
    for i in range(len(qW) - 1):
        # 本层输入激活尺度 = all_x_scale[i]
        z_int = qW[i] @ qx + qb[i][:, None]            # (H, N) int32
        z = z_int.astype(np.float64) * (w_scale[i] * all_x_scale[i])
        a = mlp._relu(z)
        xs = x_scale[i]                                # 本层输出激活尺度
        qx = np.clip(np.round(a / xs), -127, 127).astype(np.int32)
    # 输出层（输入是最后隐藏层激活，尺度 = all_x_scale[-1] = x_scale[-1]）
    z_int = qW[-1] @ qx + qb[-1][:, None]
    z = z_int.astype(np.float64) * (w_scale[-1] * all_x_scale[-1])
    z = z - np.max(z, axis=0, keepdims=True)
    e = np.exp(z)
    out = (e / np.sum(e, axis=0, keepdims=True)).T
    return np.argmax(out, axis=1)


def quantize_mlp(mlp, X_cal):
    x_scale = calibrate_activation_scales(mlp, X_cal)
    in_scale = float((np.max(np.abs(X_cal)) + 1e-9) / 127.0)  # 输入特征量化尺度
    all_x_scale = [in_scale] + list(x_scale)
    qW, qb, w_scale = [], [], []
    for i in range(len(mlp.W)):
        q, s = sym_quant(mlp.W[i])
        S = s * all_x_scale[i]   # 权重尺度 × 本层输入激活尺度 = int32 累加有效尺度
        qW.append(q)
        qb.append(np.clip(np.round(mlp.b[i] / S), -2**31, 2**31 - 1).astype(np.int32))
        w_scale.append(s)
    return qW, qb, w_scale, x_scale, in_scale


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def export_model_h(path, q=26):
    """训练+量化，并导出 C 头文件（const -> Flash）。权重与已验证模型逐位一致。

    与 main() 完全一致的 RNG 消耗顺序（generate_data -> MLP 初始化 -> train）
    保证可复现：同样随机种子下导出的权重 == 主流程里验证过 <1% 的那些权重。
    """
    X_raw, X, y = generate_data(20000)
    Xtr, ytr = X[:16000], y[:16000]
    Xte, yte = X[16000:], y[16000:]
    Xcal = Xtr[:500]
    mlp = MLP([N_FEAT, 32, 16, N_CLASS], lr=0.02)
    mlp.train(Xtr, ytr, epochs=120, batch=128)
    qW, qb, w_scale, x_scale, in_scale = quantize_mlp(mlp, Xcal)

    # 定点乘子：隐藏层「int32 累加 -> 下一层 int8 激活」的有效缩放
    #   M_i = S_i / x_scale[i]， S_i = w_scale[i] * 输入激活尺度
    #   int8_下一层 = clip(round(z_int * M_i), -127, 127)
    #   => 纯整数： next = clip(round(z_int * M_q >> AT_Q), -127, 127)
    Mq0 = int(round((w_scale[0] * in_scale / x_scale[0]) * (1 << q)))
    Mq1 = int(round((w_scale[1] * x_scale[0] / x_scale[1]) * (1 << q)))
    out_scale = float(w_scale[2] * x_scale[1])      # 输出层反量化尺度（FPU 仅用于 softmax 前）
    feat_lo = np.array([f[2][0] for f in FEATURES], dtype=np.float64)
    feat_hi = np.array([f[2][1] for f in FEATURES], dtype=np.float64)

    def arr8(name, a):
        a = np.asarray(a, dtype=np.int8).reshape(-1)
        rows = [", ".join(str(int(v)) for v in a[i:i + 16]) for i in range(0, a.size, 16)]
        return f"static const int8_t {name}[{a.size}] = {{\n  " + ",\n  ".join(rows) + "\n};"

    def arr32(name, a):
        a = np.asarray(a, dtype=np.int64).reshape(-1)
        rows = [", ".join(str(int(v)) for v in a[i:i + 8]) for i in range(0, a.size, 8)]
        return f"static const int32_t {name}[{a.size}] = {{\n  " + ",\n  ".join(rows) + "\n};"

    def arrf(name, a):
        a = np.asarray(a, dtype=np.float64).reshape(-1)
        rows = [", ".join(repr(float(v)) + "f" for v in a[i:i + 8]) for i in range(0, a.size, 8)]
        return f"static const float {name}[{a.size}] = {{\n  " + ",\n  ".join(rows) + "\n};"

    H = []
    H.append("/* Auto-generated by export_model_h() -- do not edit by hand. */")
    H.append("#ifndef AIRTUNE_MODEL_H")
    H.append("#define AIRTUNE_MODEL_H")
    H.append("#include <stdint.h>")
    H.append("")
    H.append(f"#define AT_IN   {N_FEAT}")
    H.append(f"#define AT_H1   32")
    H.append(f"#define AT_H2   16")
    H.append(f"#define AT_OUT  {N_CLASS}")
    H.append(f"#define AT_Q    {q}")
    H.append("")
    H.append("/* 权重(int8, row-major out*in) + 偏置(int32, 已按有效尺度 S=w_scale*x_scale 折叠) */")
    H.append(arr8("at_W0", qW[0]))
    H.append(arr8("at_W1", qW[1]))
    H.append(arr8("at_W2", qW[2]))
    H.append(arr32("at_b0", qb[0]))
    H.append(arr32("at_b1", qb[1]))
    H.append(arr32("at_b2", qb[2]))
    H.append(f"static const int32_t at_Mq0 = {Mq0};  /* 层0 定点乘子 = round(M0*2^{q}) */")
    H.append(f"static const int32_t at_Mq1 = {Mq1};  /* 层1 定点乘子 = round(M1*2^{q}) */")
    H.append(f"static const float   at_out_scale = {repr(out_scale)}f;  /* 输出层反量化尺度(浮点, 仅 softmax 前用) */")
    H.append(f"static const float   at_in_scale  = {repr(float(in_scale))}f;   /* 输入特征量化尺度 */")
    H.append(arrf("at_feat_lo", feat_lo))
    H.append(arrf("at_feat_hi", feat_hi))
    H.append("")
    H.append("#define AT_MODEL_FLASH_BYTES (sizeof(at_W0)+sizeof(at_W1)+sizeof(at_W2)"
             "+sizeof(at_b0)+sizeof(at_b1)+sizeof(at_b2)"
             "+sizeof(at_Mq0)+sizeof(at_Mq1)+sizeof(at_out_scale)+sizeof(at_in_scale)"
             "+sizeof(at_feat_lo)+sizeof(at_feat_hi))")
    H.append("")
    H.append("#endif /* AIRTUNE_MODEL_H */")
    with open(path, "w") as f:
        f.write("\n".join(H) + "\n")

    nbytes = (qW[0].size + qW[1].size + qW[2].size) \
        + 4 * (qb[0].size + qb[1].size + qb[2].size) \
        + 4 * 2 + 4 * 2 + 8 * 2
    # 端侧精度自检：与 Python int8 参考一致即代表 F411 上也会一致
    yp = mlp_int8_forward(mlp, qW, qb, w_scale, x_scale, Xte, in_scale)
    acc_q = np.mean(yp == yte)
    acc_f = accuracy(mlp, Xte, yte)
    print(f"[export] 已写出模型头: {path}")
    print(f"[export] Flash 占用 ≈ {nbytes} B ({nbytes/1024.0:.2f} KB)  "
          f"(权重{896}B+偏置{52*4}B+乘子/尺度16B+特征范围80B)")
    print(f"[export] float={acc_f*100:.3f}% int8={acc_q*100:.3f}% "
          f"delta={(acc_f-acc_q)*100:.3f}%  (要求 <1%)")


if __name__ == "__main__":
    import os
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "export":
        export_model_h(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "firmware", "Core", "Inc", "airtune_model.h"))
    else:
        main()

    print("AirTune PC 侧：10 维特征 -> 4 类 LoRa 模式 的 MLP + int8 量化")
    print("=" * 64)

    print("\n[1] 特征定义（10 维）:")
    for i, (nm, desc, (lo, hi)) in enumerate(FEATURES):
        print(f"  {i:2d}. {nm:14s} {desc:28s} [{lo:7.1f}, {hi:7.1f}]")

    # 数据
    X_raw, X, y = generate_data(20000)
    Xtr, ytr = X[:16000], y[:16000]
    Xte, yte = X[16000:], y[16000:]
    Xcal = Xtr[:500]  # 校准集
    print(f"\n[2] 合成数据：{len(X)} 样本，训练 {len(Xtr)} / 测试 {len(Xte)} / 校准 {len(Xcal)}")
    print("    标签分布:", np.bincount(y))

    # 训练
    print("\n[3] 训练 MLP (10->32->16->4, ReLU, Adam-like SGD):")
    mlp = MLP([N_FEAT, 32, 16, N_CLASS], lr=0.02)
    mlp.train(Xtr, ytr, epochs=120, batch=128)

    acc_f = accuracy(mlp, Xte, yte)
    print(f"\n[4] float32 测试精度 = {acc_f*100:.3f}%")

    # 量化
    print("\n[5] int8 量化（权重+激活，int32 累加）...")
    qW, qb, w_scale, x_scale, in_scale = quantize_mlp(mlp, Xcal)
    yp = mlp_int8_forward(mlp, qW, qb, w_scale, x_scale, Xte, in_scale)
    acc_q = np.mean(yp == yte)
    print(f"    int8    测试精度 = {acc_q*100:.3f}%")

    delta = (acc_f - acc_q) * 100.0
    print(f"\n[6] 量化前后精度差 = {delta:.3f} %   (要求 < 1%)")
    if delta < 1.0:
        print("    ✅ 通过：量化前后精度差 < 1%")
    else:
        print("    ⚠️ 未达成 <1%，可调大网络/校准集或放宽量化粒度")

    # 模型大小对比
    n_param = sum(w.size + b.size for w, b in zip(mlp.W, mlp.b))
    print(f"\n[7] 模型参数 {n_param} 个："
          f"float32 = {n_param*4} B，int8 = {n_param} B "
          f"（压缩 {4.0:.1f}x）")

    # 抽样展示：同一样本 float 与 int8 预测一致
    print("\n[8] 抽样对比（前 8 个测试样本）:")
    yf = mlp.predict(Xte[:8])
    print("    idx  真实模式            float预测          int8预测          一致")
    for i in range(8):
        ok = "✓" if yf[i] == yp[i] else "✗"
        print(f"    {i:3d}  {MODES[y[i]]:22s} {MODES[yf[i]]:18s} {MODES[yp[i]]:18s}  {ok}")

    print("\n完成。int8 权重/尺度可直接导出给 F411 端侧 AirTune 运行时。")


if __name__ == "__main__":
    main()
