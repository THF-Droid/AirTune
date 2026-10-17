"""生成随机归一化测试向量 + Python int8 参考标签，供 C 定点推理交叉验证。
导出 test_vectors.h：AT_TV_N 个样本 + 其 int8 参考类别 at_tv_Y[]。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from AirTune import (generate_data, MLP, quantize_mlp, mlp_int8_forward,
                     N_FEAT, N_CLASS)

# 复现与导出一致的训练（同 RNG 顺序 -> 同权重）
X_raw, X, y = generate_data(20000)
Xtr, ytr = X[:16000], y[:16000]
Xcal = Xtr[:500]
mlp = MLP([N_FEAT, 32, 16, N_CLASS], lr=0.02)
mlp.train(Xtr, ytr, epochs=120, batch=128)
qW, qb, w_scale, x_scale, in_scale = quantize_mlp(mlp, Xcal)

# 独立随机流造测试向量（不与训练共享 RNG）
rng2 = np.random.default_rng(777)
N = 4000
Xt = rng2.uniform(-1.0, 1.0, (N, N_FEAT)).astype(np.float64)
yt = mlp_int8_forward(mlp, qW, qb, w_scale, x_scale, Xt, in_scale)

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_vectors.h")
with open(OUT, "w") as f:
    f.write("/* Auto-generated test vectors for C fixed-point cross-check (Python int8 ref). */\n")
    f.write(f"#define AT_TV_N {N}\n")
    f.write("static const float at_tv_X[AT_TV_N][AT_IN] = {\n")
    for i in range(N):
        f.write("  {" + ", ".join(repr(float(v)) + "f" for v in Xt[i]) + "},\n")
    f.write("};\n")
    f.write("static const int at_tv_Y[AT_TV_N] = {\n  ")
    f.write(", ".join(str(int(v)) for v in yt))
    f.write("\n};\n")
print(f"[gen_testvec] 写出 {N} 个测试向量 -> {OUT}")
