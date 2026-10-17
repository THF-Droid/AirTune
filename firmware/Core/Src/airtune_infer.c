/* AirTune 端侧推理：纯 C 定点 MLP（int8 × int8 -> int32 累加 -> 定点乘子 -> int8）
 * 隐藏层全程定点整数运算；仅输出层在 softmax 前用一次 float 反量化（F411 有 FPU）。
 * 权重/偏置/尺度全部 const -> 烧进 Flash，运行时 0 RAM 占用。
 */
#include "airtune_infer.h"

uint32_t g_at_sp_entry = 0xFFFFFFFFU;

/* 取当前栈指针(MSP)。ARM(CMSIS) 用 __get_MSP()；非 ARM 平台返回 0，不影响 argmax。 */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION) || (defined(__GNUC__) && (defined(__arm__) || defined(__thumb__)))
  #include "stm32f4xx.h"   /* 提供 CMSIS __get_MSP() */
  #define AT_HAVE_MSP 1
#endif
static inline uint32_t at_get_msp(void)
{
#if defined(AT_HAVE_MSP)
    return __get_MSP();
#else
    return 0U;
#endif
}

#define AT_CLIP(v, lo, hi)  ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))

/* 归一化特征 -> int8（对称量化，输入尺度 at_in_scale） */
static int8_t at_q8(float v, float scale)
{
    float r = v / scale;
    long i = (long)(r < 0.0f ? r - 0.5f : r + 0.5f);
    if (i > 127) i = 127;
    if (i < -127) i = -127;
    return (int8_t)i;
}

int airtune_predict_norm(const float x_norm[AT_IN], int *cls_out)
{
    g_at_sp_entry = at_get_msp();   /* 记录进入时最深栈位置（供 bench/ram 测栈深） */

    /* 输入层量化 */
    int8_t x[AT_IN];
    for (int j = 0; j < AT_IN; j++)
        x[j] = at_q8(x_norm[j], at_in_scale);

    /* 隐藏层 0: 32 神经元 */
    int8_t h1[AT_H1];
    const int8_t *W = at_W0;
    for (int o = 0; o < AT_H1; o++)
    {
        int32_t acc = at_b0[o];
        for (int j = 0; j < AT_IN; j++)
            acc += (int32_t)W[j] * (int32_t)x[j];
        /* 定点：先 ReLU（量化前的非线性），再裁到 [0,127]
           注意：必须 ReLU 之后才量化，否则负激活会原样传进下一层 */
        int32_t q = (int32_t)(((int64_t)acc * (int64_t)at_Mq0) >> AT_Q);
        if (q < 0) q = 0;
        h1[o] = (int8_t)AT_CLIP(q, 0, 127);
        W += AT_IN;
    }

    /* 隐藏层 1: 16 神经元 */
    int8_t h2[AT_H2];
    W = at_W1;
    for (int o = 0; o < AT_H2; o++)
    {
        int32_t acc = at_b1[o];
        for (int j = 0; j < AT_H1; j++)
            acc += (int32_t)W[j] * (int32_t)h1[j];
        int32_t q = (int32_t)(((int64_t)acc * (int64_t)at_Mq1) >> AT_Q);
        if (q < 0) q = 0;                       /* ReLU */
        h2[o] = (int8_t)AT_CLIP(q, 0, 127);
        W += AT_H1;
    }

    /* 输出层: 4 类。int32 累加 -> float 反量化 -> 直接取 argmax（无需 exp） */
    float best = -1e30f;
    int bestc = 0;
    W = at_W2;
    for (int o = 0; o < AT_OUT; o++)
    {
        int32_t acc = at_b2[o];
        for (int j = 0; j < AT_H2; j++)
            acc += (int32_t)W[j] * (int32_t)h2[j];
        float zo = (float)acc * at_out_scale;
        if (zo > best) { best = zo; bestc = o; }
        W += AT_H2;
    }

    if (cls_out) *cls_out = bestc;
    return bestc;
}

int airtune_predict_raw(const float x_raw[AT_IN], int *cls_out)
{
    float x_norm[AT_IN];
    for (int j = 0; j < AT_IN; j++)
    {
        float n = (x_raw[j] - at_feat_lo[j]) / (at_feat_hi[j] - at_feat_lo[j]) * 2.0f - 1.0f;
        x_norm[j] = AT_CLIP(n, -1.0f, 1.0f);
    }
    return airtune_predict_norm(x_norm, cls_out);
}
