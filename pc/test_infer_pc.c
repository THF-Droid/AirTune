/* 纯 C 定点 MLP 与 Python int8 参考的交叉验证（在 PC 用 gcc 编译运行）。
 * 同一份 airtune_infer.c 整数算术，验证 F411 上推理精度与 PC 验证一致（<1% 差）。
 * 编译: gcc test_infer_pc.c -I D:/AirTune/Core/Inc -I D:/AirTune/Core/Src -I . -o test_infer_pc
 */
#include <stdio.h>
#include "airtune_infer.c"   /* 直接包含实现，单文件编译 */
#include "test_vectors.h"

int main(void)
{
    int ok = 0;
    for (int i = 0; i < AT_TV_N; i++)
    {
        int cls;
        airtune_predict_norm(at_tv_X[i], &cls);
        if (cls == at_tv_Y[i]) ok++;
    }
    double acc = 100.0 * ok / AT_TV_N;
    printf("C fixed-point vs Python int8 ref: %d/%d = %.3f%%\n", ok, AT_TV_N, acc);
    printf("与 int8 参考的不一致率 = %.3f%%  (应 ≈ 0，说明 F411 定点与 PC int8 逐位一致)\n",
           100.0 - acc);
    return 0;
}
