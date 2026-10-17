#ifndef AIRTUNE_INFER_H
#define AIRTUNE_INFER_H

#include <stdint.h>
#include "airtune_model.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 归一化特征(已映射到 [-1,1]) 推理；返回 0..3 类，cls_out 可空 */
int airtune_predict_norm(const float x_norm[AT_IN], int *cls_out);

/* 原始物理量纲特征推理：内部先做归一化再走定点 MLP */
int airtune_predict_raw(const float x_raw[AT_IN], int *cls_out);

/* 推理进入时的栈指针（最低地址 = 最深）。bench/ram 用它测栈深。
   非 ARM 平台编译时恒为 0（不影响 argmax 正确性）。 */
extern uint32_t g_at_sp_entry;

#ifdef __cplusplus
}
#endif

#endif /* AIRTUNE_INFER_H */
