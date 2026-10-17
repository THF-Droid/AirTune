#ifndef LORA_SX1278_H
#define LORA_SX1278_H

#include <stdint.h>

/* SX1278 (Ra-02) 驱动 —— 挂在 SPI1（PA5/6/7），NSS=PB6 软件片选，RST=PB5
 * 用法：串口 lora init / status / send <text> / recv [ms] / freq <mhz> / apply
 * 轮询 IrqFlags 判收发完成，不依赖 DIO0 接线（DIO0 可悬空）。
 */

typedef struct {
  uint8_t  version;      /* RegVersion 回读，SX1278 = 0x12 */
  float    freq_mhz;     /* 载波频率 MHz（Ra-02 常见 433） */
  int      sf;           /* 扩频因子 7..12 */
  int      bw_khz;       /* 带宽 125/250/500 */
  int      tx_dbm;       /* 发射功率 dBm（2..20） */
  int      last_rssi;    /* 最近一包 RSSI dBm */
  float    last_snr;     /* 最近一包 SNR dB */
} lora_info_t;

int      lora_init(void);                              /* 复位+初始化，成功返回 0x12，失败 -1 */
int      lora_is_up(void);
void     lora_set_freq(float mhz);                     /* 137..1020 */
void     lora_set_sf(int sf);                          /* 7..12 */
void     lora_set_bw(int bw_khz);                      /* 125/250/500 */
void     lora_set_power(int dbm);                      /* 2..20 */
void     lora_apply_params(int sf, int bw_khz, int tx_dbm);  /* AirTune 联动：参数->电台 */
int      lora_send(const uint8_t *data, int len);      /* 阻塞发送，0=成功，-1=超时/未初始化 */
int      lora_recv(uint8_t *buf, int buflen, int timeout_ms,
                   int *rssi, float *snr);             /* 返回字节数；-1 超时，-2 CRC 错 */
const lora_info_t *lora_info(void);
uint8_t  lora_read_reg(uint8_t addr);                  /* 裸读寄存器（诊断） */
void     lora_dump_regs(uint8_t start, uint8_t end);   /* 寄存器堆转储（诊断） */

#endif /* LORA_SX1278_H */
