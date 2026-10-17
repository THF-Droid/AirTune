/* ===================== SX1278 (Ra-02) 驱动（begin） =====================
 * SPI1 (PA5=SCK PA6=MISO PA7=MOSI, 6.25MHz Mode0)，NSS=PB6 软件片选，RST=PB5。
 * 轮询 IrqFlags(0x12) 判 TxDone/RxDone，DIO0 不用接。
 * 寄存器值/位定义均按 Semtech SX1276/77/78 数据手册。
 * ========================================================================= */
#include "main.h"
#include "lora_sx1278.h"
#include <stdio.h>   /* snprintf/printf: lora_dump_regs */

extern SPI_HandleTypeDef hspi1;

#define NSS_LOW()   HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET)
#define NSS_HIGH()  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET)
#define RST_LOW()   HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET)
#define RST_HIGH()  HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET)

#define SPI_TMO     100

/* ---- 寄存器 ---- */
#define REG_FIFO            0x00
#define REG_OPMODE          0x01
#define REG_FRF_MSB         0x06
#define REG_FRF_MID         0x07
#define REG_FRF_LSB         0x08
#define REG_PA_CONFIG       0x09
#define REG_OCP             0x0B
#define REG_LNA             0x0C
#define REG_FIFO_ADDR_PTR   0x0D
#define REG_FIFO_TX_BASE    0x0E
#define REG_FIFO_RX_BASE    0x0F
#define REG_FIFO_RX_CUR     0x10
#define REG_IRQ_FLAGS_MASK  0x11
#define REG_IRQ_FLAGS       0x12
#define REG_RX_NB_BYTES     0x13
#define REG_PKT_SNR         0x19
#define REG_RSSI            0x1B
#define REG_MODEM_CONFIG1   0x1D
#define REG_MODEM_CONFIG2   0x1E
#define REG_SYMB_TIMEOUT    0x1F
#define REG_PREAMBLE_MSB    0x20
#define REG_PREAMBLE_LSB    0x21
#define REG_PAYLOAD_LEN     0x22
#define REG_MAX_PAYLOAD     0x23
#define REG_MODEM_CONFIG3   0x26
#define REG_DETECT_OPT      0x31
#define REG_DETECT_THR      0x37
#define REG_SYNC_WORD       0x39
#define REG_DIO_MAPPING1    0x40
#define REG_VERSION         0x42
#define REG_PA_DAC          0x4D

/* ---- IrqFlags 位 ---- */
#define IRQ_RX_TIMEOUT      0x80
#define IRQ_RX_DONE         0x40
#define IRQ_PAYLOAD_CRC_ERR 0x20
#define IRQ_TX_DONE         0x08

/* ---- 模式（Or 0x80 = LongRangeMode）---- */
#define MODE_SLEEP          0x00
#define MODE_STANDBY        0x01
#define MODE_TX             0x03
#define MODE_RX_CONT        0x05

#define LORA_MAX_PAYLOAD    64
#define LORA_DEFAULT_FREQ   433.0f
#define FXOSC_HZ            32000000ull

static lora_info_t s_info;
static int s_up = 0;

/* 清掉上一笔事务的残留：HAL_SPI_Transmit 是纯发送，不回读 DR，
 * 于是 RXNE 里留着地址阶段的脏字节；下一笔读事务会整体错位一个字节
 * （rx[0]=残留, rx[1]=脏字节, 真数据被丢下）-> 读到'稳定但错误'的值。
 * OVR 不清也会一直挂着。每笔事务前清一次。 */
static void spi_flush(void)
{
  if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE) != RESET)
    (void)hspi1.Instance->DR;
  __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
}

/* ---- SPI 寄存器访问 ---- */
static uint8_t rd(uint8_t a)
{
  /* 必须单次 TransmitReceive：分两次调（先 Transmit 再 Receive）会把
     「地址阶段」收到的脏字节当数据返回（RXNE 已被第一次置位）。 */
  uint8_t tx[2] = { (uint8_t)(a & 0x7Fu), 0x00u };
  uint8_t rx[2] = { 0x00u, 0x00u };
  spi_flush();
  NSS_LOW();
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, SPI_TMO);
  NSS_HIGH();
  return rx[1];
}

static void wr(uint8_t a, uint8_t v)
{
  uint8_t b[2] = { (uint8_t)(a | 0x80u), v };
  spi_flush();
  NSS_LOW();
  HAL_SPI_Transmit(&hspi1, b, 2, SPI_TMO);
  NSS_HIGH();
}

static void wr_fifo(const uint8_t *d, int n)
{
  uint8_t b[LORA_MAX_PAYLOAD + 1];
  b[0] = REG_FIFO | 0x80u;
  for (int i = 0; i < n; i++) b[i + 1] = d[i];
  spi_flush();
  NSS_LOW();
  HAL_SPI_Transmit(&hspi1, b, (uint16_t)(n + 1), SPI_TMO);
  NSS_HIGH();
}

static void rd_fifo(uint8_t *d, int n)
{
  /* 同上：地址 + n 个填充字节一次发完，数据从 rx[1..n] 取 */
  uint8_t tx[LORA_MAX_PAYLOAD + 1];
  uint8_t rx[LORA_MAX_PAYLOAD + 1];
  tx[0] = REG_FIFO & 0x7Fu;
  for (int i = 1; i <= n; i++) tx[i] = 0x00u;
  spi_flush();
  NSS_LOW();
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)(n + 1), SPI_TMO);
  NSS_HIGH();
  for (int i = 0; i < n; i++) d[i] = rx[i + 1];
}

static void set_mode(uint8_t m) { wr(REG_OPMODE, (uint8_t)(0x80u | (m & 0x07u))); }

/* ---- 参数设置 ---- */
void lora_set_freq(float mhz)
{
  if (mhz < 137.0f) mhz = 137.0f;
  if (mhz > 1020.0f) mhz = 1020.0f;
  uint64_t frf = ((uint64_t)(mhz * 1000000.0f + 0.5f) << 19) / FXOSC_HZ;
  wr(REG_FRF_MSB, (uint8_t)(frf >> 16));
  wr(REG_FRF_MID, (uint8_t)(frf >> 8));
  wr(REG_FRF_LSB, (uint8_t)(frf));
  s_info.freq_mhz = mhz;
}

void lora_set_sf(int sf)
{
  if (sf < 6) sf = 6;
  if (sf > 12) sf = 12;
  uint8_t m = rd(REG_MODEM_CONFIG2);
  wr(REG_MODEM_CONFIG2, (uint8_t)((m & 0x0Fu) | (uint8_t)(sf << 4)));
  /* SF11/12 @125k 必须开 LowDataRateOptimize；SF6 用另一组检测参数 */
  wr(REG_MODEM_CONFIG3, (sf >= 11) ? 0x0Cu : 0x04u);
  wr(REG_DETECT_OPT, (sf == 6) ? 0xC5u : 0xC3u);
  s_info.sf = sf;
}

void lora_set_bw(int khz)
{
  uint8_t v;
  if (khz == 500)      v = 9;
  else if (khz == 250) v = 8;
  else               { v = 7; khz = 125; }
  uint8_t m = rd(REG_MODEM_CONFIG1);
  wr(REG_MODEM_CONFIG1, (uint8_t)((m & 0x0Fu) | (uint8_t)(v << 4)));
  s_info.bw_khz = khz;
}

void lora_set_power(int dbm)
{
  if (dbm >= 20) {                    /* +20dBm：PaDac boost 档 */
    wr(REG_PA_DAC, 0x87u);
    wr(REG_PA_CONFIG, 0x8Fu);
    s_info.tx_dbm = 20;
  } else {
    if (dbm < 2) dbm = 2;
    if (dbm > 17) dbm = 17;
    wr(REG_PA_DAC, 0x84u);            /* 默认 +17dBm 档 */
    wr(REG_PA_CONFIG, (uint8_t)(0x80u | (uint8_t)(dbm - 2)));
    s_info.tx_dbm = dbm;
  }
  wr(REG_OCP, 0x20u);                 /* 过流保护放到 ~205mA，别在发射峰值误触发 */
}

void lora_apply_params(int sf, int bw_khz, int tx_dbm)
{
  if (!s_up) return;
  set_mode(MODE_STANDBY);
  lora_set_sf(sf);
  lora_set_bw(bw_khz);
  lora_set_power(tx_dbm);
}

int lora_is_up(void) { return s_up; }

/* 裸读一个寄存器（诊断用） */
uint8_t lora_read_reg(uint8_t addr) { return rd(addr); }

/* 寄存器堆转储（诊断用）：8 个一行，地址=值 对 */
void lora_dump_regs(uint8_t start, uint8_t end)
{
  char line[80];
  for (uint8_t base = start; base <= end; base += 8) {
    int n = snprintf(line, sizeof(line), "[LORA] %02X:", base);
    for (int i = 0; i < 8 && (int)(base + i) <= (int)end; i++)
      n += snprintf(line + n, (size_t)(sizeof(line) - (size_t)n), " %02X=%02X",
                    (int)(base + i), rd((uint8_t)(base + i)));
    printf("%s\r\n", line);
  }
}
const lora_info_t *lora_info(void) { return &s_info; }

/* ---- 初始化 ---- */
int lora_init(void)
{
  s_up = 0;
  /* 硬复位：RST 拉低 2ms 再拉高，等 10ms 稳定 */
  RST_LOW();  HAL_Delay(2);
  RST_HIGH(); HAL_Delay(10);

  wr(REG_OPMODE, 0x80u); HAL_Delay(1);   /* sleep + LoRa 长程模式 */
  wr(REG_OPMODE, 0x81u); HAL_Delay(1);   /* standby + LoRa */

  s_info.version = rd(REG_VERSION);
  if (s_info.version != 0x12u) return -1;   /* SX1278 应为 0x12 */

  lora_set_freq(LORA_DEFAULT_FREQ);
  lora_set_sf(7);
  lora_set_bw(125);
  lora_set_power(17);
  wr(REG_SYNC_WORD, 0x12u);                  /* 私有网络同步字（收发两端一致即可） */
  wr(REG_PREAMBLE_MSB, 0x00u);
  wr(REG_PREAMBLE_LSB, 0x08u);               /* 8 字节前导码 */
  wr(REG_MAX_PAYLOAD, LORA_MAX_PAYLOAD);
  wr(REG_SYMB_TIMEOUT, 0x40u);
  wr(REG_DETECT_THR, 0x0Au);
  wr(REG_LNA, 0x23u);                        /* LNA 增益 boost */
  wr(REG_FIFO_TX_BASE, 0x80u);
  wr(REG_FIFO_RX_BASE, 0x00u);
  wr(REG_IRQ_FLAGS_MASK, 0x00u);
  wr(REG_IRQ_FLAGS, 0xFFu);
  s_info.last_rssi = 0;
  s_info.last_snr = 0.0f;
  set_mode(MODE_STANDBY);
  s_up = 1;
  return 0x12;
}

/* ---- 发送：阻塞至 TxDone（SF12@125k 最长约 1~2s，超时 3s）---- */
int lora_send(const uint8_t *data, int len)
{
  if (!s_up) return -1;
  if (len < 1) len = 1;
  if (len > LORA_MAX_PAYLOAD) len = LORA_MAX_PAYLOAD;

  set_mode(MODE_STANDBY);
  wr(REG_FIFO_TX_BASE, 0x80u);
  wr(REG_FIFO_ADDR_PTR, 0x80u);
  wr_fifo(data, len);
  wr(REG_PAYLOAD_LEN, (uint8_t)len);
  wr(REG_IRQ_FLAGS, 0xFFu);
  set_mode(MODE_TX);

  uint32_t t0 = HAL_GetTick();
  while (!(rd(REG_IRQ_FLAGS) & IRQ_TX_DONE)) {
    if (HAL_GetTick() - t0 > 3000u) { set_mode(MODE_STANDBY); return -1; }
    HAL_Delay(1);
  }
  wr(REG_IRQ_FLAGS, 0xFFu);
  set_mode(MODE_STANDBY);
  return 0;
}

/* ---- 接收：进连续接收，轮询 RxDone；返回字节数，-1 超时，-2 CRC 错 ---- */
int lora_recv(uint8_t *buf, int buflen, int timeout_ms, int *rssi, float *snr)
{
  if (!s_up) return -1;
  if (buflen > LORA_MAX_PAYLOAD) buflen = LORA_MAX_PAYLOAD;
  if (timeout_ms < 10) timeout_ms = 10;

  wr(REG_FIFO_RX_BASE, 0x00u);
  wr(REG_FIFO_ADDR_PTR, 0x00u);
  wr(REG_IRQ_FLAGS, 0xFFu);
  set_mode(MODE_RX_CONT);

  uint32_t t0 = HAL_GetTick();
  uint8_t fl = 0;
  for (;;) {
    fl = rd(REG_IRQ_FLAGS);
    if (fl & IRQ_RX_DONE) break;
    if (HAL_GetTick() - t0 > (uint32_t)timeout_ms) { set_mode(MODE_STANDBY); return -1; }
    HAL_Delay(1);
  }
  if (fl & IRQ_PAYLOAD_CRC_ERR) {
    wr(REG_IRQ_FLAGS, 0xFFu);
    set_mode(MODE_STANDBY);
    return -2;
  }

  uint8_t cur = rd(REG_FIFO_RX_CUR);
  uint8_t n = rd(REG_RX_NB_BYTES);
  if (n > buflen) n = (uint8_t)buflen;
  wr(REG_FIFO_ADDR_PTR, cur);
  rd_fifo(buf, n);

  s_info.last_rssi = -164 + (int)rd(REG_RSSI);        /* 433MHz 低频段 */
  s_info.last_snr = (float)((int8_t)rd(REG_PKT_SNR)) / 4.0f;
  if (rssi) *rssi = s_info.last_rssi;
  if (snr)  *snr  = s_info.last_snr;

  wr(REG_IRQ_FLAGS, 0xFFu);
  set_mode(MODE_STANDBY);
  return n;
}
/* ===================== SX1278 驱动（end） ===================== */
