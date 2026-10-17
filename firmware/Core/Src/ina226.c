/* ===================== INA226 驱动（begin） =====================
 * INA226：高端侧电流/电压/功率监测芯片，I2C 接口
 *  - 挂 I2C1（PB8=SCL, PB9=SDA, 100kHz）
 *  - 地址自动探测：默认 0x40，找不到依次试 0x41/0x44/0x45（模块 AD0/A1 跳帽变体），
 *    只用芯片 ID（man=0x5449 / die=0x2260）校验，不会误写别的器件
 *  - 串口命令：power 单次读数；pwrlog <n> <ms> 流式 CSV；i2cscan 扫总线
 *  - 采样电阻值由 g_params.shunt_mohm 设定（丝印 R010=10mΩ / R100=100mΩ）
 *  - LSB：分流 2.5uV(有符号) / 总线 1.25mV / 电流 Current_LSB=Imax/32768≈30.5uA
 *    CAL = 0.00512 / (Current_LSB * Rshunt)
 * ================================================================ */
#include "main.h"
#include "ina226.h"

extern I2C_HandleTypeDef hi2c1;

#define INA226_TMO_MS   100
#define INA226_IMAX_A   1.0f   /* 期望最大电流 -> Current_LSB ≈ 30.5uA。
                              注意：CAL 寄存器只有 15 位(D15 不可写，上限 0x7FFF)，
                              Imax 取小了 CAL 会溢出被静默截断 -> 电流读数缩小。
                              10mΩ 时 CAL≈0x4181，100mΩ 时≈0x068E，都在限内。 */

/* 常见 I2C 地址变体（A0/A1 跳帽）：0x40=都悬空/接地，0x41/0x44/0x45 见模块丝印 */
static const uint8_t k_cand_addr[4] = { 0x40, 0x41, 0x44, 0x45 };
static uint8_t s_addr = INA226_I2C_ADDR;

static uint16_t s_shunt_mohm = 10;   /* 采样电阻毫欧值，默认 10mΩ */

static int ina_wr16(uint8_t reg, uint16_t val)
{
  uint8_t b[3];
  b[0] = reg;
  b[1] = (uint8_t)(val >> 8);
  b[2] = (uint8_t)(val & 0xFFu);
  return (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(s_addr << 1), b, 3,
                                  INA226_TMO_MS) == HAL_OK) ? 0 : -1;
}

static int ina_rd16(uint8_t reg, uint16_t *val)
{
  uint8_t b[2];
  if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(s_addr << 1), &reg, 1,
                              INA226_TMO_MS) != HAL_OK) return -1;
  if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(s_addr << 1), b, 2,
                             INA226_TMO_MS) != HAL_OK) return -1;
  *val = (uint16_t)(((uint16_t)b[0] << 8) | (uint16_t)b[1]);
  return 0;
}

/* 写校准寄存器：CAL = 0.00512 / (Current_LSB * Rshunt)，限幅 1..65535 */
static void ina_write_calib(void)
{
  float r   = (float)s_shunt_mohm / 1000.0f;
  float cal = 0.00512f / ((INA226_IMAX_A / 32768.0f) * r);
  if (cal < 1.0f)     cal = 1.0f;
  if (cal > 32767.0f) cal = 32767.0f;   /* CAL 15 位上限，超限芯片会丢高位 */
  (void)ina_wr16(INA226_REG_CALIB, (uint16_t)cal);
}

void ina226_set_shunt_mohm(uint32_t milliohm)
{
  if (milliohm < 1)    milliohm = 1;
  if (milliohm > 1000) milliohm = 1000;
  s_shunt_mohm = (uint16_t)milliohm;
  ina_write_calib();
}

/* 读芯片 ID 验证：man=0x5449(TI) die=0x2260 才算通 */
int ina226_probe(uint16_t *man, uint16_t *die)
{
  uint16_t m = 0, d = 0;
  if (ina_rd16(INA226_REG_MANID, &m) != 0) return -1;
  if (ina_rd16(INA226_REG_DIEID, &d) != 0) return -1;
  if (man) *man = m;
  if (die) *die = d;
  return (m == INA226_MAN_ID && d == INA226_DIE_ID) ? 0 : 1;
}

uint8_t ina226_addr(void) { return s_addr; }

/* 初始化：按 ID 校验自动找地址；找到才写 CONFIG+CALIB（不会误写别的器件） */
int ina226_init(void)
{
  for (unsigned i = 0; i < sizeof(k_cand_addr); i++)
  {
    s_addr = k_cand_addr[i];
    if (ina226_probe(NULL, NULL) == 0)
    {
      if (ina_wr16(INA226_REG_CONFIG, (uint16_t)INA226_CFG_CONT) != 0) return -1;
      ina_write_calib();
      return 0;
    }
  }
  s_addr = INA226_I2C_ADDR;   /* 没找到，回到默认地址，供 power/i2cscan 诊断 */
  return -1;
}

int ina226_read_all(ina226_t *o)
{
  uint16_t shunt = 0, bus = 0, cur = 0;
  for (int attempt = 0; attempt < 2; attempt++)
  {
    if (ina_rd16(INA226_REG_SHUNT, &shunt) == 0 &&
        ina_rd16(INA226_REG_BUS, &bus) == 0 &&
        ina_rd16(INA226_REG_CURRENT, &cur) == 0 &&
        ina_rd16(INA226_REG_CONFIG, &o->config) == 0 &&
        ina_rd16(INA226_REG_CALIB, &o->calib) == 0)
    {
      int32_t  s16 = (int16_t)shunt;   /* 2.5uV/LSB，有符号 */
      uint32_t b16 = bus;              /* 1.25mV/LSB，无符号 */
      int32_t  c16 = (int16_t)cur;     /* Current_LSB，有符号 */

      o->vshunt_uV  = s16 * 5 / 2;
      o->vbus_mV    = (int32_t)((b16 * 5u) >> 2);
      o->current_uA = (int32_t)(((int64_t)c16 * 1000000) / 32768); /* Current_LSB=Imax/32768≈30.5uA */
      o->current_mA = (o->current_uA + 500) / 1000;
      o->power_mW   = (int32_t)(((int64_t)o->vbus_mV * o->current_uA + 500000) / 1000000);
      return 0;
    }
    /* 失败一次就做总线恢复：清掉残留的 AF/BUSY 错误状态，重试一次
       （直连线半接触时地址轮询能过、连续读多个寄存器会踩到接触坏的瞬间） */
    HAL_I2C_DeInit(&hi2c1);
    HAL_I2C_Init(&hi2c1);
  }
  return -1;
}
/* ===================== INA226 驱动（end） ===================== */
