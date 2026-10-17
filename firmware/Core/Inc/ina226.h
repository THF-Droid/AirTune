#ifndef INA226_H
#define INA226_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* INA226 I2C 地址：A0=A1=GND -> 0x40（部分模块跳线可改 0x41/0x44/0x45） */
#define INA226_I2C_ADDR   0x40

/* 寄存器（16bit 大端） */
#define INA226_REG_CONFIG   0x00
#define INA226_REG_SHUNT    0x01
#define INA226_REG_BUS      0x02
#define INA226_REG_POWER    0x03
#define INA226_REG_CURRENT  0x04
#define INA226_REG_CALIB    0x05
#define INA226_REG_MANID    0xFE
#define INA226_REG_DIEID    0xFF

/* CONFIG = AVG(4) | VBUSCT(588us) | VSHCT(588us) | MODE(111 连续分流+总线)
   AVG@D11:9=001, VBUSCT@D8:6=011, VSHCT@D5:3=011, MODE@D2:0=111 -> 0x02DF */
#define INA226_CFG_CONT     0x02DFu

#define INA226_MAN_ID       0x5449u   /* TI */
#define INA226_DIE_ID       0x2260u

typedef struct {
  uint16_t config;      /* CONFIG 寄存器回读值 */
  uint16_t calib;       /* CALIB 寄存器回读值 */
  int32_t  vbus_mV;     /* 总线电压（IN- 对地）mV */
  int32_t  vshunt_uV;   /* 采样电阻两端电压 uV（有符号） */
  int32_t  current_uA;  /* 电流 uA（Current_LSB=0.5A/32768≈15.26uA） */
  int32_t  current_mA;  /* 电流 mA（四舍五入） */
  int32_t  power_mW;    /* 功率 mW = Vbus * I */
} ina226_t;

void     ina226_set_shunt_mohm(uint32_t milliohm);
int      ina226_probe(uint16_t *man, uint16_t *die);
uint8_t  ina226_addr(void);
int      ina226_init(void);
int      ina226_read_all(ina226_t *out);

#endif /* INA226_H */
