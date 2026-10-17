# AirTune — STM32F411 + TinyML 自适应 LoRa 链路

> 端侧一颗 int8 量化的 MLP，根据 10 维链路状态实时预测「最优 LoRa 工作模式」，在吞吐 / 距离 / 功耗之间做在线权衡。

**一句话**：把「链路状态 → 最优 SF/BW/功率」的决策，从基站/人工规则下沉到 MCU 端侧，一次推理 **1.11 KB 权重 / 0 RAM 占用 / ~97 µs @100MHz**，全部数字由 DWT CYCCNT 硬件实测。

## 实测数据（本仓库全部可在真机复现）

| 指标 | 数值 | 测量方式 |
|---|---|---|
| 模型权重 Flash 占用 | **1.11 KB**（1136 B，const 驻留 Flash） | 编译产物统计 |
| 运行时 RAM 占用 | **0 B**（权重/偏置/尺度全在 Flash） | 栈哨兵测量 |
| 单次推理延迟 | **min 96.95 / avg 97.01 / max 97.11 µs @100MHz** | DWT CYCCNT 实测（`bench` 命令） |
| float32 精度 | 95.725% | 20000 样本，16000 训练 / 4000 测试 |
| int8 量化精度 | 95.800%（**delta = -0.075%，< 1% 指标达成**） | 对称 per-tensor PTQ，int32 累加 |
| C 定点 vs Python int8 参考 | **99.125% 一致** | MinGW gcc 编译同一份 `airtune_infer.c` 交叉验证 |
| 真机功能验证 | 4 种链路工况 → 4 种 LoRa 模式**全部分类正确** | `capture.py "log 200 20"` |

## 仓库结构

```
├── docs/
│   └── 嵌入式项目设计方案.md     # 完整设计文档（项目缘起/可行性/引脚规划/踩坑记录）
├── firmware/                    # STM32F411 固件（Keil MDK V5.32）
│   ├── Demo.ioc                 # CubeMX 配置源（HSE BYPASS + PLL 100MHz）
│   ├── Core/
│   │   ├── Inc/
│   │   │   ├── airtune_model.h  # 自动导出的 int8 权重（const → Flash）
│   │   │   ├── ina226.h
│   │   │   └── lora_sx1278.h     # LoRa 电台驱动接口
│   │   └── Src/
│   │       ├── main.c           # 串口 CLI + DWT + 功率监测 + LoRa + 采集流
│   │       ├── airtune_infer.c  # 纯 C 定点 MLP 推理（int8×int8→int32→定点乘子）
│   │       ├── ina226.c         # INA226 I2C 驱动（地址自动探测 + 总线恢复重试）
│   │       └── lora_sx1278.c    # SX1278 驱动（SPI 寄存器/收发/SF-BW-功率配置）
│   ├── Drivers/                 # STM32 HAL + CMSIS（已裁剪到 F411 必需集）
│   └── MDK-ARM/                 # Keil 工程 + startup
└── pc/                          # PC 侧：训练 / 量化 / 导出 / 采集
    ├── AirTune.py               # 纯 numpy：10 维特征 → MLP → int8 PTQ（自写 Adam）
    ├── export_model.py          # 导出 airtune_model.h
    ├── gen_testvec.py           # 生成交叉验证测试向量
    ├── test_infer_pc.c          # PC 侧跑同一份定点码做交叉验证
    ├── capture.py               # 一条命令：串口 → CSV → 图
    └── results/                 # 真机实测样本（log 200 20 的 CSV + 图）
```

## 硬件

| 模块 | 接 F411 | 用途 |
|---|---|---|
| **Ra-02（SX1278）** | SPI1：SCK/MISO/MOSI=PA5/6/7，NSS=PB6，RST=PB5，DIO0=PB4 | LoRa 电台（AirTune 控制对象） |
| **INA226** | I2C1：SCL=PB8，SDA=PB9，VCC=3.3V | 总线电压/电流/功率监测 |
| MPU6050（可选） | I2C1 共享 PB8/PB9（地址 0x68） | DriftGuard 传感器流 |
| INMP441（可选） | I2S2：PB10/PB12/PB15 | DriftGuard 声学场景 |

> ⚠️ LoRa 只能接 **3.3V**；INA226 的 IN+/IN− 必须**串进**被测电源线（3.3V→IN+→IN−→负载），绝不能跨接电源两端。

## 快速开始

### 固件

1. Keil 打开 `firmware/MDK-ARM/Demo.uvprojx`
2. **Options → Target → 勾选 Use MicroLIB**（printf 重定向必需）
3. Rebuild（F7）→ 烧录 → 串口 **COM4 @115200 8N1**
4. 开机自检通过会打印 `[BOOT] INA226 OK on I2C1 @0x40 (man=TI die=0x2260)` 和 `[BOOT] LoRa SX1278 OK: ver=0x12`

### PC 侧

```bash
cd pc
python -m venv .venv && .venv/bin/pip install numpy matplotlib pyserial   # Windows: .venv\Scripts\activate
.venv/bin/python AirTune.py            # 训练 + int8 量化验证（应看到 delta < 1%）
.venv/bin/python export_model.py       # 导出 ../firmware/Core/Inc/airtune_model.h
.venv/bin/python gen_testvec.py        # 生成测试向量（交叉验证用）
```

C 定点交叉验证（与 Python int8 参考对比）：

```bash
gcc test_infer_pc.c -I ../firmware/Core/Inc -O2 -o test_infer_pc && ./test_infer_pc
```

一条命令采集真机数据并出图：

```bash
.venv/bin/python capture.py "log 200 20"      # AirTune 推理遥测 → CSV + PNG
.venv/bin/python capture.py "pwrlog 300 20"   # INA226 功率遥测 → CSV + PNG
```

## 串口 CLI

| 命令 | 功能 |
|---|---|
| `help` / `get` / `set <name> <val>` | 参数实时调整（tx_power / sf / bw / heartbeat_ms / threshold / shunt_mohm） |
| `infer [k]` | 跑一次 AirTune 预测（k=0..3 四种链路工况） |
| `bench` | DWT CYCCNT 测推理延迟 + 权重 Flash KB + 栈深 |
| `ram` | 模型 Flash/RAM 占用报告 |
| `log <n> <ms>` | 流式推理遥测（CSV，配 capture.py 出图） |
| `power` / `pwrlog <n> <ms>` | INA226 电压/电流/功率（单次/流式） |
| `i2cscan` / `i2cpins` | I2C 总线扫描 / 线路电平诊断 |
| `lora init\|status\|send <text>\|recv [ms]\|freq <mhz>\|apply` | LoRa 电台：初始化/状态/收发/频率/应用 AirTune 参数 |

> CLI 带**空闲 1.5s 自动执行**，串口工具不配换行符也能用。

### LoRa 通联（AirTune 闭环）

```bash
lora init            # 复位+初始化，读 RegVersion，SX1278 应返回 0x12（验证 SPI 接线，不需天线）
set sf 12            # 改参数 → 电台实时跟随（打印 "radio applied"）
lora apply           # 一键把 AirTune 选出的 sf/bw/tx_power 灌给电台
lora send hello      # 发包（≤64 字节）
lora recv 3000       # 收包，打印内容 + RSSI + SNR
```

> ⚠️ **TX 前必须接天线**（IPEX 座直接插，或焊 17cm 线 @433MHz），没天线满功率发射会烧 PA。
> DIO0 不用接——驱动轮询 IrqFlags 判收发完成。

## 踩坑记录（面试可讲）

1. **INA226 CAL 寄存器只有 15 位**：Imax=0.5A + 10mΩ 时 CAL=34133 超出 0x7FFF，芯片静默丢弃最高位 → 电流读数缩小 24.6 倍。改为 Imax=1.0A（CAL≈0x4189）解决。*教训：芯片手册里的"不可写位"会静默改变量程，回读校验不能省。*
2. **IN+/IN− 跨接电源 = 短路**：采样电阻（10mΩ）直接跨在 3.3V 上要 30A，板子掉压重启循环，且 I2C 因掉压全程扫不到——一根线引发两个完全不同的症状。
3. **两个良品同时失联 → 查公共路径**：INA226 和 MPU6050 在面包板上齐失联、直连板子全活 → 祸首是板子到面包板的长跳线，不是器件。*教训：多器件共性问题优先怀疑共享路径。*
4. **I2C 半接触的签名**：`BUSY=1` + 「地址轮询能过、连续多寄存器读失败」= 接触不良。固件侧加 DeInit/Init 总线恢复 + 重试。
5. **ARMCC 中文乱码**：字符串字面量里的中文会触发 `#870`，printf 全用 ASCII。
6. **Keil 文件锁**：Keil 打开时普通重写报 Errno 22，需 `r+` 原地写 + `truncate()`。

## License

MIT（固件中 ST 的 HAL/CMSIS 文件遵循其原始许可，见各目录 LICENSE.txt）
