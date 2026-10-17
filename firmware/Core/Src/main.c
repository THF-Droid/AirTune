/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "airtune_infer.h"
#include "ina226.h"
#include "lora_sx1278.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
/* USART2/I2C1/SPI1 均由 CubeMX 生成的 HAL 驱动初始化（见自动生成的 MX_USART2_UART_Init 等），
   句柄 huart2 由 CubeMX 在自动段声明，本段只放用户私有变量。 */

/* ---- 可调全局参数：串口 CLI 用 set 命令修改的就是 g_params ---- */
typedef struct {
  int tx_power;     /* LoRa 发射功率 dBm，范围 2..20 */
  int sf;           /* 扩频因子，范围 7..12 */
  int bw;           /* 带宽 kHz：125 / 250 / 500 */
  int heartbeat_ms; /* 心跳打印周期(ms)，最小 100 */
  float threshold;  /* TinyML 异常检测阈值 */
  int shunt_mohm;   /* INA226 采样电阻毫欧值：模块丝印 R010=10 / R100=100 */
} params_t;
params_t g_params = {17, 7, 125, 1000, 0.85f, 10};

#define LORA_RX_PRINT_MAX 65   /* lora recv 打印缓冲（64 载荷 + NUL） */

/* CLI 行缓冲（非阻塞轮询用） */
static char cli_line[64];
static int cli_len = 0;
static uint32_t cli_last_char_ms = 0;   /* 最近一次收到字符的时间，用于空闲超时执行 */
static uint32_t cli_last_alive = 0;

/* AirTune 演示样本：4 个典型链路工况（归一化特征），bench/infer 用 */
static const float at_demo_x[4][AT_IN] = {
  { 0.6f, 0.6f,-0.5f,-0.2f, 0.5f,-0.1f, 0.7f, 0.7f, 0.0f,-0.5f},
  { 0.1f, 0.1f,-0.1f, 0.0f, 0.1f, 0.2f,-0.1f, 0.1f, 0.0f,-0.1f},
  {-0.2f,-0.1f, 0.1f, 0.1f, 0.2f, 0.4f,-0.2f,-0.2f, 0.0f,-0.2f},
  {-0.7f,-0.6f, 0.6f, 0.3f,-0.6f, 0.3f,-0.6f,-0.6f, 0.0f,-0.6f},
};
static const char *at_mode_name[4] = {
  "SF7/BW500 (fast)", "SF7/BW125 (balanced)", "SF10/BW125 (mid)", "SF12/BW125 (far)"
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void MX_USART2_UART_Init(void);
void MX_I2C1_Init(void);
void MX_SPI1_Init(void);


void cli_poll(void);
void cli_process(char *line);

void DWT_Init(void);
void DWT_Reset(void);
uint32_t DWT_GetCycles(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
MX_I2C1_Init();     /* I2C1: PB8/PB9 -> INA226 */
  MX_SPI1_Init();    /* SPI1: PA5/6/7 -> LoRa Ra-02 */
  MX_USART2_UART_Init();
  printf("[BOOT] HCLK=%lu PCLK1=%lu PCLK2=%lu\r\n",
         HAL_RCC_GetHCLKFreq(), HAL_RCC_GetPCLK1Freq(), HAL_RCC_GetPCLK2Freq());
  
  DWT_Init();   /* AIRTUNE_ML: 使能 DWT CYCCNT */
  if (ina226_init() == 0)
    printf("[BOOT] INA226 OK on I2C1 @0x%02X (man=TI die=0x2260)\r\n", ina226_addr());
  else
    printf("[BOOT] INA226 NOT found on I2C1: run 'i2cscan'; check SCL=PB8 SDA=PB9 VCC=3V3\r\n");
  if (lora_init() == 0x12)
    printf("[BOOT] LoRa SX1278 OK: ver=0x12 %.1fMHz sf7 bw125 17dBm (type 'lora status')\r\n",
           lora_info()->freq_mhz);
  else
    printf("[BOOT] LoRa SX1278 not found: check SPI1 wiring (PA5/6/7, NSS=PB6, RST=PB5)\r\n");
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    cli_poll();   /* 非阻塞读串口：收到回车即解析命令，不阻塞主循环 */

    /* 心跳：按 g_params.heartbeat_ms 周期打印（不再用 HAL_Delay 阻塞，CLI 才能实时响应） */
    if (HAL_GetTick() - cli_last_alive >= (uint32_t)g_params.heartbeat_ms)
    {
      cli_last_alive = HAL_GetTick();
      HAL_GPIO_TogglePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin);
      printf("[ALIVE] tick=%lu HCLK=%lu hb=%dms tx=%ddBm sf=%d th=%.2f\r\n",
             HAL_GetTick(), HAL_RCC_GetHCLKFreq(),
             g_params.heartbeat_ms, g_params.tx_power, g_params.sf,
             (double)g_params.threshold);
    }
/* USER CODE END 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LORA_RST_Pin|LORA_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_RST_Pin LORA_NSS_Pin */
  GPIO_InitStruct.Pin = LORA_RST_Pin|LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void MX_USART2_UART_Init(void)
{
  __HAL_RCC_USART2_CLK_ENABLE();
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_I2C1_Init(void)
{
  __HAL_RCC_I2C1_CLK_ENABLE();
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_SPI1_Init(void)
{
  __HAL_RCC_SPI1_CLK_ENABLE();
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}


/* Keil 下 printf 重定向：需在 Options for Target -> Target 勾选 Use MicroLIB。
   串口已由 CubeMX 生成的 MX_USART2_UART_Init（HAL_UART）初始化，这里直接转发。 */
int fputc(int ch, FILE *f)
{
  (void)f;
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* ===================== 串口 CLI 框架（begin） ===================== */
/* 非阻塞轮询：每次调一次，有字符就收；收到 \r/\n 才解析整行。
   用 HAL_UART_Receive(...,0) 立即返回，不阻塞主循环。 */
void cli_poll(void)
{
  /* 空闲超时自动执行：串口工具若不发 \r/\n，停止输入 1.5s 后也执行本行 */
  if (cli_len > 0 && (HAL_GetTick() - cli_last_char_ms) >= 1500)
  {
    cli_line[cli_len] = '\0';
    printf("\r\n");
    cli_process(cli_line);
    cli_len = 0;
  }
  uint8_t ch;
  if (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK)
  {
    if (ch == '\r' || ch == '\n')
    {
      if (cli_len > 0)
      {
        cli_line[cli_len] = '\0';
        printf("\r\n");
        cli_process(cli_line);
        cli_len = 0;
      }
    }
    else if (ch == '\b' || ch == 0x7F)   /* 退格 */
    {
      if (cli_len > 0) { cli_len--; printf("\b \b"); }
    }
    else if (cli_len < (int)sizeof(cli_line) - 1)
    {
      cli_line[cli_len++] = (char)ch;
      cli_last_char_ms = HAL_GetTick();
      printf("%c", ch);   /* 回显，方便看自己输了啥 */
    }
  }
}

/* 命令解析：help / get [name] / set <name> <val> / reboot */
void cli_process(char *line)
{
  char *tok = strtok(line, " ");
  if (tok == NULL) return;

  if (strcmp(tok, "help") == 0)
  {
    printf("Commands:\r\n");
    printf("  help              show this help\r\n");
    printf("  get [name]        print all or one param\r\n");
    printf("  set <name> <val>  change a param (live)\r\n");
    printf("  reboot            reset MCU\r\n");
    printf("  infer [k]         run one AirTune prediction (k=0..3)\r\n");
    printf("  bench             measure inference latency via DWT CYCCNT\r\n");
    printf("  ram               report model Flash KB + stack depth\r\n");
    printf("  log <n> <ms>      stream n CSV telemetry samples (Python capture)\r\n");
    printf("  power             read INA226 Vbus/current/power (one shot)\r\n");
    printf("  pwrlog <n> <ms>   stream n INA226 CSV samples (Python capture)\r\n");
    printf("  i2cscan           scan I2C1 bus for ACKing devices\r\n");
    printf("  i2cpins           read PB8/PB9 pin level (wiring check, no DMM)\r\n");
    printf("  lora init|status|send <text>|recv [ms]|freq <mhz>|apply\r\n");
    printf("Params: tx_power sf bw heartbeat_ms threshold shunt_mohm\r\n");
  }
  else if (strcmp(tok, "get") == 0)
  {
    char *name = strtok(NULL, " ");
    if (name == NULL)
    {
      printf("tx_power=%d sf=%d bw=%d heartbeat_ms=%d threshold=%.3f shunt_mohm=%d\r\n",
             g_params.tx_power, g_params.sf, g_params.bw,
             g_params.heartbeat_ms, (double)g_params.threshold, g_params.shunt_mohm);
    }
    else if (strcmp(name, "tx_power") == 0)     printf("tx_power=%d\r\n", g_params.tx_power);
    else if (strcmp(name, "sf") == 0)           printf("sf=%d\r\n", g_params.sf);
    else if (strcmp(name, "bw") == 0)           printf("bw=%d\r\n", g_params.bw);
    else if (strcmp(name, "heartbeat_ms") == 0) printf("heartbeat_ms=%d\r\n", g_params.heartbeat_ms);
    else if (strcmp(name, "threshold") == 0)    printf("threshold=%.3f\r\n", (double)g_params.threshold);
    else if (strcmp(name, "shunt_mohm") == 0)   printf("shunt_mohm=%d\r\n", g_params.shunt_mohm);
    else printf("unknown param: %s\r\n", name);
  }
  else if (strcmp(tok, "set") == 0)
  {
    char *name = strtok(NULL, " ");
    char *val  = strtok(NULL, " ");
    if (name == NULL || val == NULL) { printf("usage: set <name> <val>\r\n"); return; }
    int iv = atoi(val);
    float fv = (float)atof(val);
    if (strcmp(name, "tx_power") == 0) {
      if (iv < 2 || iv > 20) printf("tx_power out of range 2..20\r\n");
      else { g_params.tx_power = iv;
             if (lora_is_up()) lora_set_power(iv);
             printf("tx_power=%d%s\r\n", iv, lora_is_up() ? " (radio applied)" : ""); }
    } else if (strcmp(name, "sf") == 0) {
      if (iv < 7 || iv > 12) printf("sf out of range 7..12\r\n");
      else { g_params.sf = iv;
             if (lora_is_up()) lora_set_sf(iv);
             printf("sf=%d%s\r\n", iv, lora_is_up() ? " (radio applied)" : ""); }
    } else if (strcmp(name, "bw") == 0) {
      g_params.bw = iv;
      if (lora_is_up()) lora_set_bw(iv);
      printf("bw=%d%s\r\n", iv, lora_is_up() ? " (radio applied)" : "");
    } else if (strcmp(name, "heartbeat_ms") == 0) {
      if (iv < 100) printf("heartbeat_ms min 100\r\n");
      else { g_params.heartbeat_ms = iv; printf("heartbeat_ms=%d\r\n", iv); }
    } else if (strcmp(name, "threshold") == 0) {
      g_params.threshold = fv; printf("threshold=%.3f\r\n", (double)fv);
    } else if (strcmp(name, "shunt_mohm") == 0) {
      if (iv < 1 || iv > 1000) printf("shunt_mohm out of range 1..1000 (mOhm)\r\n");
      else { g_params.shunt_mohm = iv; ina226_set_shunt_mohm((uint32_t)iv);
             printf("shunt_mohm=%d (CAL rewritten)\r\n", iv); }
    } else {
      printf("unknown param: %s\r\n", name);
    }
  }
  else if (strcmp(tok, "reboot") == 0)
  {
    printf("rebooting...\r\n");
    HAL_Delay(50);
    HAL_NVIC_SystemReset();
  }  else if (strcmp(tok, "infer") == 0)
  {
    char *ks = strtok(NULL, " ");
    int k = 0;
    if (ks) { int v = atoi(ks); if (v >= 0 && v < 4) k = v; }
    int cls; airtune_predict_norm(at_demo_x[k], &cls);
    printf("infer mode%d -> pred=%d %s\r\n", k, cls, at_mode_name[cls]);
  }
  else if (strcmp(tok, "bench") == 0)
  {
    const int N = 200; int cls; uint32_t t0, t1;
    uint32_t best = 0xFFFFFFFFu, worst = 0, sum = 0;
    uint32_t sp_caller = __get_MSP();
    for (int i = 0; i < N; i++) {
      DWT_Reset(); t0 = DWT_GetCycles();
      airtune_predict_norm(at_demo_x[0], &cls);
      t1 = DWT_GetCycles();
      uint32_t dt = t1 - t0; sum += dt; if (dt < best) best = dt; if (dt > worst) worst = dt;
    }
    double hz = (double)SystemCoreClock;
    printf("[BENCH] N=%d min=%lu cyc (%.2f us) avg=%.2f us worst=%.2f us @%.0fMHz\r\n",
           N, best, (double)best/hz*1e6, (double)(sum/N)/hz*1e6, (double)worst/hz*1e6, hz/1e6);
    uint32_t depth = sp_caller - g_at_sp_entry;
    printf("[BENCH] weights Flash=%.2f KB  infer stack=%lu B  (const weights use 0 RAM)\r\n",
           (double)AT_MODEL_FLASH_BYTES/1024.0, depth);
  }
  else if (strcmp(tok, "ram") == 0)
  {
    int cls; uint32_t sp_caller = __get_MSP();
    airtune_predict_norm(at_demo_x[0], &cls);
    uint32_t depth = sp_caller - g_at_sp_entry;
    printf("[RAM] model const(Flash)=%d B (%.2f KB)  infer stack=%lu B  weights RAM=0\r\n",
           (int)AT_MODEL_FLASH_BYTES, (double)AT_MODEL_FLASH_BYTES/1024.0, depth);
    printf("[RAM] weights/bias/scales all const -> in Flash, 0 B RAM at runtime\r\n");
  }
  else if (strcmp(tok, "log") == 0)
  {
    char *ns = strtok(NULL, " ");
    char *ms = strtok(NULL, " ");
    int n = ns ? atoi(ns) : 50;
    int period = ms ? atoi(ms) : 20;
    if (n < 1) n = 1;        if (n > 2000) n = 2000;
    if (period < 1) period = 1;
    printf("LOG_BEGIN %d %d\r\n", n, period);
    for (int i = 0; i < n; i++) {
      float x[AT_IN];
      for (int j = 0; j < AT_IN; j++) x[j] = at_demo_x[i & 3][j];
      int cls;
      DWT_Reset();
      uint32_t t0 = DWT_GetCycles();
      airtune_predict_norm(x, &cls);
      uint32_t cyc = DWT_GetCycles() - t0;
      printf("LOG,%lu,%d,%d,%lu,%d,%d\r\n",
             (unsigned long)HAL_GetTick(), i & 3, cls, (unsigned long)cyc,
             g_params.tx_power, g_params.sf);
      HAL_Delay((uint32_t)period);
    }
    printf("LOG_END\r\n");
  }
  else if (strcmp(tok, "power") == 0)
  {
    ina226_t pwr;
    uint16_t man = 0, die = 0;
    if (ina226_read_all(&pwr) == 0 && ina226_probe(&man, &die) == 0)
    {
      printf("[PWR] INA226@0x%02X man=0x%04X die=0x%04X cfg=0x%04X cal=0x%04X OK\r\n",
             ina226_addr(), man, die, pwr.config, pwr.calib);
      printf("[PWR] Vbus=%ld mV  Vshunt=%ld uV  I=%ld mA  P=%ld mW  (shunt=%d mOhm)\r\n",
             (long)pwr.vbus_mV, (long)pwr.vshunt_uV, (long)pwr.current_mA,
             (long)pwr.power_mW, g_params.shunt_mohm);
    }
    else
    {
      printf("[PWR] INA226 read fail @0x%02X on I2C1: run 'i2cscan'; check SCL=PB8 SDA=PB9 VCC=3V3 GND\r\n", ina226_addr());
    }
  }
  else if (strcmp(tok, "pwrlog") == 0)
  {
    char *ns = strtok(NULL, " ");
    char *ms = strtok(NULL, " ");
    int n = ns ? atoi(ns) : 100;
    int period = ms ? atoi(ms) : 20;
    if (n < 1) n = 1;        if (n > 2000) n = 2000;
    if (period < 1) period = 1;
    printf("PWR_BEGIN %d %d\r\n", n, period);
    for (int i = 0; i < n; i++) {
      ina226_t p;
      if (ina226_read_all(&p) == 0)
        printf("PWR,%lu,%ld,%ld,%ld,%ld\r\n",
               (unsigned long)HAL_GetTick(), (long)p.vbus_mV,
               (long)p.current_mA, (long)p.power_mW, (long)p.vshunt_uV);
      else
        printf("PWR,%lu,-1,-1,-1,-1\r\n", (unsigned long)HAL_GetTick());
      HAL_Delay((uint32_t)period);
    }
    printf("PWR_END\r\n");
  }
  else if (strcmp(tok, "i2cscan") == 0)
  {
    /* 外设健康快照：state=32(0x20)=READY；CR1 bit0=PE 使能；CR2=0x0032 表示 50MHz APB1；BUSY=0 总线空闲 */
    printf("[I2C] state=%d lock=%d CR1=0x%04lX CR2=0x%04lX SR2=0x%04lX BUSY=%d\r\n",
           (int)hi2c1.State, (int)hi2c1.Lock,
           (unsigned long)(hi2c1.Instance->CR1 & 0xFFFFu),
           (unsigned long)(hi2c1.Instance->CR2 & 0xFFFFu),
           (unsigned long)(hi2c1.Instance->SR2 & 0xFFFFu),
           (hi2c1.Instance->SR2 & I2C_SR2_BUSY) ? 1 : 0);
    HAL_I2C_DeInit(&hi2c1);   /* 重初始化，排除启动探测失败留下的锁死状态 */
    HAL_I2C_Init(&hi2c1);
    printf("[I2C] re-init ok, scanning 0x08..0x77 on I2C1 (PB8=SCL PB9=SDA)...\r\n");
    int found = 0;
    for (int a7 = 0x08; a7 <= 0x77; a7++)
    {
      if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a7 << 1), 2, 10) == HAL_OK)
      {
        printf("[I2C] ACK @ 0x%02X\r\n", a7);
        found++;
      }
    }
    if (found == 0)
      printf("[I2C] none -> check wiring/power: SDA<->PB9 SCL<->PB8 VCC<->3V3 GND<->GND\r\n");
    else
      printf("[I2C] %d device(s) found\r\n", found);
  }
  else if (strcmp(tok, "lora") == 0)
  {
    char *sub = strtok(NULL, " ");
    if (sub == NULL) {
      printf("usage: lora init|status|send <text>|recv [ms]|freq <mhz>|apply\r\n");
    } else if (strcmp(sub, "init") == 0) {
      int v = lora_init();
      if (v == 0x12)
        printf("[LORA] SX1278 OK ver=0x12 %.1fMHz sf=%d bw=%d tx=%ddBm\r\n",
               lora_info()->freq_mhz, lora_info()->sf,
               lora_info()->bw_khz, lora_info()->tx_dbm);
      else
        printf("[LORA] init FAIL: RegVersion=0x%02X (expect 0x12)\r\n", lora_info()->version);
        printf("[LORA] check: VCC=3V3 GND SCK=PA5 MISO=PA6 MOSI=PA7 NSS=PB6 RST=PB5\r\n");
    } else if (strcmp(sub, "status") == 0) {
      const lora_info_t *li = lora_info();
      printf("[LORA] %s ver=0x%02X freq=%.1fMHz sf=%d bw=%dkHz cr=4/5 tx=%ddBm\r\n",
             lora_is_up() ? "UP" : "DOWN", li->version, li->freq_mhz,
             li->sf, li->bw_khz, li->tx_dbm);
      if (lora_is_up())
        printf("[LORA] last pkt: RSSI=%d dBm  SNR=%.1f dB\r\n", li->last_rssi, li->last_snr);
    } else if (strcmp(sub, "send") == 0) {
      char *txt = sub + strlen(sub) + 1;      /* send 后面的原文 */
      while (*txt == ' ') txt++;
      if (*txt == 0) { printf("usage: lora send <text>\r\n"); }
      else {
        int len = (int)strlen(txt);
        int rc = lora_send((const uint8_t *)txt, len);
        if (rc == 0) printf("[LORA] TX done: %d bytes (RSSI unknown on TX)\r\n", len);
        else printf("[LORA] TX timeout/fail (radio up? antenna connected?)\r\n");
      }
    } else if (strcmp(sub, "recv") == 0) {
      char *ms = strtok(NULL, " ");
      int tmo = ms ? atoi(ms) : 2000;
      uint8_t buf[LORA_RX_PRINT_MAX];
      int rssi = 0; float snr = 0.0f;
      int n = lora_recv(buf, LORA_RX_PRINT_MAX - 1, tmo, &rssi, &snr);
      if (n > 0) {
        buf[n] = 0;
        printf("[LORA] RX %d bytes: %s  (RSSI=%d dBm SNR=%.1f dB)\r\n", n, buf, rssi, snr);
      } else if (n == -2) {
        printf("[LORA] RX CRC error\r\n");
      } else {
        printf("[LORA] RX timeout (%d ms)\r\n", tmo);
      }
    } else if (strcmp(sub, "freq") == 0) {
      char *m = strtok(NULL, " ");
      float f = m ? (float)atof(m) : 0.0f;
      if (f < 137.0f || f > 1020.0f) printf("freq out of range 137..1020 MHz\r\n");
      else { lora_set_freq(f); printf("[LORA] freq=%.1f MHz (both ends must match)\r\n", f); }
    } else if (strcmp(sub, "apply") == 0) {
      if (!lora_is_up()) { printf("[LORA] radio down, run lora init first\r\n"); }
      else {
        lora_apply_params(g_params.sf, g_params.bw, g_params.tx_power);
        printf("[LORA] applied AirTune params: sf=%d bw=%d tx=%ddBm\r\n",
               g_params.sf, g_params.bw, g_params.tx_power);
      }
    } else {
      printf("unknown lora subcmd: %s\r\n", sub);
    }
  }
  else if (strcmp(tok, "i2cpins") == 0)
  {
    /* PB8/PB9 临时切普通输入(无内部上下拉)读电平，再恢复 I2C1 AF4 开漏。
       模块已上电且线路通 -> 模块上的上拉电阻应把线拉高，读到 1；
       读到 0 = 线断/松/模块没电。 */
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_Delay(2);
    int sda = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
    int scl = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
    printf("[I2CPIN] PB9(SDA)=%d PB8(SCL)=%d  (1=pulled high OK, 0=low/floating)\r\n", sda, scl);
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &g);
    printf("[I2CPIN] PB8/PB9 restored to I2C1 AF4\r\n");
  }

  else
  {
    printf("unknown cmd: %s (type 'help')\r\n", tok);
  }
}
/* ===================== 串口 CLI 框架（end） ===================== */

/* ===================== AIRTUNE_ML: DWT + 栈测量（begin） ===================== */
void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
}
void DWT_Reset(void) { DWT->CYCCNT = 0; }
uint32_t DWT_GetCycles(void) { return DWT->CYCCNT; }
/* ===================== AIRTUNE_ML: DWT + 栈测量（end） ===================== */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
