 /**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * usddeck.c: micro SD deck driver. Implements logging to micro SD card.
 */

#define DEBUG_MODULE "CSWARM_DECK"

//System
#include <stdint.h>
#include <string.h>
#include "stm32fxxx.h"
#include "nvicconf.h"

//OS
#include "FreeRTOS.h"
#include "task.h"
#include "worker.h"
#include "timers.h"

//Filesystem
#include "ff.h"
#include "fatfs_sd.h"
#include "cswarmdeck.h"

//uSD
#include "deck.h"
#include "deck_spi.h"
#include "system.h"
#include "debug.h"
#include "led.h"

//LEDs
#include "ws2812.h" //LED driver
#include "param.h" //parameters
#include "pm.h" //power management
#include "deck_digital.h"

// Hardware defines
#define USD_CS_PIN    DECK_GPIO_IO3
#define IR_PIN        DECK_GPIO_IO1
#define IR_TIM        TIM10


/**************** Some useful macros ***************/

#define RED {0x10, 0x00, 0x00}
#define GREEN {0x00, 0x10, 0x00}
#define BLUE {0x00, 0x00, 0x10}
#define WHITE {0xff, 0xff, 0xff}
#define BLACK {0x00, 0x00, 0x00}

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)
#define COPY_COLOR(dest, orig) dest[0]=orig[0]; dest[1]=orig[1]; dest[2]=orig[2]
#define ADD_COLOR(dest, o1, o2) dest[0]=(o1[0]>>1)+(o2[0]>>1);dest[1]=(o1[1]>>1)+(o2[1]>>1);dest[2]=(o1[2]>>1)+(o2[2]>>1);
#define LIMIT(a) ((a>255)?255:(a<0)?0:a)
#define SIGN(a) ((a>=0)?1:-1)
#define DEADBAND(a, b) ((a<b) ? 0:a)
#define LINSCALE(domain_low, domain_high, codomain_low, codomain_high, value) ((codomain_high - codomain_low) / (domain_high - domain_low)) * (value - domain_low) + codomain_low
#define SET_WHITE(dest, intensity) dest[0] = intensity; dest[1] = intensity; dest[2] = intensity;

static uint32_t effect = 6;
static uint32_t neffect;
static uint8_t black[][3] = {BLACK, BLACK, BLACK,
                            BLACK, BLACK, BLACK,
                            BLACK, BLACK, BLACK,
                            BLACK, BLACK, BLACK,
                           };

static const uint8_t green[] = {0x00, 0xFF, 0x00};
static const uint8_t red[] = {0xFF, 0x00, 0x00};
static const uint8_t blue[] = {0x00, 0x00, 0xFF};
static const uint8_t white[] = WHITE;
static const uint8_t part_black[] = BLACK;

uint8_t ledringmem[NBR_LEDS * 2];

// FATFS low lever driver functions.
static void initSpi(void);
static void setSlowSpiMode(void);
static void setFastSpiMode(void);
static BYTE xchgSpi(BYTE dat);
static void rcvrSpiMulti(BYTE *buff, UINT btr);
static void xmitSpiMulti(const BYTE *buff, UINT btx);
static void csHigh(void);
static void csLow(void);

static BYTE exchangeBuff[512];
#ifdef USD_RUN_DISKIO_FUNCTION_TESTS
DWORD workBuff[512];  /* 2048 byte working buffer */
#endif

static xTimerHandle timer;
static void usdTimer(xTimerHandle timer);

//Fatfs object
FATFS FatFs;
//File object
FIL logFile;

// Low lever driver functions
static sdSpiContext_t sdSpiContext =
{
  .initSpi = initSpi,
  .setSlowSpiMode = setSlowSpiMode,
  .setFastSpiMode = setFastSpiMode,
  .xchgSpi = xchgSpi,
  .rcvrSpiMulti = rcvrSpiMulti,
  .xmitSpiMulti = xmitSpiMulti,
  .csLow = csLow,
  .csHigh = csHigh,

  .stat = STA_NOINIT,
  .timer1 = 0,
  .timer2 = 0
};

static DISKIO_LowLevelDriver_t fatDrv =
{
    SD_disk_initialize,
    SD_disk_status,
    SD_disk_ioctl,
    SD_disk_write,
    SD_disk_read,
    &sdSpiContext,
};


/*-----------------------------------------------------------------------*/
/* FATFS SPI controls (Platform dependent)                               */
/*-----------------------------------------------------------------------*/

/* Initialize MMC interface */
static void initSpi(void)
{
  spiBegin();   /* Enable SPI function */
  pinMode(USD_CS_PIN, OUTPUT);
  csHigh();

  // FIXME: DELAY of 10ms?
}

static void setSlowSpiMode(void)
{
  spiConfigureSlow();
}

static void setFastSpiMode(void)
{
  spiConfigureFast();
}

/* Exchange a byte */
static BYTE xchgSpi(BYTE dat)
{
  BYTE receive;

  spiExchange(1, &dat, &receive);
  return (BYTE)receive;
}

/* Receive multiple byte */
static void rcvrSpiMulti(BYTE *buff, UINT btr)
{
  memset(exchangeBuff, 0xFFFFFFFF, btr);
  spiExchange(btr, exchangeBuff, buff);
}

/* Send multiple byte */
static void xmitSpiMulti(const BYTE *buff, UINT btx)
{
  spiExchange(btx, buff, exchangeBuff);
}

static void csHigh(void)
{
  digitalWrite(USD_CS_PIN, 1);
}

static void csLow(void)
{
  digitalWrite(USD_CS_PIN, 0);
}



/*********** Deck driver initialization ***************/

static bool isInit = false;

static void usdInit(DeckInfo *info)
{
  isInit = true;

  FATFS_AddDriver(&fatDrv, 0);

  timer = xTimerCreate( "usdTimer", M2T(SD_DISK_TIMER_PERIOD_MS), pdTRUE, NULL, usdTimer);
  xTimerStart(timer, 0);
}

// static bool usdTest()
// {
//   if (!isInit)
//   {
//     DEBUG_PRINT("Error while initializing uSD deck\n");
//   }
// #ifdef USD_RUN_DISKIO_FUNCTION_TESTS
//   int result;
//   extern int test_diskio (BYTE pdrv, UINT ncyc, DWORD* buff, UINT sz_buff);

//   result = test_diskio(0, 1, (DWORD*)&workBuff, sizeof(workBuff));
//   if (result)
//   {
//     DEBUG_PRINT("(result=%d)\nFatFs diskio functions test [FAIL].\n", result);
//     isInit = false;
//   }
//   else
//   {
//     DEBUG_PRINT("FatFs diskio functions test [OK].\n");
//   }
// #endif

//   return isInit;
// }

static void usdTimer(xTimerHandle timer)
{
  SD_disk_timerproc(&sdSpiContext);
}

/****************** LED Drivers ********************/
typedef void (*led4Effect)(uint8_t buffer[][3], bool reset);

static void cardinalEffect(uint8_t buffer[][3], bool reset)
{
  int i;
  //led pattern 0, 1, 2, 3
  //          FrontRight, BR, BL, FL

  if (reset)
  {
    for (i=0; i<NBR_LEDS; i++) {
      COPY_COLOR(buffer[i], part_black);
    }
  }

  for (i=0; i<NBR_LEDS; i++)
  {
                            //f:b
    buffer[i][0] = (i%2==0)?50:0; //r
    buffer[i][1] = (i%2==0)?0:50; //g
    buffer[i][2] = (i%2==0)?0:0; //b
  }
}

// static void spinEffect(uint8_t buffer[][3], bool reset)
// {
//   int i;

//   if (reset)
//   {
//     for (i=0; i<NBR_LEDS; i++) {
//       COPY_COLOR(buffer[i], part_black);
//     }
//   }

//   for (i = 0; i < NBR_LEDS; i++)
//   {
//     uint8_t R5, G6, B5;
//     uint8_t (*led)[2] = (uint8_t (*)[2])ledringmem;
//     // Convert from RGB565 to RGB888
//     R5 = led[i][0] >> 3;
//     G6 = ((led[i][0] & 0x07) << 3) | (led[i][1] >> 5);
//     B5 = led[i][1] & 0x1F;
//     buffer[i][0] = ((uint16_t)R5 * 527 + 23 ) >> 6;
//     buffer[i][1] = ((uint16_t)G6 * 259 + 33 ) >> 6;
//     buffer[i][2] = ((uint16_t)B5 * 527 + 23 ) >> 6;
//   }
// }

//List of available 4LED effects.
static led4Effect effectsFct[] =
{
  cardinalEffect
  //spinEffect
};

static xTimerHandle LEDtimer;

//Called every 50ms. Updates LEDs.
void cswarmdeckLEDWorker(void * data)
{
  static int current_effect = 0;
  static uint8_t buffer[NBR_LEDS][3];
  bool reset = true;

  if ((effect > neffect)) {
    ws2812Send(black, NBR_LEDS);
    return;
  }

  if (current_effect != effect) {
    reset = true;
  } else {
    reset = false;
  }
  current_effect = effect;

  effectsFct[current_effect](buffer, reset);
  ws2812Send(buffer, NBR_LEDS);
}

//LED timer callback.
static void cswarmdeckLEDTimer(xTimerHandle LEDtimer)
{
  workerSchedule(cswarmdeckLEDWorker, NULL);
}

static void cswarmdeckLEDInit(DeckInfo *info)
{
  ws2812Init();

  neffect = sizeof(effectsFct)/sizeof(effectsFct[0]) - 1;

  pinMode(USD_CS_PIN, OUTPUT);

  //Every 50ms schedule the LED Worker.
  LEDtimer = xTimerCreate( "cswarmdeckLEDTimer", M2T(50),
                                     pdTRUE, NULL, cswarmdeckLEDTimer);
  xTimerStart(LEDtimer, 100);
}
/*************** Infrared Drivers ******************/
//Becuase codes are so short, it's probably better to control in software.
//Every 100ms, a hardware interrupt that controls the strobing behavior is enabled.
//Normally leave IR diode on.

//Need control over pulse frequency and pulse phase. Need to blink LED at ~120Hz.
uint16_t deviceID = 44;
volatile uint8_t IRIndex=0;
uint16_t unpacked_ID[16];

//Convert the device ID into a series of pulse widths with added parity bit
void IRUnpackID(uint8_t id)
{
  bool parity = 0;
  for(int i = 0; i < 15; i++)
  {
    unpacked_ID[i] = (deviceID & (0x4000 >> i)) ? 500 : 0;
    parity ^= deviceID & (0x4000 >> i);
  }
  unpacked_ID[15] = parity ? 500: 0;
}


void IRPinsInit() {
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Clock for GPIOD */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  /* Alternating functions for pins */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
  
  /* Set pins */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
}

//Timer Initializaiton
void IRTimerInit() {
  TIM_TimeBaseInitTypeDef TIM_BaseStruct;

  /* Enable clock for TIM10 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  TIM_BaseStruct.TIM_Prescaler = 1200;
  /* Count up */
  TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_BaseStruct.TIM_Period = 1000; /* 10kHz PWM */
  TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_BaseStruct.TIM_RepetitionCounter = 0;
  /* Initialize TIM10 */
  TIM_TimeBaseInit(TIM3, &TIM_BaseStruct);
  /* Start count on TIM10 */
  TIM_Cmd(TIM3, ENABLE);
}

//Output Compare Initialization
void IRPWMInit() {
  TIM_OCInitTypeDef TIM_OCStruct;
  TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCStruct.TIM_Pulse = 500; /* 50% duty cycle */
  TIM_OC1Init(TIM3, &TIM_OCStruct);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

//Interrupt configuration
void IRInterruptInit() {
  DMA_InitTypeDef DMA_InitStructure;
  //NVIC_InitTypeDef NVIC_InitStructure;

  //Create 8 byte buffer of LED timing values.
  IRUnpackID(deviceID);

  DEBUG_PRINT("ID Vec High: %d, %d, %d, %d\n", unpacked_ID[0], unpacked_ID[1], unpacked_ID[2], unpacked_ID[3]);
  DEBUG_PRINT("ID Vec Low: %d, %d, %d, %d\n", unpacked_ID[4], unpacked_ID[5], unpacked_ID[6], unpacked_ID[7]);
  DEBUG_PRINT("ID Vec Low: %d, %d, %d, %d\n", unpacked_ID[8], unpacked_ID[9], unpacked_ID[10], unpacked_ID[11]);
  DEBUG_PRINT("ID Vec Low: %d, %d, %d, %d\n", unpacked_ID[12], unpacked_ID[13], unpacked_ID[14], unpacked_ID[15]);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  /* DMA1 Channel2 Config */
  DMA_DeInit(DMA1_Stream4);

  // USART TX DMA Channel Config
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM3->CCR1;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&unpacked_ID;    // this is the buffer memory
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_BufferSize = 16;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_Channel = DMA_Channel_5;
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);


  // NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
  // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_LOW_PRI;
  // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  // NVIC_Init(&NVIC_InitStructure);

  // DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);
  // DMA_ITConfig(DMA1_Stream4, DMA_IT_HT, ENABLE);

  /* TIM3 CC1 DMA Request enable */
  TIM_DMACmd(TIM3, TIM_DMA_CC1, ENABLE);

  DMA_Cmd(DMA1_Stream4, ENABLE);
}

static void cswarmdeckIRInit(DeckInfo *info)
{
  IRPinsInit();
  IRTimerInit();
  IRPWMInit();
  IRInterruptInit();

  DEBUG_PRINT("CCR1: %d", (int)TIM3->CCR1);
}


/******************* Deck Interfaces ***************/

static void cswarmdeckInit(DeckInfo *info)
{
  DEBUG_PRINT("cswarm deck initializing\n");
  cswarmdeckIRInit(info);
  return;
  cswarmdeckLEDInit(info);
  usdInit(info);
  DEBUG_PRINT("cswarm deck initialized\n");
}

// //Parameter registry
// PARAM_GROUP_START(cswarm_deck)
// PARAM_ADD(PARAM_UINT8, effect, &effect) //The selected parameter to display.
// PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect) //Read max # of parameters.
// PARAM_ADD(PARAM_UINT8, IRphaseSkip, &IRphaseSkip) //Set number of pulses to stall for.
// PARAM_ADD(PARAM_UINT8, IRPulse, &IRPulse) //0: off, 1: on, 2: 250Hz
// PARAM_GROUP_STOP(cswarm_deck)

//Deck driver declaration
static const DeckDriver cswarm_deck = {
  .name = "cswarm",
  .usedGpio = DECK_USING_MISO|DECK_USING_MOSI|DECK_USING_SCK|DECK_USING_IO_4,
  .usedPeriph = DECK_USING_SPI,
  .init = cswarmdeckInit,
  //.test = cswarmdeckTest, //CURRENTLY UNIMPLEMENTED
};

DECK_DRIVER(cswarm_deck);
