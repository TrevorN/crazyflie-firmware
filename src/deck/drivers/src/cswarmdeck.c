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

#define DEBUG_MODULE "uSD"

//System
#include <stdint.h>
#include <string.h>
#include "stm32fxxx.h"

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
#define IR_PIN        DECK_GPIO_IO4


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
static uint8_t deviceID = 43;
static uint8_t IRPulse = 0; //Current state of IR LED.
static uint8_t IRphaseSkip;
static uint8_t IRIndex=0;
static uint16_t IRPeriod=0xFFFE;
static uint16_t IRPulseWidth=0x7FFF;

void TIM2_IRQHandler()
{
    if(IRIndex == 8){
      TIM_PrescalerConfig(TIM10, 1, TIM_PSCReloadMode_Update);
      IRIndex = 0;
      return;
    }

    if(deviceID & (1 << IRIndex))
    {
      TIM_SelectOC1M(TIM10, 1, TIM_OCMode_PWM1);
    } else {
      TIM_SelectOC1M(TIM10, 1, TIM_ForcedAction_Active); //Force IR led high.
    }

    if(IRIndex == 7) //at the next event, disable output compare and set the prescaler.
    {
      TIM_SelectOC1M(TIM10, 1, TIM_ForcedAction_Active); //Force IR led high.

      TIM_PrescalerConfig(TIM10, 2300, TIM_PSCReloadMode_Update);
    }

    IRIndex++;
}

static void cswarmdeckIRInit(DeckInfo *info)
{
  pinMode(IR_PIN, OUTPUT);

  TIM_TimeBaseInitTypeDef IRTimer;
  TIM_OCInitTypeDef IROC;

  TIM_TimeBaseStructInit(&IRTimer);
  TIM_OCStructInit(&IROC);
  //Timer configuration
  //Timer 10 selected: 16 bit counter, 16 bit prescaler, one output channel.
  IRTimer.TIM_Period = IRPeriod;
  IRTimer.TIM_Prescaler = 0; //For now don't prescale.
  IRTimer.TIM_ClockDivision = 0; //Also no clock division.
  IRTimer.TIM_CounterMode = TIM_CounterMode_Up;
  IRTimer.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM10, &IRTimer);

  //Output compare configuration
  IROC.TIM_OCMode = TIM_OCMode_PWM1;
  IROC.TIM_OutputState = TIM_OutputState_Enable;
  IROC.TIM_Pulse = IRPulseWidth; //Half, for now.
  IROC.TIM_OCPolarity = TIM_OCPolarity_High;
  IROC.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OC1Init(TIM10, &IROC);

  //Interrupt configuration
  TIM_ITConfig(TIM10, TIM_IT_Update, ENABLE); //Enable update interrupts, used to trigger phase shifts.

  NVIC_InitTypeDef nvicStructure;
  nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);

  TIM_Cmd(TIM10, ENABLE);
}


/******************* Deck Interfaces ***************/

static void cswarmdeckInit(DeckInfo *info)
{
  cswarmdeckLEDInit(info);
  cswarmdeckIRInit(info);
  usdInit(info);
}

//Parameter registry
PARAM_GROUP_START(cswarm_deck)
PARAM_ADD(PARAM_UINT8, effect, &effect) //The selected parameter to display.
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect) //Read max # of parameters.
PARAM_ADD(PARAM_UINT8, IRphaseSkip, &IRphaseSkip) //Set number of pulses to stall for.
PARAM_ADD(PARAM_UINT8, IRPulse, &IRPulse) //0: off, 1: on, 2: 250Hz
PARAM_GROUP_STOP(cswarm_deck)

//Deck driver declaration
static const DeckDriver cswarm_deck = {
  .vid = 0xBC,//TODO: Pick new vid
  .pid = 0x08,//TODO: Pick new pid
  .name = "bcUSD",
  .usedGpio = DECK_USING_MISO|DECK_USING_MOSI|DECK_USING_SCK|DECK_USING_IO_4,
  .usedPeriph = DECK_USING_SPI,
  .init = cswarmdeckInit,
  //.test = cswarmdeckTest, //CURRENTLY UNIMPLEMENTED
};

DECK_DRIVER(cswarm_deck);
