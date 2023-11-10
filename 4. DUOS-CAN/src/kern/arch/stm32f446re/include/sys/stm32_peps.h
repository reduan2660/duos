/*
 * Copyright (c) 2022
 * Computer Science and Engineering, University of Dhaka
 * Credit: CSE Batch 25 (starter) and Prof. Mosaddek Tushar
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef __STM32_PEPS_H
#define __STM32_PEPS_H
#include <stdint.h>
#define PWR ((PWR_TypeDef *)0x40007000)
#define FLASH ((FLASH_TypeDef *)0x40023C00)
#define RCC ((RCC_TypeDef *)0x40023800)
#define SYSCFG ((SYSCFG_TypeDef *)0x40013800)
#define EXTI ((EXTI_TypeDef *)0x40013C00)
/**
 * Define GPIOx Port
 **/
#define GPIOA ((GPIO_TypeDef *)0x40020000)
#define GPIOB ((GPIO_TypeDef *)0x40020400)
#define GPIOC ((GPIO_TypeDef *)0x40020800)
#define GPIOD ((GPIO_TypeDef *)0x40020C00)
#define GPIOE ((GPIO_TypeDef *)0x40021000)
#define GPIOF ((GPIO_TypeDef *)0x40021400)
#define GPIOG ((GPIO_TypeDef *)0x40021800)
#define GPIOH ((GPIO_TypeDef *)0x40021C00)
/**
 * Define USARTx
 **/
#define USART1 ((USART_TypeDef *)0x40011000)
#define USART2 ((USART_TypeDef *)0x40004400)
#define USART3 ((USART_TypeDef *)0x40004800)
#define USART4 ((USART_TypeDef *)0x40004C00)
#define USART5 ((USART_TypeDef *)0x40005000)
#define USART6 ((USART_TypeDef *)0x40011400)
/**
 * Define TIMERx
 **/
#define TIM1 ((TIM_TypeDef *)0x40010000)
#define TIM2 ((TIM_TypeDef *)0x40000000)
#define TIM3 ((TIM_TypeDef *)0x40000400)
#define TIM4 ((TIM_TypeDef *)0x40000800)
#define TIM5 ((TIM_TypeDef *)0x40000C00)
#define TIM6 ((TIM_TypeDef *)0x40001000)
#define TIM7 ((TIM_TypeDef *)0x40001400)
#define TIM8 ((TIM_TypeDef *)0x40010400)
#define TIM9 ((TIM_TypeDef *)0x40014000)
#define TIM10 ((TIM_TypeDef *)0x40014400)
#define TIM11 ((TIM_TypeDef *)0x40014800)
#define TIM12 ((TIM_TypeDef *)0x40001800)
#define TIM13 ((TIM_TypeDef *)0x40001C00)
#define TIM14 ((TIM_TypeDef *)0x40002000)

/**
 *
 * Data Structure for FLASH control Register
 **/
typedef struct
{
    uint32_t volatile ACR;
    uint32_t volatile KEYR;
    uint32_t volatile OPTKEYR;
    uint32_t volatile SR;
    uint32_t volatile CR;
    uint32_t volatile OPTCR;
} FLASH_TypeDef;
/*
 * Data Structure for Power control, Address Range: 0x4000 7000 - 0x4000 73FF */
typedef struct
{
    uint32_t volatile CR;
    uint32_t volatile CSR;
} PWR_TypeDef;

/*
 * Data Structure for Reset and Clock Control Registers (RCC), Address Range: 0x4002 3800 - 0x4002 3BFF */
typedef struct
{
    uint32_t volatile CR;       /* Offset: 0x00 (R/W) Clock Control Register */
    uint32_t volatile PLLCFGR;  /* Offset: 0x04 (R/W) PLL Configuration Register */
    uint32_t volatile CFGR;     /* Offset: 0x08 (R/W) Clock Configuration Register */
    uint32_t volatile CIR;      /* Offset: 0x0C (R/W) Clock Interrupt Register */
    uint32_t volatile AHB1RSTR; /* Offset: 0x10 (R/W) AHB1 Peripheral Reset Register */
    uint32_t volatile AHB2RSTR; /* Offset: 0x14 (R/W) AHB2 Peripheral Reset Register */
    uint32_t volatile AHB3RSTR; /* Offset: 0x18 (R/W) AHB3 Peripheral Reset Register */
    uint32_t volatile reserved0;
    uint32_t volatile APB1RSTR; /* Offset: 0x20 (R/W) APB1 Peripheral Reset Register */
    uint32_t volatile APB2RSTR; /* Offset: 0x24 (R/W) APB2 Peripheral Reset Register */
    uint32_t reserved1[2];
    uint32_t volatile AHB1ENR; /* Offset: 0x30 (R/W) AHB1 Peripheral Clock Enable Register */
    uint32_t volatile AHB2ENR; /* Offset: 0x34 (R/W) AHB2 Peripheral Clock Enable Register */
    uint32_t volatile AHB3ENR; /* Offset: 0x38 (R/W) AHB3 Peripheral Clock Enable Register */
    uint32_t reserved2;
    uint32_t volatile APB1ENR; /* Offset: 0x40 (R/W) APB1 Peripheral Clock Enable Register */
    uint32_t volatile APB2ENR; /* Offset: 0x44 (R/W) APB1 Peripheral Clock Enable Register */
    uint32_t reserved3[2];
    uint32_t volatile AHB1LPENR; /* Offset: 0x50 (R/W) AHB1 Peripheral Clock Enable Lower Power Mode Register */
    uint32_t volatile AHB2LPENR; /* Offset: 0x54 (R/W) AHB2 Peripheral Clock Enable Lower Power Mode Register */
    uint32_t volatile AHB3LPENR; /* Offset: 0x58 (R/W) AHB3 Peripheral Clock Enable Lower Power Mode Register */
    uint32_t reserved4;
    uint32_t volatile APB1LPENR; /* Offset: 0x60 (R/W) APB1 Peripheral Clock Enable Lower Power Mode Register */
    uint32_t volatile APB2LPENR; /* Offset: 0x64 (R/W) APB2 Peripheral Clock Enable Lower Power Mode Register */
    uint32_t reserved5[2];
    uint32_t volatile BDCR; /* Offset: 0x70 (R/W) Backup Domain Control Register */
    uint32_t volatile CSR;  /* Offset: 0x74 (R/W) Clock Control & Status Register */
    uint32_t reserved6[2];
    uint32_t volatile SSCGR;      /* Offset: 0x80 (R/W) Spread Spectrum Clock Generation Register */
    uint32_t volatile PLLI2SCFGR; /* Offset: 0x84 (R/W) PLLI2S Configuration Register */
    uint32_t volatile PLLSAICFGR; /* Offset: 0x88 (R/W) PLLSAI Configuration Register */
    uint32_t volatile DCKCFGR;    /* Offset: 0x8C (R/W) Dedicated Clocks Configuration Register */
    uint32_t volatile CKGATENR;   /* Offset: 0x90 (R/W) Clocks Gated Enabled Register */
    uint32_t volatile DCKCFGR2;   /* Offset: 0x94 (R/W) Dedicated Clocks Configuration Register 2 */
} RCC_TypeDef;

typedef struct
{
    uint32_t volatile TR;       /*!< RTC time register,                                        Address offset: 0x00 */
    uint32_t volatile DR;       /*!< RTC date register,                                        Address offset: 0x04 */
    uint32_t volatile CR;       /*!< RTC control register,                                     Address offset: 0x08 */
    uint32_t volatile ISR;      /*!< RTC initialization and status register,                   Address offset: 0x0C */
    uint32_t volatile PRER;     /*!< RTC prescaler register,                                   Address offset: 0x10 */
    uint32_t volatile WUTR;     /*!< RTC wakeup timer register,                                Address offset: 0x14 */
    uint32_t volatile CALIBR;   /*!< RTC calibration register,                                 Address offset: 0x18 */
    uint32_t volatile ALRMAR;   /*!< RTC alarm A register,                                     Address offset: 0x1C */
    uint32_t volatile ALRMBR;   /*!< RTC alarm B register,                                     Address offset: 0x20 */
    uint32_t volatile WPR;      /*!< RTC write protection register,                            Address offset: 0x24 */
    uint32_t volatile SSR;      /*!< RTC sub second register,                                  Address offset: 0x28 */
    uint32_t volatile SHIFTR;   /*!< RTC shift control register,                               Address offset: 0x2C */
    uint32_t volatile TSTR;     /*!< RTC time stamp time register,                             Address offset: 0x30 */
    uint32_t volatile TSDR;     /*!< RTC time stamp date register,                             Address offset: 0x34 */
    uint32_t volatile TSSSR;    /*!< RTC time-stamp sub second register,                       Address offset: 0x38 */
    uint32_t volatile CALR;     /*!< RTC calibration register,                                 Address offset: 0x3C */
    uint32_t volatile TAFCR;    /*!< RTC tamper and alternate function configuration register, Address offset: 0x40 */
    uint32_t volatile ALRMASSR; /*!< RTC alarm A sub second register,                          Address offset: 0x44 */
    uint32_t volatile ALRMBSSR; /*!< RTC alarm B sub second register,                          Address offset: 0x48 */
    uint32_t reserved7;         /*!< Reserved, 0x4C                                                                 */
    uint32_t volatile BKP0R;    /*!< RTC backup register 1,                                    Address offset: 0x50 */
    uint32_t volatile BKP1R;    /*!< RTC backup register 1,                                    Address offset: 0x54 */
    uint32_t volatile BKP2R;    /*!< RTC backup register 2,                                    Address offset: 0x58 */
    uint32_t volatile BKP3R;    /*!< RTC backup register 3,                                    Address offset: 0x5C */
    uint32_t volatile BKP4R;    /*!< RTC backup register 4,                                    Address offset: 0x60 */
    uint32_t volatile BKP5R;    /*!< RTC backup register 5,                                    Address offset: 0x64 */
    uint32_t volatile BKP6R;    /*!< RTC backup register 6,                                    Address offset: 0x68 */
    uint32_t volatile BKP7R;    /*!< RTC backup register 7,                                    Address offset: 0x6C */
    uint32_t volatile BKP8R;    /*!< RTC backup register 8,                                    Address offset: 0x70 */
    uint32_t volatile BKP9R;    /*!< RTC backup register 9,                                    Address offset: 0x74 */
    uint32_t volatile BKP10R;   /*!< RTC backup register 10,                                   Address offset: 0x78 */
    uint32_t volatile BKP11R;   /*!< RTC backup register 11,                                   Address offset: 0x7C */
    uint32_t volatile BKP12R;   /*!< RTC backup register 12,                                   Address offset: 0x80 */
    uint32_t volatile BKP13R;   /*!< RTC backup register 13,                                   Address offset: 0x84 */
    uint32_t volatile BKP14R;   /*!< RTC backup register 14,                                   Address offset: 0x88 */
    uint32_t volatile BKP15R;   /*!< RTC backup register 15,                                   Address offset: 0x8C */
    uint32_t volatile BKP16R;   /*!< RTC backup register 16,                                   Address offset: 0x90 */
    uint32_t volatile BKP17R;   /*!< RTC backup register 17,                                   Address offset: 0x94 */
    uint32_t volatile BKP18R;   /*!< RTC backup register 18,                                   Address offset: 0x98 */
    uint32_t volatile BKP19R;   /*!< RTC backup register 19,                                   Address offset: 0x9C */
} RTC_TypeDef;

/*
 * Data Structure for GPIO port
 */
typedef struct
{
    uint32_t volatile MODER;   /* Offset: 0x00 (R/W) Mode Register */
    uint32_t volatile OTYPER;  /* Offset: 0x04 (R/W) Output Type Register */
    uint32_t volatile OSPEEDR; /* Offset: 0x08 (R/W) Output Speed Register */
    uint32_t volatile PUPDR;   /* Offset: 0x0C (R/W) Pull-up/Pull-down Register */
    uint32_t volatile IDR;     /* Offset: 0x10 (R/W) Input Data Register */
    uint32_t volatile ODR;     /* Offset: 0x14 (R/W) Output Data Register */
    uint32_t volatile BSRR;    /* Offset: 0x18 (R/W) Bit Set/Reset Register */
    uint32_t volatile LCKR;    /* Offset: 0x1C (R/W) Configuration Lock Register */
    uint32_t volatile AFRL;    /* Offset: 0x20 (R/W) Alternate Function Low Register */
    uint32_t volatile AFRH;    /* Offset: 0x24 (R/W) Alternate Function High Register */
} GPIO_TypeDef;

/*
 * Data Structure for USART
 */
typedef struct
{
    uint32_t volatile SR;   /* Offset: 0x00 (R) Status Register */
    uint32_t volatile DR;   /* Offset: 0x04 (R/W) Data Register */
    uint32_t volatile BRR;  /* Offset: 0x08 (R/W) Baud Rate Register */
    uint32_t volatile CR1;  /* Offset: 0x0C (R/W) Control Register 1 */
    uint32_t volatile CR2;  /* Offset: 0x10 (R/W) Control Register 2 */
    uint32_t volatile CR3;  /* Offset: 0x14 (R/W) Control Register 3 */
    uint32_t volatile GTPR; /* Offset: 0x18 (R/W) Guard time and prescaler register */
} USART_TypeDef;

/*
 * Data Structure for Timer 1 & 8
 * Alert Timer data structure is not 100% correct, see datasheet and reference manual for currection
 */
typedef struct
{
    uint32_t volatile CR1;   /* Offset: 0x00 (R/W) TIM1&TIM8 control register 1 */
    uint32_t volatile CR2;   /* Offset: 0x04 (R/W) TIM1&TIM8 control register 2 */
    uint32_t volatile SMCR;  /* Offset: 0x08 (R/W) TIM1&TIM8 slave mode control register */
    uint32_t volatile DIER;  /* Offset: 0x0C (R/W) TIM1&TIM8 DMA/interrupt enable register */
    uint32_t volatile SR;    /* Offset: 0x10 TIM1&TIM8 status register */
    uint32_t volatile EGR;   /* Offset: 0x14 (R/W) TIM1&TIM8 event generation register */
    uint32_t volatile CCMR1; /* Offset: 0x18 (R/W) TIM1&TIM8 capture/compare mode register 1 */
    uint32_t volatile CCMR2; /* Offset: 0x1C (R/W) TIM1&TIM8 capture/compare mode register 2 */
    uint32_t volatile CCER;  /* Offset: 0x20 (R/W) TIM1&TIM8 capture/compare enable register  */
    uint32_t volatile CNT;   /* Offset: 0x24 (R/W) TIM1&TIM8 counter */
    uint32_t volatile PSC;   /* Offset: 0x28 (R/W) TIM1&TIM8 prescaler */
    uint32_t volatile ARR;   /* Offset: 0x2C (R/W) TIM1&TIM8 auto-reload register */
    uint32_t volatile RCR;   /* Offset: 0x30 (R/W) TIM1&TIM8 repetition counter register */
    uint32_t volatile CCR1;  /* Offset: 0x34 (R/W) TIM1&TIM8 capture/compare register 1 */
    uint32_t volatile CCR2;  /* Offset: 0x38 (R/W) TIM1&TIM8 capture/compare register 2 */
    uint32_t volatile CCR3;  /* Offset: 0x3C (R/W) TIM1&TIM8 capture/compare register 3 */
    uint32_t volatile CCR4;  /* Offset: 0x40 (R/W) TIM1&TIM8 capture/compare register 4 */
    uint32_t volatile BDTR;  /* Offset: 0x44 (R/W) TIM1&TIM8 break and dead-time register */
    uint32_t volatile DCR;   /* Offset: 0x48 (R/W) TIM1&TIM8 DMA control register */
    uint32_t volatile DMAR;  /* Offset: 0x4C (R/W) TIM1&TIM8 DMA address for full transfer */
    uint32_t volatile OR;    /*!< TIM option register, Address offset: 0x50 */
} TIM_TypeDef;

typedef struct
{
    uint32_t volatile MEMRMP;    /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
    uint32_t volatile PMC;       /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
    uint32_t volatile EXTICR[4]; /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
    uint32_t RESERVED[2];        /*!< Reserved, 0x18-0x1C                                                          */
    uint32_t volatile CMPCR;     /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
    uint32_t RESERVED1[2];       /*!< Reserved, 0x24-0x28                                                          */
    uint32_t volatile CFGR;      /*!< SYSCFG Configuration register,                     Address offset: 0x2C      */
} SYSCFG_TypeDef;

typedef struct
{
    uint32_t volatile IMR;   /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
    uint32_t volatile EMR;   /*!< EXTI Event mask register,                Address offset: 0x04 */
    uint32_t volatile RTSR;  /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
    uint32_t volatile FTSR;  /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
    uint32_t volatile SWIER; /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
    uint32_t volatile PR;    /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_TypeDef;

/******************************************************************************/
/*                                                                            */
/*                                GPIO                                        */
/*                                                                            */
/******************************************************************************/

#define GPIO_PIN_0 ((uint16_t)0x0001)   /* Pin 0 selected    */
#define GPIO_PIN_1 ((uint16_t)0x0002)   /* Pin 1 selected    */
#define GPIO_PIN_2 ((uint16_t)0x0004)   /* Pin 2 selected    */
#define GPIO_PIN_3 ((uint16_t)0x0008)   /* Pin 3 selected    */
#define GPIO_PIN_4 ((uint16_t)0x0010)   /* Pin 4 selected    */
#define GPIO_PIN_5 ((uint16_t)0x0020)   /* Pin 5 selected    */
#define GPIO_PIN_6 ((uint16_t)0x0040)   /* Pin 6 selected    */
#define GPIO_PIN_7 ((uint16_t)0x0080)   /* Pin 7 selected    */
#define GPIO_PIN_8 ((uint16_t)0x0100)   /* Pin 8 selected    */
#define GPIO_PIN_9 ((uint16_t)0x0200)   /* Pin 9 selected    */
#define GPIO_PIN_10 ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11 ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12 ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13 ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14 ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15 ((uint16_t)0x8000)  /* Pin 15 selected   */
#define GPIO_PIN_All ((uint16_t)0xFFFF) /* All pins selected */
#define GPIO_PIN_MASK 0x0000FFFFU       /* PIN mask for assert test */

/* #define  GPIO_MODE_INPUT                        0x00000000U   !< Input Floating Mode                   */
/* #define  GPIO_MODE_OUTPUT_PP                    0xzU   			 !< Output Push Pull Mode                 */
/* #define  GPIO_MODE_OUTPUT_OD                    0x00000011U   !< Output Open Drain Mode                */
/* #define  GPIO_MODE_AF_PP                        0x00000002U   !< Alternate Function Push Pull Mode     */
/* #define  GPIO_MODE_AF_OD                        0x00000012U   !< Alternate Function Open Drain Mode    */

#define GPIO_MODE_INPUT 0U     /* 00 */
#define GPIO_MODE_OUTPUT 1U    /* 01 */
#define GPIO_MODE_ALTERNATE 2U /* 10 */
#define GPIO_MODE_ANALOG 3U    /* 11 */

/* #define  GPIO_MODE_ANALOG                       0x00000003U   !< Analog Mode  */

#define GPIO_MODE_IT_RISING 0x10110000U         /*!< External Interrupt Mode with Rising edge trigger detection          */
#define GPIO_MODE_IT_FALLING 0x10210000U        /*!< External Interrupt Mode with Falling edge trigger detection         */
#define GPIO_MODE_IT_RISING_FALLING 0x10310000U /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */

#define GPIO_MODE_EVT_RISING 0x10120000U         /*!< External Event Mode with Rising edge trigger detection               */
#define GPIO_MODE_EVT_FALLING 0x10220000U        /*!< External Event Mode with Falling edge trigger detection              */
#define GPIO_MODE_EVT_RISING_FALLING 0x10320000U /*!< External Event Mode with Rising/Falling edge trigger detection       */

#define GPIO_SPEED_FREQ_LOW 0x00000000U       /*!< IO works at 2 MHz, please refer to the product datasheet */
#define GPIO_SPEED_FREQ_MEDIUM 0x00000001U    /*!< range 12,5 MHz to 50 MHz, please refer to the product datasheet */
#define GPIO_SPEED_FREQ_HIGH 0x00000002U      /*!< range 25 MHz to 100 MHz, please refer to the product datasheet  */
#define GPIO_SPEED_FREQ_VERY_HIGH 0x00000003U /*!< range 50 MHz to 200 MHz, please refer to the product datasheet  */

#define GPIO_NOPULL 0x00000000U   /*!< No Pull-up or Pull-down activation  */
#define GPIO_PULLUP 0x00000001U   /*!< Pull-up activation                  */
#define GPIO_PULLDOWN 0x00000002U /*!< Pull-down activation                */

#define GPIO_MODE 0x00000003U
#define EXTI_MODE 0x10000000U
#define GPIO_MODE_IT 0x00010000U
#define GPIO_MODE_EVT 0x00020000U
#define RISING_EDGE 0x00100000U
#define FALLING_EDGE 0x00200000U
#define GPIO_OUTPUT_TYPE 0x00000010U

#define GPIO_NUMBER 16U

#define RCC_AHB1ENR_GPIOAEN_Pos (0U)
#define RCC_AHB1ENR_GPIOAEN_Msk (0x1UL << RCC_AHB1ENR_GPIOAEN_Pos) /*!< 0x00000001 */
#define RCC_AHB1ENR_GPIOAEN RCC_AHB1ENR_GPIOAEN_Msk

#define RCC_APB1ENR_UART4EN_Pos (19U)
#define RCC_APB1ENR_UART4EN_Msk (0x1UL << RCC_APB1ENR_UART4EN_Pos) /*!< 0x00080000 */
#define RCC_APB1ENR_UART4EN RCC_APB1ENR_UART4EN_Msk

#define USART_CR1_RXNEIE_Pos (5U)
#define USART_CR1_RXNEIE_Msk (0x1UL << USART_CR1_RXNEIE_Pos) /*!< 0x00000020 */
#define USART_CR1_RXNEIE USART_CR1_RXNEIE_Msk                /*!<RXNE Interrupt Enable                  */

#define USART_SR_RXNE_Pos (5U)
#define USART_SR_RXNE_Msk (0x1UL << USART_SR_RXNE_Pos) /*!< 0x00000020 */
#define USART_SR_RXNE USART_SR_RXNE_Msk                /*!<Read Data Register Not Empty */

#define USART_SR_TXE_Pos (7U)
#define USART_SR_TXE_Msk (0x1UL << USART_SR_TXE_Pos) /*!< 0x00000080 */
#define USART_SR_TXE USART_SR_TXE_Msk                /*!<Transmit Data Register Empty */

#define PERIPH_BASE 0x40000000UL /*!< Peripheral base address in the alias region                                */
#define APB1PERIPH_BASE PERIPH_BASE
#define UART4_BASE (APB1PERIPH_BASE + 0x4C00UL)
#define UART4 ((USART_TypeDef *)UART4_BASE)

/******************************************************************************/
/*                                                                            */
/*                         Controller Area Network                            */
/*                                                                            */
/******************************************************************************/

/*
 * Data Structure for CAN
 */

typedef struct
{
    uint32_t TIR;  /*!< CAN TX mailbox identifier register */
    uint32_t TDTR; /*!< CAN mailbox data length control and time stamp register */
    uint32_t TDLR; /*!< CAN mailbox data low register */
    uint32_t TDHR; /*!< CAN mailbox data high register */
} CAN_TxMailBox_TypeDef;

typedef struct
{
    uint32_t RIR;  /*!< CAN receive FIFO mailbox identifier register */
    uint32_t RDTR; /*!< CAN receive FIFO mailbox data length control and time stamp register */
    uint32_t RDLR; /*!< CAN receive FIFO mailbox data low register */
    uint32_t RDHR; /*!< CAN receive FIFO mailbox data high register */
} CAN_FIFOMailBox_TypeDef;

typedef struct
{
    uint32_t FR1; /*!< CAN Filter bank register 1 */
    uint32_t FR2; /*!< CAN Filter bank register 1 */
} CAN_FilterRegister_TypeDef;

typedef struct
{
    uint32_t MCR;                                   /*!< CAN master control register,         Address offset: 0x00          */
    uint32_t MSR;                                   /*!< CAN master status register,          Address offset: 0x04          */
    uint32_t TSR;                                   /*!< CAN transmit status register,        Address offset: 0x08          */
    uint32_t RF0R;                                  /*!< CAN receive FIFO 0 register,         Address offset: 0x0C          */
    uint32_t RF1R;                                  /*!< CAN receive FIFO 1 register,         Address offset: 0x10          */
    uint32_t IER;                                   /*!< CAN interrupt enable register,       Address offset: 0x14          */
    uint32_t ESR;                                   /*!< CAN error status register,           Address offset: 0x18          */
    uint32_t BTR;                                   /*!< CAN bit timing register,             Address offset: 0x1C          */
    uint32_t RESERVED0[88];                         /*!< Reserved, 0x020 - 0x17F                                            */
    CAN_TxMailBox_TypeDef sTxMailBox[3];            /*!< CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC */
    CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];        /*!< CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC */
    uint32_t RESERVED1[12];                         /*!< Reserved, 0x1D0 - 0x1FF                                            */
    uint32_t FMR;                                   /*!< CAN filter master register,          Address offset: 0x200         */
    uint32_t FM1R;                                  /*!< CAN filter mode register,            Address offset: 0x204         */
    uint32_t RESERVED2;                             /*!< Reserved, 0x208                                                    */
    uint32_t FS1R;                                  /*!< CAN filter scale register,           Address offset: 0x20C         */
    uint32_t RESERVED3;                             /*!< Reserved, 0x210                                                    */
    uint32_t FFA1R;                                 /*!< CAN filter FIFO assignment register, Address offset: 0x214         */
    uint32_t RESERVED4;                             /*!< Reserved, 0x218                                                    */
    uint32_t FA1R;                                  /*!< CAN filter activation register,      Address offset: 0x21C         */
    uint32_t RESERVED5[8];                          /*!< Reserved, 0x220-0x23F                                              */
    CAN_FilterRegister_TypeDef sFilterRegister[28]; /*!< CAN Filter Register,                 Address offset: 0x240-0x31C   */
} CAN_TypeDef;

/*!<CAN control and status registers */
#define PERIPH_BASE 0x40000000UL /*!< Peripheral base address in the alias region                                */
#define APB1PERIPH_BASE PERIPH_BASE
#define CAN1_BASE (APB1PERIPH_BASE + 0x6400UL)
#define CAN2_BASE (APB1PERIPH_BASE + 0x6800UL)
#define CAN1 ((CAN_TypeDef *)CAN1_BASE)
#define CAN2 ((CAN_TypeDef *)CAN2_BASE)

/*******************  Bit definition for CAN_MCR register  ********************/
#define CAN_MCR_INRQ_Pos (0U)
#define CAN_MCR_INRQ_Msk (0x1UL << CAN_MCR_INRQ_Pos) /*!< 0x00000001 */
#define CAN_MCR_INRQ CAN_MCR_INRQ_Msk                /*!<Initialization Request */
#define CAN_MCR_SLEEP_Pos (1U)
#define CAN_MCR_SLEEP_Msk (0x1UL << CAN_MCR_SLEEP_Pos) /*!< 0x00000002 */
#define CAN_MCR_SLEEP CAN_MCR_SLEEP_Msk                /*!<Sleep Mode Request */
#define CAN_MCR_TXFP_Pos (2U)
#define CAN_MCR_TXFP_Msk (0x1UL << CAN_MCR_TXFP_Pos) /*!< 0x00000004 */
#define CAN_MCR_TXFP CAN_MCR_TXFP_Msk                /*!<Transmit FIFO Priority */
#define CAN_MCR_RFLM_Pos (3U)
#define CAN_MCR_RFLM_Msk (0x1UL << CAN_MCR_RFLM_Pos) /*!< 0x00000008 */
#define CAN_MCR_RFLM CAN_MCR_RFLM_Msk                /*!<Receive FIFO Locked Mode */
#define CAN_MCR_NART_Pos (4U)
#define CAN_MCR_NART_Msk (0x1UL << CAN_MCR_NART_Pos) /*!< 0x00000010 */
#define CAN_MCR_NART CAN_MCR_NART_Msk                /*!<No Automatic Retransmission */
#define CAN_MCR_AWUM_Pos (5U)
#define CAN_MCR_AWUM_Msk (0x1UL << CAN_MCR_AWUM_Pos) /*!< 0x00000020 */
#define CAN_MCR_AWUM CAN_MCR_AWUM_Msk                /*!<Automatic Wakeup Mode */
#define CAN_MCR_ABOM_Pos (6U)
#define CAN_MCR_ABOM_Msk (0x1UL << CAN_MCR_ABOM_Pos) /*!< 0x00000040 */
#define CAN_MCR_ABOM CAN_MCR_ABOM_Msk                /*!<Automatic Bus-Off Management */
#define CAN_MCR_TTCM_Pos (7U)
#define CAN_MCR_TTCM_Msk (0x1UL << CAN_MCR_TTCM_Pos) /*!< 0x00000080 */
#define CAN_MCR_TTCM CAN_MCR_TTCM_Msk                /*!<Time Triggered Communication Mode */
#define CAN_MCR_RESET_Pos (15U)
#define CAN_MCR_RESET_Msk (0x1UL << CAN_MCR_RESET_Pos) /*!< 0x00008000 */
#define CAN_MCR_RESET CAN_MCR_RESET_Msk                /*!<bxCAN software master reset */
#define CAN_MCR_DBF_Pos (16U)
#define CAN_MCR_DBF_Msk (0x1UL << CAN_MCR_DBF_Pos) /*!< 0x00010000 */
#define CAN_MCR_DBF CAN_MCR_DBF_Msk                /*!<bxCAN Debug freeze */
/*******************  Bit definition for CAN_MSR register  ********************/
#define CAN_MSR_INAK_Pos (0U)
#define CAN_MSR_INAK_Msk (0x1UL << CAN_MSR_INAK_Pos) /*!< 0x00000001 */
#define CAN_MSR_INAK CAN_MSR_INAK_Msk                /*!<Initialization Acknowledge */
#define CAN_MSR_SLAK_Pos (1U)
#define CAN_MSR_SLAK_Msk (0x1UL << CAN_MSR_SLAK_Pos) /*!< 0x00000002 */
#define CAN_MSR_SLAK CAN_MSR_SLAK_Msk                /*!<Sleep Acknowledge */
#define CAN_MSR_ERRI_Pos (2U)
#define CAN_MSR_ERRI_Msk (0x1UL << CAN_MSR_ERRI_Pos) /*!< 0x00000004 */
#define CAN_MSR_ERRI CAN_MSR_ERRI_Msk                /*!<Error Interrupt */
#define CAN_MSR_WKUI_Pos (3U)
#define CAN_MSR_WKUI_Msk (0x1UL << CAN_MSR_WKUI_Pos) /*!< 0x00000008 */
#define CAN_MSR_WKUI CAN_MSR_WKUI_Msk                /*!<Wakeup Interrupt */
#define CAN_MSR_SLAKI_Pos (4U)
#define CAN_MSR_SLAKI_Msk (0x1UL << CAN_MSR_SLAKI_Pos) /*!< 0x00000010 */
#define CAN_MSR_SLAKI CAN_MSR_SLAKI_Msk                /*!<Sleep Acknowledge Interrupt */
#define CAN_MSR_TXM_Pos (8U)
#define CAN_MSR_TXM_Msk (0x1UL << CAN_MSR_TXM_Pos) /*!< 0x00000100 */
#define CAN_MSR_TXM CAN_MSR_TXM_Msk                /*!<Transmit Mode */
#define CAN_MSR_RXM_Pos (9U)
#define CAN_MSR_RXM_Msk (0x1UL << CAN_MSR_RXM_Pos) /*!< 0x00000200 */
#define CAN_MSR_RXM CAN_MSR_RXM_Msk                /*!<Receive Mode */
#define CAN_MSR_SAMP_Pos (10U)
#define CAN_MSR_SAMP_Msk (0x1UL << CAN_MSR_SAMP_Pos) /*!< 0x00000400 */
#define CAN_MSR_SAMP CAN_MSR_SAMP_Msk                /*!<Last Sample Point */
#define CAN_MSR_RX_Pos (11U)
#define CAN_MSR_RX_Msk (0x1UL << CAN_MSR_RX_Pos) /*!< 0x00000800 */
#define CAN_MSR_RX CAN_MSR_RX_Msk                /*!<CAN Rx Signal */

/*******************  Bit definition for CAN_TSR register  ********************/
#define CAN_TSR_RQCP0_Pos (0U)
#define CAN_TSR_RQCP0_Msk (0x1UL << CAN_TSR_RQCP0_Pos) /*!< 0x00000001 */
#define CAN_TSR_RQCP0 CAN_TSR_RQCP0_Msk                /*!<Request Completed Mailbox0 */
#define CAN_TSR_TXOK0_Pos (1U)
#define CAN_TSR_TXOK0_Msk (0x1UL << CAN_TSR_TXOK0_Pos) /*!< 0x00000002 */
#define CAN_TSR_TXOK0 CAN_TSR_TXOK0_Msk                /*!<Transmission OK of Mailbox0 */
#define CAN_TSR_ALST0_Pos (2U)
#define CAN_TSR_ALST0_Msk (0x1UL << CAN_TSR_ALST0_Pos) /*!< 0x00000004 */
#define CAN_TSR_ALST0 CAN_TSR_ALST0_Msk                /*!<Arbitration Lost for Mailbox0 */
#define CAN_TSR_TERR0_Pos (3U)
#define CAN_TSR_TERR0_Msk (0x1UL << CAN_TSR_TERR0_Pos) /*!< 0x00000008 */
#define CAN_TSR_TERR0 CAN_TSR_TERR0_Msk                /*!<Transmission Error of Mailbox0 */
#define CAN_TSR_ABRQ0_Pos (7U)
#define CAN_TSR_ABRQ0_Msk (0x1UL << CAN_TSR_ABRQ0_Pos) /*!< 0x00000080 */
#define CAN_TSR_ABRQ0 CAN_TSR_ABRQ0_Msk                /*!<Abort Request for Mailbox0 */
#define CAN_TSR_RQCP1_Pos (8U)
#define CAN_TSR_RQCP1_Msk (0x1UL << CAN_TSR_RQCP1_Pos) /*!< 0x00000100 */
#define CAN_TSR_RQCP1 CAN_TSR_RQCP1_Msk                /*!<Request Completed Mailbox1 */
#define CAN_TSR_TXOK1_Pos (9U)
#define CAN_TSR_TXOK1_Msk (0x1UL << CAN_TSR_TXOK1_Pos) /*!< 0x00000200 */
#define CAN_TSR_TXOK1 CAN_TSR_TXOK1_Msk                /*!<Transmission OK of Mailbox1 */
#define CAN_TSR_ALST1_Pos (10U)
#define CAN_TSR_ALST1_Msk (0x1UL << CAN_TSR_ALST1_Pos) /*!< 0x00000400 */
#define CAN_TSR_ALST1 CAN_TSR_ALST1_Msk                /*!<Arbitration Lost for Mailbox1 */
#define CAN_TSR_TERR1_Pos (11U)
#define CAN_TSR_TERR1_Msk (0x1UL << CAN_TSR_TERR1_Pos) /*!< 0x00000800 */
#define CAN_TSR_TERR1 CAN_TSR_TERR1_Msk                /*!<Transmission Error of Mailbox1 */
#define CAN_TSR_ABRQ1_Pos (15U)
#define CAN_TSR_ABRQ1_Msk (0x1UL << CAN_TSR_ABRQ1_Pos) /*!< 0x00008000 */
#define CAN_TSR_ABRQ1 CAN_TSR_ABRQ1_Msk                /*!<Abort Request for Mailbox 1 */
#define CAN_TSR_RQCP2_Pos (16U)
#define CAN_TSR_RQCP2_Msk (0x1UL << CAN_TSR_RQCP2_Pos) /*!< 0x00010000 */
#define CAN_TSR_RQCP2 CAN_TSR_RQCP2_Msk                /*!<Request Completed Mailbox2 */
#define CAN_TSR_TXOK2_Pos (17U)
#define CAN_TSR_TXOK2_Msk (0x1UL << CAN_TSR_TXOK2_Pos) /*!< 0x00020000 */
#define CAN_TSR_TXOK2 CAN_TSR_TXOK2_Msk                /*!<Transmission OK of Mailbox 2 */
#define CAN_TSR_ALST2_Pos (18U)
#define CAN_TSR_ALST2_Msk (0x1UL << CAN_TSR_ALST2_Pos) /*!< 0x00040000 */
#define CAN_TSR_ALST2 CAN_TSR_ALST2_Msk                /*!<Arbitration Lost for mailbox 2 */
#define CAN_TSR_TERR2_Pos (19U)
#define CAN_TSR_TERR2_Msk (0x1UL << CAN_TSR_TERR2_Pos) /*!< 0x00080000 */
#define CAN_TSR_TERR2 CAN_TSR_TERR2_Msk                /*!<Transmission Error of Mailbox 2 */
#define CAN_TSR_ABRQ2_Pos (23U)
#define CAN_TSR_ABRQ2_Msk (0x1UL << CAN_TSR_ABRQ2_Pos) /*!< 0x00800000 */
#define CAN_TSR_ABRQ2 CAN_TSR_ABRQ2_Msk                /*!<Abort Request for Mailbox 2 */
#define CAN_TSR_CODE_Pos (24U)
#define CAN_TSR_CODE_Msk (0x3UL << CAN_TSR_CODE_Pos) /*!< 0x03000000 */
#define CAN_TSR_CODE CAN_TSR_CODE_Msk                /*!<Mailbox Code */

#define CAN_TSR_TME_Pos (26U)
#define CAN_TSR_TME_Msk (0x7UL << CAN_TSR_TME_Pos) /*!< 0x1C000000 */
#define CAN_TSR_TME CAN_TSR_TME_Msk                /*!<TME[2:0] bits */
#define CAN_TSR_TME0_Pos (26U)
#define CAN_TSR_TME0_Msk (0x1UL << CAN_TSR_TME0_Pos) /*!< 0x04000000 */
#define CAN_TSR_TME0 CAN_TSR_TME0_Msk                /*!<Transmit Mailbox 0 Empty */
#define CAN_TSR_TME1_Pos (27U)
#define CAN_TSR_TME1_Msk (0x1UL << CAN_TSR_TME1_Pos) /*!< 0x08000000 */
#define CAN_TSR_TME1 CAN_TSR_TME1_Msk                /*!<Transmit Mailbox 1 Empty */
#define CAN_TSR_TME2_Pos (28U)
#define CAN_TSR_TME2_Msk (0x1UL << CAN_TSR_TME2_Pos) /*!< 0x10000000 */
#define CAN_TSR_TME2 CAN_TSR_TME2_Msk                /*!<Transmit Mailbox 2 Empty */

#define CAN_TSR_LOW_Pos (29U)
#define CAN_TSR_LOW_Msk (0x7UL << CAN_TSR_LOW_Pos) /*!< 0xE0000000 */
#define CAN_TSR_LOW CAN_TSR_LOW_Msk                /*!<LOW[2:0] bits */
#define CAN_TSR_LOW0_Pos (29U)
#define CAN_TSR_LOW0_Msk (0x1UL << CAN_TSR_LOW0_Pos) /*!< 0x20000000 */
#define CAN_TSR_LOW0 CAN_TSR_LOW0_Msk                /*!<Lowest Priority Flag for Mailbox 0 */
#define CAN_TSR_LOW1_Pos (30U)
#define CAN_TSR_LOW1_Msk (0x1UL << CAN_TSR_LOW1_Pos) /*!< 0x40000000 */
#define CAN_TSR_LOW1 CAN_TSR_LOW1_Msk                /*!<Lowest Priority Flag for Mailbox 1 */
#define CAN_TSR_LOW2_Pos (31U)
#define CAN_TSR_LOW2_Msk (0x1UL << CAN_TSR_LOW2_Pos) /*!< 0x80000000 */
#define CAN_TSR_LOW2 CAN_TSR_LOW2_Msk                /*!<Lowest Priority Flag for Mailbox 2 */

/*******************  Bit definition for CAN_RF0R register  *******************/
#define CAN_RF0R_FMP0_Pos (0U)
#define CAN_RF0R_FMP0_Msk (0x3UL << CAN_RF0R_FMP0_Pos) /*!< 0x00000003 */
#define CAN_RF0R_FMP0 CAN_RF0R_FMP0_Msk                /*!<FIFO 0 Message Pending */
#define CAN_RF0R_FULL0_Pos (3U)
#define CAN_RF0R_FULL0_Msk (0x1UL << CAN_RF0R_FULL0_Pos) /*!< 0x00000008 */
#define CAN_RF0R_FULL0 CAN_RF0R_FULL0_Msk                /*!<FIFO 0 Full */
#define CAN_RF0R_FOVR0_Pos (4U)
#define CAN_RF0R_FOVR0_Msk (0x1UL << CAN_RF0R_FOVR0_Pos) /*!< 0x00000010 */
#define CAN_RF0R_FOVR0 CAN_RF0R_FOVR0_Msk                /*!<FIFO 0 Overrun */
#define CAN_RF0R_RFOM0_Pos (5U)
#define CAN_RF0R_RFOM0_Msk (0x1UL << CAN_RF0R_RFOM0_Pos) /*!< 0x00000020 */
#define CAN_RF0R_RFOM0 CAN_RF0R_RFOM0_Msk                /*!<Release FIFO 0 Output Mailbox */

/*******************  Bit definition for CAN_RF1R register  *******************/
#define CAN_RF1R_FMP1_Pos (0U)
#define CAN_RF1R_FMP1_Msk (0x3UL << CAN_RF1R_FMP1_Pos) /*!< 0x00000003 */
#define CAN_RF1R_FMP1 CAN_RF1R_FMP1_Msk                /*!<FIFO 1 Message Pending */
#define CAN_RF1R_FULL1_Pos (3U)
#define CAN_RF1R_FULL1_Msk (0x1UL << CAN_RF1R_FULL1_Pos) /*!< 0x00000008 */
#define CAN_RF1R_FULL1 CAN_RF1R_FULL1_Msk                /*!<FIFO 1 Full */
#define CAN_RF1R_FOVR1_Pos (4U)
#define CAN_RF1R_FOVR1_Msk (0x1UL << CAN_RF1R_FOVR1_Pos) /*!< 0x00000010 */
#define CAN_RF1R_FOVR1 CAN_RF1R_FOVR1_Msk                /*!<FIFO 1 Overrun */
#define CAN_RF1R_RFOM1_Pos (5U)
#define CAN_RF1R_RFOM1_Msk (0x1UL << CAN_RF1R_RFOM1_Pos) /*!< 0x00000020 */
#define CAN_RF1R_RFOM1 CAN_RF1R_RFOM1_Msk                /*!<Release FIFO 1 Output Mailbox */

/********************  Bit definition for CAN_IER register  *******************/
#define CAN_IER_TMEIE_Pos (0U)
#define CAN_IER_TMEIE_Msk (0x1UL << CAN_IER_TMEIE_Pos) /*!< 0x00000001 */
#define CAN_IER_TMEIE CAN_IER_TMEIE_Msk                /*!<Transmit Mailbox Empty Interrupt Enable */
#define CAN_IER_FMPIE0_Pos (1U)
#define CAN_IER_FMPIE0_Msk (0x1UL << CAN_IER_FMPIE0_Pos) /*!< 0x00000002 */
#define CAN_IER_FMPIE0 CAN_IER_FMPIE0_Msk                /*!<FIFO Message Pending Interrupt Enable */
#define CAN_IER_FFIE0_Pos (2U)
#define CAN_IER_FFIE0_Msk (0x1UL << CAN_IER_FFIE0_Pos) /*!< 0x00000004 */
#define CAN_IER_FFIE0 CAN_IER_FFIE0_Msk                /*!<FIFO Full Interrupt Enable */
#define CAN_IER_FOVIE0_Pos (3U)
#define CAN_IER_FOVIE0_Msk (0x1UL << CAN_IER_FOVIE0_Pos) /*!< 0x00000008 */
#define CAN_IER_FOVIE0 CAN_IER_FOVIE0_Msk                /*!<FIFO Overrun Interrupt Enable */
#define CAN_IER_FMPIE1_Pos (4U)
#define CAN_IER_FMPIE1_Msk (0x1UL << CAN_IER_FMPIE1_Pos) /*!< 0x00000010 */
#define CAN_IER_FMPIE1 CAN_IER_FMPIE1_Msk                /*!<FIFO Message Pending Interrupt Enable */
#define CAN_IER_FFIE1_Pos (5U)
#define CAN_IER_FFIE1_Msk (0x1UL << CAN_IER_FFIE1_Pos) /*!< 0x00000020 */
#define CAN_IER_FFIE1 CAN_IER_FFIE1_Msk                /*!<FIFO Full Interrupt Enable */
#define CAN_IER_FOVIE1_Pos (6U)
#define CAN_IER_FOVIE1_Msk (0x1UL << CAN_IER_FOVIE1_Pos) /*!< 0x00000040 */
#define CAN_IER_FOVIE1 CAN_IER_FOVIE1_Msk                /*!<FIFO Overrun Interrupt Enable */
#define CAN_IER_EWGIE_Pos (8U)
#define CAN_IER_EWGIE_Msk (0x1UL << CAN_IER_EWGIE_Pos) /*!< 0x00000100 */
#define CAN_IER_EWGIE CAN_IER_EWGIE_Msk                /*!<Error Warning Interrupt Enable */
#define CAN_IER_EPVIE_Pos (9U)
#define CAN_IER_EPVIE_Msk (0x1UL << CAN_IER_EPVIE_Pos) /*!< 0x00000200 */
#define CAN_IER_EPVIE CAN_IER_EPVIE_Msk                /*!<Error Passive Interrupt Enable */
#define CAN_IER_BOFIE_Pos (10U)
#define CAN_IER_BOFIE_Msk (0x1UL << CAN_IER_BOFIE_Pos) /*!< 0x00000400 */
#define CAN_IER_BOFIE CAN_IER_BOFIE_Msk                /*!<Bus-Off Interrupt Enable */
#define CAN_IER_LECIE_Pos (11U)
#define CAN_IER_LECIE_Msk (0x1UL << CAN_IER_LECIE_Pos) /*!< 0x00000800 */
#define CAN_IER_LECIE CAN_IER_LECIE_Msk                /*!<Last Error Code Interrupt Enable */
#define CAN_IER_ERRIE_Pos (15U)
#define CAN_IER_ERRIE_Msk (0x1UL << CAN_IER_ERRIE_Pos) /*!< 0x00008000 */
#define CAN_IER_ERRIE CAN_IER_ERRIE_Msk                /*!<Error Interrupt Enable */
#define CAN_IER_WKUIE_Pos (16U)
#define CAN_IER_WKUIE_Msk (0x1UL << CAN_IER_WKUIE_Pos) /*!< 0x00010000 */
#define CAN_IER_WKUIE CAN_IER_WKUIE_Msk                /*!<Wakeup Interrupt Enable */
#define CAN_IER_SLKIE_Pos (17U)
#define CAN_IER_SLKIE_Msk (0x1UL << CAN_IER_SLKIE_Pos) /*!< 0x00020000 */
#define CAN_IER_SLKIE CAN_IER_SLKIE_Msk                /*!<Sleep Interrupt Enable */
#define CAN_IER_EWGIE_Pos (8U)

/********************  Bit definition for CAN_ESR register  *******************/
#define CAN_ESR_EWGF_Pos (0U)
#define CAN_ESR_EWGF_Msk (0x1UL << CAN_ESR_EWGF_Pos) /*!< 0x00000001 */
#define CAN_ESR_EWGF CAN_ESR_EWGF_Msk                /*!<Error Warning Flag */
#define CAN_ESR_EPVF_Pos (1U)
#define CAN_ESR_EPVF_Msk (0x1UL << CAN_ESR_EPVF_Pos) /*!< 0x00000002 */
#define CAN_ESR_EPVF CAN_ESR_EPVF_Msk                /*!<Error Passive Flag */
#define CAN_ESR_BOFF_Pos (2U)
#define CAN_ESR_BOFF_Msk (0x1UL << CAN_ESR_BOFF_Pos) /*!< 0x00000004 */
#define CAN_ESR_BOFF CAN_ESR_BOFF_Msk                /*!<Bus-Off Flag */

#define CAN_ESR_LEC_Pos (4U)
#define CAN_ESR_LEC_Msk (0x7UL << CAN_ESR_LEC_Pos) /*!< 0x00000070 */
#define CAN_ESR_LEC CAN_ESR_LEC_Msk                /*!<LEC[2:0] bits (Last Error Code) */
#define CAN_ESR_LEC_0 (0x1UL << CAN_ESR_LEC_Pos)   /*!< 0x00000010 */
#define CAN_ESR_LEC_1 (0x2UL << CAN_ESR_LEC_Pos)   /*!< 0x00000020 */
#define CAN_ESR_LEC_2 (0x4UL << CAN_ESR_LEC_Pos)   /*!< 0x00000040 */

#define CAN_ESR_TEC_Pos (16U)
#define CAN_ESR_TEC_Msk (0xFFUL << CAN_ESR_TEC_Pos) /*!< 0x00FF0000 */
#define CAN_ESR_TEC CAN_ESR_TEC_Msk                 /*!<Least significant byte of the 9-bit Transmit Error Counter */
#define CAN_ESR_REC_Pos (24U)
#define CAN_ESR_REC_Msk (0xFFUL << CAN_ESR_REC_Pos) /*!< 0xFF000000 */
#define CAN_ESR_REC CAN_ESR_REC_Msk                 /*!<Receive Error Counter */

/*******************  Bit definition for CAN_BTR register  ********************/
#define CAN_BTR_BRP_Pos (0U)
#define CAN_BTR_BRP_Msk (0x3FFUL << CAN_BTR_BRP_Pos) /*!< 0x000003FF */
#define CAN_BTR_BRP CAN_BTR_BRP_Msk                  /*!<Baud Rate Prescaler */
#define CAN_BTR_TS1_Pos (16U)
#define CAN_BTR_TS1_Msk (0xFUL << CAN_BTR_TS1_Pos) /*!< 0x000F0000 */
#define CAN_BTR_TS1 CAN_BTR_TS1_Msk                /*!<Time Segment 1 */
#define CAN_BTR_TS1_0 (0x1UL << CAN_BTR_TS1_Pos)   /*!< 0x00010000 */
#define CAN_BTR_TS1_1 (0x2UL << CAN_BTR_TS1_Pos)   /*!< 0x00020000 */
#define CAN_BTR_TS1_2 (0x4UL << CAN_BTR_TS1_Pos)   /*!< 0x00040000 */
#define CAN_BTR_TS1_3 (0x8UL << CAN_BTR_TS1_Pos)   /*!< 0x00080000 */
#define CAN_BTR_TS2_Pos (20U)
#define CAN_BTR_TS2_Msk (0x7UL << CAN_BTR_TS2_Pos) /*!< 0x00700000 */
#define CAN_BTR_TS2 CAN_BTR_TS2_Msk                /*!<Time Segment 2 */
#define CAN_BTR_TS2_0 (0x1UL << CAN_BTR_TS2_Pos)   /*!< 0x00100000 */
#define CAN_BTR_TS2_1 (0x2UL << CAN_BTR_TS2_Pos)   /*!< 0x00200000 */
#define CAN_BTR_TS2_2 (0x4UL << CAN_BTR_TS2_Pos)   /*!< 0x00400000 */
#define CAN_BTR_SJW_Pos (24U)
#define CAN_BTR_SJW_Msk (0x3UL << CAN_BTR_SJW_Pos) /*!< 0x03000000 */
#define CAN_BTR_SJW CAN_BTR_SJW_Msk                /*!<Resynchronization Jump Width */
#define CAN_BTR_SJW_0 (0x1UL << CAN_BTR_SJW_Pos)   /*!< 0x01000000 */
#define CAN_BTR_SJW_1 (0x2UL << CAN_BTR_SJW_Pos)   /*!< 0x02000000 */
#define CAN_BTR_LBKM_Pos (30U)
#define CAN_BTR_LBKM_Msk (0x1UL << CAN_BTR_LBKM_Pos) /*!< 0x40000000 */
#define CAN_BTR_LBKM CAN_BTR_LBKM_Msk                /*!<Loop Back Mode (Debug) */
#define CAN_BTR_SILM_Pos (31U)
#define CAN_BTR_SILM_Msk (0x1UL << CAN_BTR_SILM_Pos) /*!< 0x80000000 */
#define CAN_BTR_SILM CAN_BTR_SILM_Msk                /*!<Silent Mode */

/*!<Mailbox registers */
/******************  Bit definition for CAN_TI0R register  ********************/
#define CAN_TI0R_TXRQ_Pos (0U)
#define CAN_TI0R_TXRQ_Msk (0x1UL << CAN_TI0R_TXRQ_Pos) /*!< 0x00000001 */
#define CAN_TI0R_TXRQ CAN_TI0R_TXRQ_Msk                /*!<Transmit Mailbox Request */
#define CAN_TI0R_RTR_Pos (1U)
#define CAN_TI0R_RTR_Msk (0x1UL << CAN_TI0R_RTR_Pos) /*!< 0x00000002 */
#define CAN_TI0R_RTR CAN_TI0R_RTR_Msk                /*!<Remote Transmission Request */
#define CAN_TI0R_IDE_Pos (2U)
#define CAN_TI0R_IDE_Msk (0x1UL << CAN_TI0R_IDE_Pos) /*!< 0x00000004 */
#define CAN_TI0R_IDE CAN_TI0R_IDE_Msk                /*!<Identifier Extension */
#define CAN_TI0R_EXID_Pos (3U)
#define CAN_TI0R_EXID_Msk (0x3FFFFUL << CAN_TI0R_EXID_Pos) /*!< 0x001FFFF8 */
#define CAN_TI0R_EXID CAN_TI0R_EXID_Msk                    /*!<Extended Identifier */
#define CAN_TI0R_STID_Pos (21U)
#define CAN_TI0R_STID_Msk (0x7FFUL << CAN_TI0R_STID_Pos) /*!< 0xFFE00000 */
#define CAN_TI0R_STID CAN_TI0R_STID_Msk                  /*!<Standard Identifier or Extended Identifier */

/******************  Bit definition for CAN_TDT0R register  *******************/
#define CAN_TDT0R_DLC_Pos (0U)
#define CAN_TDT0R_DLC_Msk (0xFUL << CAN_TDT0R_DLC_Pos) /*!< 0x0000000F */
#define CAN_TDT0R_DLC CAN_TDT0R_DLC_Msk                /*!<Data Length Code */
#define CAN_TDT0R_TGT_Pos (8U)
#define CAN_TDT0R_TGT_Msk (0x1UL << CAN_TDT0R_TGT_Pos) /*!< 0x00000100 */
#define CAN_TDT0R_TGT CAN_TDT0R_TGT_Msk                /*!<Transmit Global Time */
#define CAN_TDT0R_TIME_Pos (16U)
#define CAN_TDT0R_TIME_Msk (0xFFFFUL << CAN_TDT0R_TIME_Pos) /*!< 0xFFFF0000 */
#define CAN_TDT0R_TIME CAN_TDT0R_TIME_Msk                   /*!<Message Time Stamp */

/******************  Bit definition for CAN_TDL0R register  *******************/
#define CAN_TDL0R_DATA0_Pos (0U)
#define CAN_TDL0R_DATA0_Msk (0xFFUL << CAN_TDL0R_DATA0_Pos) /*!< 0x000000FF */
#define CAN_TDL0R_DATA0 CAN_TDL0R_DATA0_Msk                 /*!<Data byte 0 */
#define CAN_TDL0R_DATA1_Pos (8U)
#define CAN_TDL0R_DATA1_Msk (0xFFUL << CAN_TDL0R_DATA1_Pos) /*!< 0x0000FF00 */
#define CAN_TDL0R_DATA1 CAN_TDL0R_DATA1_Msk                 /*!<Data byte 1 */
#define CAN_TDL0R_DATA2_Pos (16U)
#define CAN_TDL0R_DATA2_Msk (0xFFUL << CAN_TDL0R_DATA2_Pos) /*!< 0x00FF0000 */
#define CAN_TDL0R_DATA2 CAN_TDL0R_DATA2_Msk                 /*!<Data byte 2 */
#define CAN_TDL0R_DATA3_Pos (24U)
#define CAN_TDL0R_DATA3_Msk (0xFFUL << CAN_TDL0R_DATA3_Pos) /*!< 0xFF000000 */
#define CAN_TDL0R_DATA3 CAN_TDL0R_DATA3_Msk                 /*!<Data byte 3 */

/******************  Bit definition for CAN_TDH0R register  *******************/
#define CAN_TDH0R_DATA4_Pos (0U)
#define CAN_TDH0R_DATA4_Msk (0xFFUL << CAN_TDH0R_DATA4_Pos) /*!< 0x000000FF */
#define CAN_TDH0R_DATA4 CAN_TDH0R_DATA4_Msk                 /*!<Data byte 4 */
#define CAN_TDH0R_DATA5_Pos (8U)
#define CAN_TDH0R_DATA5_Msk (0xFFUL << CAN_TDH0R_DATA5_Pos) /*!< 0x0000FF00 */
#define CAN_TDH0R_DATA5 CAN_TDH0R_DATA5_Msk                 /*!<Data byte 5 */
#define CAN_TDH0R_DATA6_Pos (16U)
#define CAN_TDH0R_DATA6_Msk (0xFFUL << CAN_TDH0R_DATA6_Pos) /*!< 0x00FF0000 */
#define CAN_TDH0R_DATA6 CAN_TDH0R_DATA6_Msk                 /*!<Data byte 6 */
#define CAN_TDH0R_DATA7_Pos (24U)
#define CAN_TDH0R_DATA7_Msk (0xFFUL << CAN_TDH0R_DATA7_Pos) /*!< 0xFF000000 */
#define CAN_TDH0R_DATA7 CAN_TDH0R_DATA7_Msk                 /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI1R register  *******************/
#define CAN_TI1R_TXRQ_Pos (0U)
#define CAN_TI1R_TXRQ_Msk (0x1UL << CAN_TI1R_TXRQ_Pos) /*!< 0x00000001 */
#define CAN_TI1R_TXRQ CAN_TI1R_TXRQ_Msk                /*!<Transmit Mailbox Request */
#define CAN_TI1R_RTR_Pos (1U)
#define CAN_TI1R_RTR_Msk (0x1UL << CAN_TI1R_RTR_Pos) /*!< 0x00000002 */
#define CAN_TI1R_RTR CAN_TI1R_RTR_Msk                /*!<Remote Transmission Request */
#define CAN_TI1R_IDE_Pos (2U)
#define CAN_TI1R_IDE_Msk (0x1UL << CAN_TI1R_IDE_Pos) /*!< 0x00000004 */
#define CAN_TI1R_IDE CAN_TI1R_IDE_Msk                /*!<Identifier Extension */
#define CAN_TI1R_EXID_Pos (3U)
#define CAN_TI1R_EXID_Msk (0x3FFFFUL << CAN_TI1R_EXID_Pos) /*!< 0x001FFFF8 */
#define CAN_TI1R_EXID CAN_TI1R_EXID_Msk                    /*!<Extended Identifier */
#define CAN_TI1R_STID_Pos (21U)
#define CAN_TI1R_STID_Msk (0x7FFUL << CAN_TI1R_STID_Pos) /*!< 0xFFE00000 */
#define CAN_TI1R_STID CAN_TI1R_STID_Msk                  /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT1R register  ******************/
#define CAN_TDT1R_DLC_Pos (0U)
#define CAN_TDT1R_DLC_Msk (0xFUL << CAN_TDT1R_DLC_Pos) /*!< 0x0000000F */
#define CAN_TDT1R_DLC CAN_TDT1R_DLC_Msk                /*!<Data Length Code */
#define CAN_TDT1R_TGT_Pos (8U)
#define CAN_TDT1R_TGT_Msk (0x1UL << CAN_TDT1R_TGT_Pos) /*!< 0x00000100 */
#define CAN_TDT1R_TGT CAN_TDT1R_TGT_Msk                /*!<Transmit Global Time */
#define CAN_TDT1R_TIME_Pos (16U)
#define CAN_TDT1R_TIME_Msk (0xFFFFUL << CAN_TDT1R_TIME_Pos) /*!< 0xFFFF0000 */
#define CAN_TDT1R_TIME CAN_TDT1R_TIME_Msk                   /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL1R register  ******************/
#define CAN_TDL1R_DATA0_Pos (0U)
#define CAN_TDL1R_DATA0_Msk (0xFFUL << CAN_TDL1R_DATA0_Pos) /*!< 0x000000FF */
#define CAN_TDL1R_DATA0 CAN_TDL1R_DATA0_Msk                 /*!<Data byte 0 */
#define CAN_TDL1R_DATA1_Pos (8U)
#define CAN_TDL1R_DATA1_Msk (0xFFUL << CAN_TDL1R_DATA1_Pos) /*!< 0x0000FF00 */
#define CAN_TDL1R_DATA1 CAN_TDL1R_DATA1_Msk                 /*!<Data byte 1 */
#define CAN_TDL1R_DATA2_Pos (16U)
#define CAN_TDL1R_DATA2_Msk (0xFFUL << CAN_TDL1R_DATA2_Pos) /*!< 0x00FF0000 */
#define CAN_TDL1R_DATA2 CAN_TDL1R_DATA2_Msk                 /*!<Data byte 2 */
#define CAN_TDL1R_DATA3_Pos (24U)
#define CAN_TDL1R_DATA3_Msk (0xFFUL << CAN_TDL1R_DATA3_Pos) /*!< 0xFF000000 */
#define CAN_TDL1R_DATA3 CAN_TDL1R_DATA3_Msk                 /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH1R register  ******************/
#define CAN_TDH1R_DATA4_Pos (0U)
#define CAN_TDH1R_DATA4_Msk (0xFFUL << CAN_TDH1R_DATA4_Pos) /*!< 0x000000FF */
#define CAN_TDH1R_DATA4 CAN_TDH1R_DATA4_Msk                 /*!<Data byte 4 */
#define CAN_TDH1R_DATA5_Pos (8U)
#define CAN_TDH1R_DATA5_Msk (0xFFUL << CAN_TDH1R_DATA5_Pos) /*!< 0x0000FF00 */
#define CAN_TDH1R_DATA5 CAN_TDH1R_DATA5_Msk                 /*!<Data byte 5 */
#define CAN_TDH1R_DATA6_Pos (16U)
#define CAN_TDH1R_DATA6_Msk (0xFFUL << CAN_TDH1R_DATA6_Pos) /*!< 0x00FF0000 */
#define CAN_TDH1R_DATA6 CAN_TDH1R_DATA6_Msk                 /*!<Data byte 6 */
#define CAN_TDH1R_DATA7_Pos (24U)
#define CAN_TDH1R_DATA7_Msk (0xFFUL << CAN_TDH1R_DATA7_Pos) /*!< 0xFF000000 */
#define CAN_TDH1R_DATA7 CAN_TDH1R_DATA7_Msk                 /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI2R register  *******************/
#define CAN_TI2R_TXRQ_Pos (0U)
#define CAN_TI2R_TXRQ_Msk (0x1UL << CAN_TI2R_TXRQ_Pos) /*!< 0x00000001 */
#define CAN_TI2R_TXRQ CAN_TI2R_TXRQ_Msk                /*!<Transmit Mailbox Request */
#define CAN_TI2R_RTR_Pos (1U)
#define CAN_TI2R_RTR_Msk (0x1UL << CAN_TI2R_RTR_Pos) /*!< 0x00000002 */
#define CAN_TI2R_RTR CAN_TI2R_RTR_Msk                /*!<Remote Transmission Request */
#define CAN_TI2R_IDE_Pos (2U)
#define CAN_TI2R_IDE_Msk (0x1UL << CAN_TI2R_IDE_Pos) /*!< 0x00000004 */
#define CAN_TI2R_IDE CAN_TI2R_IDE_Msk                /*!<Identifier Extension */
#define CAN_TI2R_EXID_Pos (3U)
#define CAN_TI2R_EXID_Msk (0x3FFFFUL << CAN_TI2R_EXID_Pos) /*!< 0x001FFFF8 */
#define CAN_TI2R_EXID CAN_TI2R_EXID_Msk                    /*!<Extended identifier */
#define CAN_TI2R_STID_Pos (21U)
#define CAN_TI2R_STID_Msk (0x7FFUL << CAN_TI2R_STID_Pos) /*!< 0xFFE00000 */
#define CAN_TI2R_STID CAN_TI2R_STID_Msk                  /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT2R register  ******************/
#define CAN_TDT2R_DLC_Pos (0U)
#define CAN_TDT2R_DLC_Msk (0xFUL << CAN_TDT2R_DLC_Pos) /*!< 0x0000000F */
#define CAN_TDT2R_DLC CAN_TDT2R_DLC_Msk                /*!<Data Length Code */
#define CAN_TDT2R_TGT_Pos (8U)
#define CAN_TDT2R_TGT_Msk (0x1UL << CAN_TDT2R_TGT_Pos) /*!< 0x00000100 */
#define CAN_TDT2R_TGT CAN_TDT2R_TGT_Msk                /*!<Transmit Global Time */
#define CAN_TDT2R_TIME_Pos (16U)
#define CAN_TDT2R_TIME_Msk (0xFFFFUL << CAN_TDT2R_TIME_Pos) /*!< 0xFFFF0000 */
#define CAN_TDT2R_TIME CAN_TDT2R_TIME_Msk                   /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL2R register  ******************/
#define CAN_TDL2R_DATA0_Pos (0U)
#define CAN_TDL2R_DATA0_Msk (0xFFUL << CAN_TDL2R_DATA0_Pos) /*!< 0x000000FF */
#define CAN_TDL2R_DATA0 CAN_TDL2R_DATA0_Msk                 /*!<Data byte 0 */
#define CAN_TDL2R_DATA1_Pos (8U)
#define CAN_TDL2R_DATA1_Msk (0xFFUL << CAN_TDL2R_DATA1_Pos) /*!< 0x0000FF00 */
#define CAN_TDL2R_DATA1 CAN_TDL2R_DATA1_Msk                 /*!<Data byte 1 */
#define CAN_TDL2R_DATA2_Pos (16U)
#define CAN_TDL2R_DATA2_Msk (0xFFUL << CAN_TDL2R_DATA2_Pos) /*!< 0x00FF0000 */
#define CAN_TDL2R_DATA2 CAN_TDL2R_DATA2_Msk                 /*!<Data byte 2 */
#define CAN_TDL2R_DATA3_Pos (24U)
#define CAN_TDL2R_DATA3_Msk (0xFFUL << CAN_TDL2R_DATA3_Pos) /*!< 0xFF000000 */
#define CAN_TDL2R_DATA3 CAN_TDL2R_DATA3_Msk                 /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH2R register  ******************/
#define CAN_TDH2R_DATA4_Pos (0U)
#define CAN_TDH2R_DATA4_Msk (0xFFUL << CAN_TDH2R_DATA4_Pos) /*!< 0x000000FF */
#define CAN_TDH2R_DATA4 CAN_TDH2R_DATA4_Msk                 /*!<Data byte 4 */
#define CAN_TDH2R_DATA5_Pos (8U)
#define CAN_TDH2R_DATA5_Msk (0xFFUL << CAN_TDH2R_DATA5_Pos) /*!< 0x0000FF00 */
#define CAN_TDH2R_DATA5 CAN_TDH2R_DATA5_Msk                 /*!<Data byte 5 */
#define CAN_TDH2R_DATA6_Pos (16U)
#define CAN_TDH2R_DATA6_Msk (0xFFUL << CAN_TDH2R_DATA6_Pos) /*!< 0x00FF0000 */
#define CAN_TDH2R_DATA6 CAN_TDH2R_DATA6_Msk                 /*!<Data byte 6 */
#define CAN_TDH2R_DATA7_Pos (24U)
#define CAN_TDH2R_DATA7_Msk (0xFFUL << CAN_TDH2R_DATA7_Pos) /*!< 0xFF000000 */
#define CAN_TDH2R_DATA7 CAN_TDH2R_DATA7_Msk                 /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI0R register  *******************/
#define CAN_RI0R_RTR_Pos (1U)
#define CAN_RI0R_RTR_Msk (0x1UL << CAN_RI0R_RTR_Pos) /*!< 0x00000002 */
#define CAN_RI0R_RTR CAN_RI0R_RTR_Msk                /*!<Remote Transmission Request */
#define CAN_RI0R_IDE_Pos (2U)
#define CAN_RI0R_IDE_Msk (0x1UL << CAN_RI0R_IDE_Pos) /*!< 0x00000004 */
#define CAN_RI0R_IDE CAN_RI0R_IDE_Msk                /*!<Identifier Extension */
#define CAN_RI0R_EXID_Pos (3U)
#define CAN_RI0R_EXID_Msk (0x3FFFFUL << CAN_RI0R_EXID_Pos) /*!< 0x001FFFF8 */
#define CAN_RI0R_EXID CAN_RI0R_EXID_Msk                    /*!<Extended Identifier */
#define CAN_RI0R_STID_Pos (21U)
#define CAN_RI0R_STID_Msk (0x7FFUL << CAN_RI0R_STID_Pos) /*!< 0xFFE00000 */
#define CAN_RI0R_STID CAN_RI0R_STID_Msk                  /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT0R register  ******************/
#define CAN_RDT0R_DLC_Pos (0U)
#define CAN_RDT0R_DLC_Msk (0xFUL << CAN_RDT0R_DLC_Pos) /*!< 0x0000000F */
#define CAN_RDT0R_DLC CAN_RDT0R_DLC_Msk                /*!<Data Length Code */
#define CAN_RDT0R_FMI_Pos (8U)
#define CAN_RDT0R_FMI_Msk (0xFFUL << CAN_RDT0R_FMI_Pos) /*!< 0x0000FF00 */
#define CAN_RDT0R_FMI CAN_RDT0R_FMI_Msk                 /*!<Filter Match Index */
#define CAN_RDT0R_TIME_Pos (16U)
#define CAN_RDT0R_TIME_Msk (0xFFFFUL << CAN_RDT0R_TIME_Pos) /*!< 0xFFFF0000 */
#define CAN_RDT0R_TIME CAN_RDT0R_TIME_Msk                   /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL0R register  ******************/
#define CAN_RDL0R_DATA0_Pos (0U)
#define CAN_RDL0R_DATA0_Msk (0xFFUL << CAN_RDL0R_DATA0_Pos) /*!< 0x000000FF */
#define CAN_RDL0R_DATA0 CAN_RDL0R_DATA0_Msk                 /*!<Data byte 0 */
#define CAN_RDL0R_DATA1_Pos (8U)
#define CAN_RDL0R_DATA1_Msk (0xFFUL << CAN_RDL0R_DATA1_Pos) /*!< 0x0000FF00 */
#define CAN_RDL0R_DATA1 CAN_RDL0R_DATA1_Msk                 /*!<Data byte 1 */
#define CAN_RDL0R_DATA2_Pos (16U)
#define CAN_RDL0R_DATA2_Msk (0xFFUL << CAN_RDL0R_DATA2_Pos) /*!< 0x00FF0000 */
#define CAN_RDL0R_DATA2 CAN_RDL0R_DATA2_Msk                 /*!<Data byte 2 */
#define CAN_RDL0R_DATA3_Pos (24U)
#define CAN_RDL0R_DATA3_Msk (0xFFUL << CAN_RDL0R_DATA3_Pos) /*!< 0xFF000000 */
#define CAN_RDL0R_DATA3 CAN_RDL0R_DATA3_Msk                 /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH0R register  ******************/
#define CAN_RDH0R_DATA4_Pos (0U)
#define CAN_RDH0R_DATA4_Msk (0xFFUL << CAN_RDH0R_DATA4_Pos) /*!< 0x000000FF */
#define CAN_RDH0R_DATA4 CAN_RDH0R_DATA4_Msk                 /*!<Data byte 4 */
#define CAN_RDH0R_DATA5_Pos (8U)
#define CAN_RDH0R_DATA5_Msk (0xFFUL << CAN_RDH0R_DATA5_Pos) /*!< 0x0000FF00 */
#define CAN_RDH0R_DATA5 CAN_RDH0R_DATA5_Msk                 /*!<Data byte 5 */
#define CAN_RDH0R_DATA6_Pos (16U)
#define CAN_RDH0R_DATA6_Msk (0xFFUL << CAN_RDH0R_DATA6_Pos) /*!< 0x00FF0000 */
#define CAN_RDH0R_DATA6 CAN_RDH0R_DATA6_Msk                 /*!<Data byte 6 */
#define CAN_RDH0R_DATA7_Pos (24U)
#define CAN_RDH0R_DATA7_Msk (0xFFUL << CAN_RDH0R_DATA7_Pos) /*!< 0xFF000000 */
#define CAN_RDH0R_DATA7 CAN_RDH0R_DATA7_Msk                 /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI1R register  *******************/
#define CAN_RI1R_RTR_Pos (1U)
#define CAN_RI1R_RTR_Msk (0x1UL << CAN_RI1R_RTR_Pos) /*!< 0x00000002 */
#define CAN_RI1R_RTR CAN_RI1R_RTR_Msk                /*!<Remote Transmission Request */
#define CAN_RI1R_IDE_Pos (2U)
#define CAN_RI1R_IDE_Msk (0x1UL << CAN_RI1R_IDE_Pos) /*!< 0x00000004 */
#define CAN_RI1R_IDE CAN_RI1R_IDE_Msk                /*!<Identifier Extension */
#define CAN_RI1R_EXID_Pos (3U)
#define CAN_RI1R_EXID_Msk (0x3FFFFUL << CAN_RI1R_EXID_Pos) /*!< 0x001FFFF8 */
#define CAN_RI1R_EXID CAN_RI1R_EXID_Msk                    /*!<Extended identifier */
#define CAN_RI1R_STID_Pos (21U)
#define CAN_RI1R_STID_Msk (0x7FFUL << CAN_RI1R_STID_Pos) /*!< 0xFFE00000 */
#define CAN_RI1R_STID CAN_RI1R_STID_Msk                  /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT1R register  ******************/
#define CAN_RDT1R_DLC_Pos (0U)
#define CAN_RDT1R_DLC_Msk (0xFUL << CAN_RDT1R_DLC_Pos) /*!< 0x0000000F */
#define CAN_RDT1R_DLC CAN_RDT1R_DLC_Msk                /*!<Data Length Code */
#define CAN_RDT1R_FMI_Pos (8U)
#define CAN_RDT1R_FMI_Msk (0xFFUL << CAN_RDT1R_FMI_Pos) /*!< 0x0000FF00 */
#define CAN_RDT1R_FMI CAN_RDT1R_FMI_Msk                 /*!<Filter Match Index */
#define CAN_RDT1R_TIME_Pos (16U)
#define CAN_RDT1R_TIME_Msk (0xFFFFUL << CAN_RDT1R_TIME_Pos) /*!< 0xFFFF0000 */
#define CAN_RDT1R_TIME CAN_RDT1R_TIME_Msk                   /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL1R register  ******************/
#define CAN_RDL1R_DATA0_Pos (0U)
#define CAN_RDL1R_DATA0_Msk (0xFFUL << CAN_RDL1R_DATA0_Pos) /*!< 0x000000FF */
#define CAN_RDL1R_DATA0 CAN_RDL1R_DATA0_Msk                 /*!<Data byte 0 */
#define CAN_RDL1R_DATA1_Pos (8U)
#define CAN_RDL1R_DATA1_Msk (0xFFUL << CAN_RDL1R_DATA1_Pos) /*!< 0x0000FF00 */
#define CAN_RDL1R_DATA1 CAN_RDL1R_DATA1_Msk                 /*!<Data byte 1 */
#define CAN_RDL1R_DATA2_Pos (16U)
#define CAN_RDL1R_DATA2_Msk (0xFFUL << CAN_RDL1R_DATA2_Pos) /*!< 0x00FF0000 */
#define CAN_RDL1R_DATA2 CAN_RDL1R_DATA2_Msk                 /*!<Data byte 2 */
#define CAN_RDL1R_DATA3_Pos (24U)
#define CAN_RDL1R_DATA3_Msk (0xFFUL << CAN_RDL1R_DATA3_Pos) /*!< 0xFF000000 */
#define CAN_RDL1R_DATA3 CAN_RDL1R_DATA3_Msk                 /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH1R register  ******************/
#define CAN_RDH1R_DATA4_Pos (0U)
#define CAN_RDH1R_DATA4_Msk (0xFFUL << CAN_RDH1R_DATA4_Pos) /*!< 0x000000FF */
#define CAN_RDH1R_DATA4 CAN_RDH1R_DATA4_Msk                 /*!<Data byte 4 */
#define CAN_RDH1R_DATA5_Pos (8U)
#define CAN_RDH1R_DATA5_Msk (0xFFUL << CAN_RDH1R_DATA5_Pos) /*!< 0x0000FF00 */
#define CAN_RDH1R_DATA5 CAN_RDH1R_DATA5_Msk                 /*!<Data byte 5 */
#define CAN_RDH1R_DATA6_Pos (16U)
#define CAN_RDH1R_DATA6_Msk (0xFFUL << CAN_RDH1R_DATA6_Pos) /*!< 0x00FF0000 */
#define CAN_RDH1R_DATA6 CAN_RDH1R_DATA6_Msk                 /*!<Data byte 6 */
#define CAN_RDH1R_DATA7_Pos (24U)
#define CAN_RDH1R_DATA7_Msk (0xFFUL << CAN_RDH1R_DATA7_Pos) /*!< 0xFF000000 */
#define CAN_RDH1R_DATA7 CAN_RDH1R_DATA7_Msk                 /*!<Data byte 7 */

/*!<CAN filter registers */
/*******************  Bit definition for CAN_FMR register  ********************/
#define CAN_FMR_FINIT_Pos (0U)
#define CAN_FMR_FINIT_Msk (0x1UL << CAN_FMR_FINIT_Pos) /*!< 0x00000001 */
#define CAN_FMR_FINIT CAN_FMR_FINIT_Msk                /*!<Filter Init Mode */
#define CAN_FMR_CAN2SB_Pos (8U)
#define CAN_FMR_CAN2SB_Msk (0x3FUL << CAN_FMR_CAN2SB_Pos) /*!< 0x00003F00 */
#define CAN_FMR_CAN2SB CAN_FMR_CAN2SB_Msk                 /*!<CAN2 start bank */

/*******************  Bit definition for CAN_FM1R register  *******************/
#define CAN_FM1R_FBM_Pos (0U)
#define CAN_FM1R_FBM_Msk (0xFFFFFFFUL << CAN_FM1R_FBM_Pos) /*!< 0x0FFFFFFF */
#define CAN_FM1R_FBM CAN_FM1R_FBM_Msk                      /*!<Filter Mode */
#define CAN_FM1R_FBM0_Pos (0U)
#define CAN_FM1R_FBM0_Msk (0x1UL << CAN_FM1R_FBM0_Pos) /*!< 0x00000001 */
#define CAN_FM1R_FBM0 CAN_FM1R_FBM0_Msk                /*!<Filter Init Mode bit 0 */
#define CAN_FM1R_FBM1_Pos (1U)
#define CAN_FM1R_FBM1_Msk (0x1UL << CAN_FM1R_FBM1_Pos) /*!< 0x00000002 */
#define CAN_FM1R_FBM1 CAN_FM1R_FBM1_Msk                /*!<Filter Init Mode bit 1 */
#define CAN_FM1R_FBM2_Pos (2U)
#define CAN_FM1R_FBM2_Msk (0x1UL << CAN_FM1R_FBM2_Pos) /*!< 0x00000004 */
#define CAN_FM1R_FBM2 CAN_FM1R_FBM2_Msk                /*!<Filter Init Mode bit 2 */
#define CAN_FM1R_FBM3_Pos (3U)
#define CAN_FM1R_FBM3_Msk (0x1UL << CAN_FM1R_FBM3_Pos) /*!< 0x00000008 */
#define CAN_FM1R_FBM3 CAN_FM1R_FBM3_Msk                /*!<Filter Init Mode bit 3 */
#define CAN_FM1R_FBM4_Pos (4U)
#define CAN_FM1R_FBM4_Msk (0x1UL << CAN_FM1R_FBM4_Pos) /*!< 0x00000010 */
#define CAN_FM1R_FBM4 CAN_FM1R_FBM4_Msk                /*!<Filter Init Mode bit 4 */
#define CAN_FM1R_FBM5_Pos (5U)
#define CAN_FM1R_FBM5_Msk (0x1UL << CAN_FM1R_FBM5_Pos) /*!< 0x00000020 */
#define CAN_FM1R_FBM5 CAN_FM1R_FBM5_Msk                /*!<Filter Init Mode bit 5 */
#define CAN_FM1R_FBM6_Pos (6U)
#define CAN_FM1R_FBM6_Msk (0x1UL << CAN_FM1R_FBM6_Pos) /*!< 0x00000040 */
#define CAN_FM1R_FBM6 CAN_FM1R_FBM6_Msk                /*!<Filter Init Mode bit 6 */
#define CAN_FM1R_FBM7_Pos (7U)
#define CAN_FM1R_FBM7_Msk (0x1UL << CAN_FM1R_FBM7_Pos) /*!< 0x00000080 */
#define CAN_FM1R_FBM7 CAN_FM1R_FBM7_Msk                /*!<Filter Init Mode bit 7 */
#define CAN_FM1R_FBM8_Pos (8U)
#define CAN_FM1R_FBM8_Msk (0x1UL << CAN_FM1R_FBM8_Pos) /*!< 0x00000100 */
#define CAN_FM1R_FBM8 CAN_FM1R_FBM8_Msk                /*!<Filter Init Mode bit 8 */
#define CAN_FM1R_FBM9_Pos (9U)
#define CAN_FM1R_FBM9_Msk (0x1UL << CAN_FM1R_FBM9_Pos) /*!< 0x00000200 */
#define CAN_FM1R_FBM9 CAN_FM1R_FBM9_Msk                /*!<Filter Init Mode bit 9 */
#define CAN_FM1R_FBM10_Pos (10U)
#define CAN_FM1R_FBM10_Msk (0x1UL << CAN_FM1R_FBM10_Pos) /*!< 0x00000400 */
#define CAN_FM1R_FBM10 CAN_FM1R_FBM10_Msk                /*!<Filter Init Mode bit 10 */
#define CAN_FM1R_FBM11_Pos (11U)
#define CAN_FM1R_FBM11_Msk (0x1UL << CAN_FM1R_FBM11_Pos) /*!< 0x00000800 */
#define CAN_FM1R_FBM11 CAN_FM1R_FBM11_Msk                /*!<Filter Init Mode bit 11 */
#define CAN_FM1R_FBM12_Pos (12U)
#define CAN_FM1R_FBM12_Msk (0x1UL << CAN_FM1R_FBM12_Pos) /*!< 0x00001000 */
#define CAN_FM1R_FBM12 CAN_FM1R_FBM12_Msk                /*!<Filter Init Mode bit 12 */
#define CAN_FM1R_FBM13_Pos (13U)
#define CAN_FM1R_FBM13_Msk (0x1UL << CAN_FM1R_FBM13_Pos) /*!< 0x00002000 */
#define CAN_FM1R_FBM13 CAN_FM1R_FBM13_Msk                /*!<Filter Init Mode bit 13 */
#define CAN_FM1R_FBM14_Pos (14U)
#define CAN_FM1R_FBM14_Msk (0x1UL << CAN_FM1R_FBM14_Pos) /*!< 0x00004000 */
#define CAN_FM1R_FBM14 CAN_FM1R_FBM14_Msk                /*!<Filter Init Mode bit 14 */
#define CAN_FM1R_FBM15_Pos (15U)
#define CAN_FM1R_FBM15_Msk (0x1UL << CAN_FM1R_FBM15_Pos) /*!< 0x00008000 */
#define CAN_FM1R_FBM15 CAN_FM1R_FBM15_Msk                /*!<Filter Init Mode bit 15 */
#define CAN_FM1R_FBM16_Pos (16U)
#define CAN_FM1R_FBM16_Msk (0x1UL << CAN_FM1R_FBM16_Pos) /*!< 0x00010000 */
#define CAN_FM1R_FBM16 CAN_FM1R_FBM16_Msk                /*!<Filter Init Mode bit 16 */
#define CAN_FM1R_FBM17_Pos (17U)
#define CAN_FM1R_FBM17_Msk (0x1UL << CAN_FM1R_FBM17_Pos) /*!< 0x00020000 */
#define CAN_FM1R_FBM17 CAN_FM1R_FBM17_Msk                /*!<Filter Init Mode bit 17 */
#define CAN_FM1R_FBM18_Pos (18U)
#define CAN_FM1R_FBM18_Msk (0x1UL << CAN_FM1R_FBM18_Pos) /*!< 0x00040000 */
#define CAN_FM1R_FBM18 CAN_FM1R_FBM18_Msk                /*!<Filter Init Mode bit 18 */
#define CAN_FM1R_FBM19_Pos (19U)
#define CAN_FM1R_FBM19_Msk (0x1UL << CAN_FM1R_FBM19_Pos) /*!< 0x00080000 */
#define CAN_FM1R_FBM19 CAN_FM1R_FBM19_Msk                /*!<Filter Init Mode bit 19 */
#define CAN_FM1R_FBM20_Pos (20U)
#define CAN_FM1R_FBM20_Msk (0x1UL << CAN_FM1R_FBM20_Pos) /*!< 0x00100000 */
#define CAN_FM1R_FBM20 CAN_FM1R_FBM20_Msk                /*!<Filter Init Mode bit 20 */
#define CAN_FM1R_FBM21_Pos (21U)
#define CAN_FM1R_FBM21_Msk (0x1UL << CAN_FM1R_FBM21_Pos) /*!< 0x00200000 */
#define CAN_FM1R_FBM21 CAN_FM1R_FBM21_Msk                /*!<Filter Init Mode bit 21 */
#define CAN_FM1R_FBM22_Pos (22U)
#define CAN_FM1R_FBM22_Msk (0x1UL << CAN_FM1R_FBM22_Pos) /*!< 0x00400000 */
#define CAN_FM1R_FBM22 CAN_FM1R_FBM22_Msk                /*!<Filter Init Mode bit 22 */
#define CAN_FM1R_FBM23_Pos (23U)
#define CAN_FM1R_FBM23_Msk (0x1UL << CAN_FM1R_FBM23_Pos) /*!< 0x00800000 */
#define CAN_FM1R_FBM23 CAN_FM1R_FBM23_Msk                /*!<Filter Init Mode bit 23 */
#define CAN_FM1R_FBM24_Pos (24U)
#define CAN_FM1R_FBM24_Msk (0x1UL << CAN_FM1R_FBM24_Pos) /*!< 0x01000000 */
#define CAN_FM1R_FBM24 CAN_FM1R_FBM24_Msk                /*!<Filter Init Mode bit 24 */
#define CAN_FM1R_FBM25_Pos (25U)
#define CAN_FM1R_FBM25_Msk (0x1UL << CAN_FM1R_FBM25_Pos) /*!< 0x02000000 */
#define CAN_FM1R_FBM25 CAN_FM1R_FBM25_Msk                /*!<Filter Init Mode bit 25 */
#define CAN_FM1R_FBM26_Pos (26U)
#define CAN_FM1R_FBM26_Msk (0x1UL << CAN_FM1R_FBM26_Pos) /*!< 0x04000000 */
#define CAN_FM1R_FBM26 CAN_FM1R_FBM26_Msk                /*!<Filter Init Mode bit 26 */
#define CAN_FM1R_FBM27_Pos (27U)
#define CAN_FM1R_FBM27_Msk (0x1UL << CAN_FM1R_FBM27_Pos) /*!< 0x08000000 */
#define CAN_FM1R_FBM27 CAN_FM1R_FBM27_Msk                /*!<Filter Init Mode bit 27 */

/*******************  Bit definition for CAN_FS1R register  *******************/
#define CAN_FS1R_FSC_Pos (0U)
#define CAN_FS1R_FSC_Msk (0xFFFFFFFUL << CAN_FS1R_FSC_Pos) /*!< 0x0FFFFFFF */
#define CAN_FS1R_FSC CAN_FS1R_FSC_Msk                      /*!<Filter Scale Configuration */
#define CAN_FS1R_FSC0_Pos (0U)
#define CAN_FS1R_FSC0_Msk (0x1UL << CAN_FS1R_FSC0_Pos) /*!< 0x00000001 */
#define CAN_FS1R_FSC0 CAN_FS1R_FSC0_Msk                /*!<Filter Scale Configuration bit 0 */
#define CAN_FS1R_FSC1_Pos (1U)
#define CAN_FS1R_FSC1_Msk (0x1UL << CAN_FS1R_FSC1_Pos) /*!< 0x00000002 */
#define CAN_FS1R_FSC1 CAN_FS1R_FSC1_Msk                /*!<Filter Scale Configuration bit 1 */
#define CAN_FS1R_FSC2_Pos (2U)
#define CAN_FS1R_FSC2_Msk (0x1UL << CAN_FS1R_FSC2_Pos) /*!< 0x00000004 */
#define CAN_FS1R_FSC2 CAN_FS1R_FSC2_Msk                /*!<Filter Scale Configuration bit 2 */
#define CAN_FS1R_FSC3_Pos (3U)
#define CAN_FS1R_FSC3_Msk (0x1UL << CAN_FS1R_FSC3_Pos) /*!< 0x00000008 */
#define CAN_FS1R_FSC3 CAN_FS1R_FSC3_Msk                /*!<Filter Scale Configuration bit 3 */
#define CAN_FS1R_FSC4_Pos (4U)
#define CAN_FS1R_FSC4_Msk (0x1UL << CAN_FS1R_FSC4_Pos) /*!< 0x00000010 */
#define CAN_FS1R_FSC4 CAN_FS1R_FSC4_Msk                /*!<Filter Scale Configuration bit 4 */
#define CAN_FS1R_FSC5_Pos (5U)
#define CAN_FS1R_FSC5_Msk (0x1UL << CAN_FS1R_FSC5_Pos) /*!< 0x00000020 */
#define CAN_FS1R_FSC5 CAN_FS1R_FSC5_Msk                /*!<Filter Scale Configuration bit 5 */
#define CAN_FS1R_FSC6_Pos (6U)
#define CAN_FS1R_FSC6_Msk (0x1UL << CAN_FS1R_FSC6_Pos) /*!< 0x00000040 */
#define CAN_FS1R_FSC6 CAN_FS1R_FSC6_Msk                /*!<Filter Scale Configuration bit 6 */
#define CAN_FS1R_FSC7_Pos (7U)
#define CAN_FS1R_FSC7_Msk (0x1UL << CAN_FS1R_FSC7_Pos) /*!< 0x00000080 */
#define CAN_FS1R_FSC7 CAN_FS1R_FSC7_Msk                /*!<Filter Scale Configuration bit 7 */
#define CAN_FS1R_FSC8_Pos (8U)
#define CAN_FS1R_FSC8_Msk (0x1UL << CAN_FS1R_FSC8_Pos) /*!< 0x00000100 */
#define CAN_FS1R_FSC8 CAN_FS1R_FSC8_Msk                /*!<Filter Scale Configuration bit 8 */
#define CAN_FS1R_FSC9_Pos (9U)
#define CAN_FS1R_FSC9_Msk (0x1UL << CAN_FS1R_FSC9_Pos) /*!< 0x00000200 */
#define CAN_FS1R_FSC9 CAN_FS1R_FSC9_Msk                /*!<Filter Scale Configuration bit 9 */
#define CAN_FS1R_FSC10_Pos (10U)
#define CAN_FS1R_FSC10_Msk (0x1UL << CAN_FS1R_FSC10_Pos) /*!< 0x00000400 */
#define CAN_FS1R_FSC10 CAN_FS1R_FSC10_Msk                /*!<Filter Scale Configuration bit 10 */
#define CAN_FS1R_FSC11_Pos (11U)
#define CAN_FS1R_FSC11_Msk (0x1UL << CAN_FS1R_FSC11_Pos) /*!< 0x00000800 */
#define CAN_FS1R_FSC11 CAN_FS1R_FSC11_Msk                /*!<Filter Scale Configuration bit 11 */
#define CAN_FS1R_FSC12_Pos (12U)
#define CAN_FS1R_FSC12_Msk (0x1UL << CAN_FS1R_FSC12_Pos) /*!< 0x00001000 */
#define CAN_FS1R_FSC12 CAN_FS1R_FSC12_Msk                /*!<Filter Scale Configuration bit 12 */
#define CAN_FS1R_FSC13_Pos (13U)
#define CAN_FS1R_FSC13_Msk (0x1UL << CAN_FS1R_FSC13_Pos) /*!< 0x00002000 */
#define CAN_FS1R_FSC13 CAN_FS1R_FSC13_Msk                /*!<Filter Scale Configuration bit 13 */
#define CAN_FS1R_FSC14_Pos (14U)
#define CAN_FS1R_FSC14_Msk (0x1UL << CAN_FS1R_FSC14_Pos) /*!< 0x00004000 */
#define CAN_FS1R_FSC14 CAN_FS1R_FSC14_Msk                /*!<Filter Scale Configuration bit 14 */
#define CAN_FS1R_FSC15_Pos (15U)
#define CAN_FS1R_FSC15_Msk (0x1UL << CAN_FS1R_FSC15_Pos) /*!< 0x00008000 */
#define CAN_FS1R_FSC15 CAN_FS1R_FSC15_Msk                /*!<Filter Scale Configuration bit 15 */
#define CAN_FS1R_FSC16_Pos (16U)
#define CAN_FS1R_FSC16_Msk (0x1UL << CAN_FS1R_FSC16_Pos) /*!< 0x00010000 */
#define CAN_FS1R_FSC16 CAN_FS1R_FSC16_Msk                /*!<Filter Scale Configuration bit 16 */
#define CAN_FS1R_FSC17_Pos (17U)
#define CAN_FS1R_FSC17_Msk (0x1UL << CAN_FS1R_FSC17_Pos) /*!< 0x00020000 */
#define CAN_FS1R_FSC17 CAN_FS1R_FSC17_Msk                /*!<Filter Scale Configuration bit 17 */
#define CAN_FS1R_FSC18_Pos (18U)
#define CAN_FS1R_FSC18_Msk (0x1UL << CAN_FS1R_FSC18_Pos) /*!< 0x00040000 */
#define CAN_FS1R_FSC18 CAN_FS1R_FSC18_Msk                /*!<Filter Scale Configuration bit 18 */
#define CAN_FS1R_FSC19_Pos (19U)
#define CAN_FS1R_FSC19_Msk (0x1UL << CAN_FS1R_FSC19_Pos) /*!< 0x00080000 */
#define CAN_FS1R_FSC19 CAN_FS1R_FSC19_Msk                /*!<Filter Scale Configuration bit 19 */
#define CAN_FS1R_FSC20_Pos (20U)
#define CAN_FS1R_FSC20_Msk (0x1UL << CAN_FS1R_FSC20_Pos) /*!< 0x00100000 */
#define CAN_FS1R_FSC20 CAN_FS1R_FSC20_Msk                /*!<Filter Scale Configuration bit 20 */
#define CAN_FS1R_FSC21_Pos (21U)
#define CAN_FS1R_FSC21_Msk (0x1UL << CAN_FS1R_FSC21_Pos) /*!< 0x00200000 */
#define CAN_FS1R_FSC21 CAN_FS1R_FSC21_Msk                /*!<Filter Scale Configuration bit 21 */
#define CAN_FS1R_FSC22_Pos (22U)
#define CAN_FS1R_FSC22_Msk (0x1UL << CAN_FS1R_FSC22_Pos) /*!< 0x00400000 */
#define CAN_FS1R_FSC22 CAN_FS1R_FSC22_Msk                /*!<Filter Scale Configuration bit 22 */
#define CAN_FS1R_FSC23_Pos (23U)
#define CAN_FS1R_FSC23_Msk (0x1UL << CAN_FS1R_FSC23_Pos) /*!< 0x00800000 */
#define CAN_FS1R_FSC23 CAN_FS1R_FSC23_Msk                /*!<Filter Scale Configuration bit 23 */
#define CAN_FS1R_FSC24_Pos (24U)
#define CAN_FS1R_FSC24_Msk (0x1UL << CAN_FS1R_FSC24_Pos) /*!< 0x01000000 */
#define CAN_FS1R_FSC24 CAN_FS1R_FSC24_Msk                /*!<Filter Scale Configuration bit 24 */
#define CAN_FS1R_FSC25_Pos (25U)
#define CAN_FS1R_FSC25_Msk (0x1UL << CAN_FS1R_FSC25_Pos) /*!< 0x02000000 */
#define CAN_FS1R_FSC25 CAN_FS1R_FSC25_Msk                /*!<Filter Scale Configuration bit 25 */
#define CAN_FS1R_FSC26_Pos (26U)
#define CAN_FS1R_FSC26_Msk (0x1UL << CAN_FS1R_FSC26_Pos) /*!< 0x04000000 */
#define CAN_FS1R_FSC26 CAN_FS1R_FSC26_Msk                /*!<Filter Scale Configuration bit 26 */
#define CAN_FS1R_FSC27_Pos (27U)
#define CAN_FS1R_FSC27_Msk (0x1UL << CAN_FS1R_FSC27_Pos) /*!< 0x08000000 */
#define CAN_FS1R_FSC27 CAN_FS1R_FSC27_Msk                /*!<Filter Scale Configuration bit 27 */

/******************  Bit definition for CAN_FFA1R register  *******************/
#define CAN_FFA1R_FFA_Pos (0U)
#define CAN_FFA1R_FFA_Msk (0xFFFFFFFUL << CAN_FFA1R_FFA_Pos) /*!< 0x0FFFFFFF */
#define CAN_FFA1R_FFA CAN_FFA1R_FFA_Msk                      /*!<Filter FIFO Assignment */
#define CAN_FFA1R_FFA0_Pos (0U)
#define CAN_FFA1R_FFA0_Msk (0x1UL << CAN_FFA1R_FFA0_Pos) /*!< 0x00000001 */
#define CAN_FFA1R_FFA0 CAN_FFA1R_FFA0_Msk                /*!<Filter FIFO Assignment bit 0 */
#define CAN_FFA1R_FFA1_Pos (1U)
#define CAN_FFA1R_FFA1_Msk (0x1UL << CAN_FFA1R_FFA1_Pos) /*!< 0x00000002 */
#define CAN_FFA1R_FFA1 CAN_FFA1R_FFA1_Msk                /*!<Filter FIFO Assignment bit 1 */
#define CAN_FFA1R_FFA2_Pos (2U)
#define CAN_FFA1R_FFA2_Msk (0x1UL << CAN_FFA1R_FFA2_Pos) /*!< 0x00000004 */
#define CAN_FFA1R_FFA2 CAN_FFA1R_FFA2_Msk                /*!<Filter FIFO Assignment bit 2 */
#define CAN_FFA1R_FFA3_Pos (3U)
#define CAN_FFA1R_FFA3_Msk (0x1UL << CAN_FFA1R_FFA3_Pos) /*!< 0x00000008 */
#define CAN_FFA1R_FFA3 CAN_FFA1R_FFA3_Msk                /*!<Filter FIFO Assignment bit 3 */
#define CAN_FFA1R_FFA4_Pos (4U)
#define CAN_FFA1R_FFA4_Msk (0x1UL << CAN_FFA1R_FFA4_Pos) /*!< 0x00000010 */
#define CAN_FFA1R_FFA4 CAN_FFA1R_FFA4_Msk                /*!<Filter FIFO Assignment bit 4 */
#define CAN_FFA1R_FFA5_Pos (5U)
#define CAN_FFA1R_FFA5_Msk (0x1UL << CAN_FFA1R_FFA5_Pos) /*!< 0x00000020 */
#define CAN_FFA1R_FFA5 CAN_FFA1R_FFA5_Msk                /*!<Filter FIFO Assignment bit 5 */
#define CAN_FFA1R_FFA6_Pos (6U)
#define CAN_FFA1R_FFA6_Msk (0x1UL << CAN_FFA1R_FFA6_Pos) /*!< 0x00000040 */
#define CAN_FFA1R_FFA6 CAN_FFA1R_FFA6_Msk                /*!<Filter FIFO Assignment bit 6 */
#define CAN_FFA1R_FFA7_Pos (7U)
#define CAN_FFA1R_FFA7_Msk (0x1UL << CAN_FFA1R_FFA7_Pos) /*!< 0x00000080 */
#define CAN_FFA1R_FFA7 CAN_FFA1R_FFA7_Msk                /*!<Filter FIFO Assignment bit 7 */
#define CAN_FFA1R_FFA8_Pos (8U)
#define CAN_FFA1R_FFA8_Msk (0x1UL << CAN_FFA1R_FFA8_Pos) /*!< 0x00000100 */
#define CAN_FFA1R_FFA8 CAN_FFA1R_FFA8_Msk                /*!<Filter FIFO Assignment bit 8 */
#define CAN_FFA1R_FFA9_Pos (9U)
#define CAN_FFA1R_FFA9_Msk (0x1UL << CAN_FFA1R_FFA9_Pos) /*!< 0x00000200 */
#define CAN_FFA1R_FFA9 CAN_FFA1R_FFA9_Msk                /*!<Filter FIFO Assignment bit 9 */
#define CAN_FFA1R_FFA10_Pos (10U)
#define CAN_FFA1R_FFA10_Msk (0x1UL << CAN_FFA1R_FFA10_Pos) /*!< 0x00000400 */
#define CAN_FFA1R_FFA10 CAN_FFA1R_FFA10_Msk                /*!<Filter FIFO Assignment bit 10 */
#define CAN_FFA1R_FFA11_Pos (11U)
#define CAN_FFA1R_FFA11_Msk (0x1UL << CAN_FFA1R_FFA11_Pos) /*!< 0x00000800 */
#define CAN_FFA1R_FFA11 CAN_FFA1R_FFA11_Msk                /*!<Filter FIFO Assignment bit 11 */
#define CAN_FFA1R_FFA12_Pos (12U)
#define CAN_FFA1R_FFA12_Msk (0x1UL << CAN_FFA1R_FFA12_Pos) /*!< 0x00001000 */
#define CAN_FFA1R_FFA12 CAN_FFA1R_FFA12_Msk                /*!<Filter FIFO Assignment bit 12 */
#define CAN_FFA1R_FFA13_Pos (13U)
#define CAN_FFA1R_FFA13_Msk (0x1UL << CAN_FFA1R_FFA13_Pos) /*!< 0x00002000 */
#define CAN_FFA1R_FFA13 CAN_FFA1R_FFA13_Msk                /*!<Filter FIFO Assignment bit 13 */
#define CAN_FFA1R_FFA14_Pos (14U)
#define CAN_FFA1R_FFA14_Msk (0x1UL << CAN_FFA1R_FFA14_Pos) /*!< 0x00004000 */
#define CAN_FFA1R_FFA14 CAN_FFA1R_FFA14_Msk                /*!<Filter FIFO Assignment bit 14 */
#define CAN_FFA1R_FFA15_Pos (15U)
#define CAN_FFA1R_FFA15_Msk (0x1UL << CAN_FFA1R_FFA15_Pos) /*!< 0x00008000 */
#define CAN_FFA1R_FFA15 CAN_FFA1R_FFA15_Msk                /*!<Filter FIFO Assignment bit 15 */
#define CAN_FFA1R_FFA16_Pos (16U)
#define CAN_FFA1R_FFA16_Msk (0x1UL << CAN_FFA1R_FFA16_Pos) /*!< 0x00010000 */
#define CAN_FFA1R_FFA16 CAN_FFA1R_FFA16_Msk                /*!<Filter FIFO Assignment bit 16 */
#define CAN_FFA1R_FFA17_Pos (17U)
#define CAN_FFA1R_FFA17_Msk (0x1UL << CAN_FFA1R_FFA17_Pos) /*!< 0x00020000 */
#define CAN_FFA1R_FFA17 CAN_FFA1R_FFA17_Msk                /*!<Filter FIFO Assignment bit 17 */
#define CAN_FFA1R_FFA18_Pos (18U)
#define CAN_FFA1R_FFA18_Msk (0x1UL << CAN_FFA1R_FFA18_Pos) /*!< 0x00040000 */
#define CAN_FFA1R_FFA18 CAN_FFA1R_FFA18_Msk                /*!<Filter FIFO Assignment bit 18 */
#define CAN_FFA1R_FFA19_Pos (19U)
#define CAN_FFA1R_FFA19_Msk (0x1UL << CAN_FFA1R_FFA19_Pos) /*!< 0x00080000 */
#define CAN_FFA1R_FFA19 CAN_FFA1R_FFA19_Msk                /*!<Filter FIFO Assignment bit 19 */
#define CAN_FFA1R_FFA20_Pos (20U)
#define CAN_FFA1R_FFA20_Msk (0x1UL << CAN_FFA1R_FFA20_Pos) /*!< 0x00100000 */
#define CAN_FFA1R_FFA20 CAN_FFA1R_FFA20_Msk                /*!<Filter FIFO Assignment bit 20 */
#define CAN_FFA1R_FFA21_Pos (21U)
#define CAN_FFA1R_FFA21_Msk (0x1UL << CAN_FFA1R_FFA21_Pos) /*!< 0x00200000 */
#define CAN_FFA1R_FFA21 CAN_FFA1R_FFA21_Msk                /*!<Filter FIFO Assignment bit 21 */
#define CAN_FFA1R_FFA22_Pos (22U)
#define CAN_FFA1R_FFA22_Msk (0x1UL << CAN_FFA1R_FFA22_Pos) /*!< 0x00400000 */
#define CAN_FFA1R_FFA22 CAN_FFA1R_FFA22_Msk                /*!<Filter FIFO Assignment bit 22 */
#define CAN_FFA1R_FFA23_Pos (23U)
#define CAN_FFA1R_FFA23_Msk (0x1UL << CAN_FFA1R_FFA23_Pos) /*!< 0x00800000 */
#define CAN_FFA1R_FFA23 CAN_FFA1R_FFA23_Msk                /*!<Filter FIFO Assignment bit 23 */
#define CAN_FFA1R_FFA24_Pos (24U)
#define CAN_FFA1R_FFA24_Msk (0x1UL << CAN_FFA1R_FFA24_Pos) /*!< 0x01000000 */
#define CAN_FFA1R_FFA24 CAN_FFA1R_FFA24_Msk                /*!<Filter FIFO Assignment bit 24 */
#define CAN_FFA1R_FFA25_Pos (25U)
#define CAN_FFA1R_FFA25_Msk (0x1UL << CAN_FFA1R_FFA25_Pos) /*!< 0x02000000 */
#define CAN_FFA1R_FFA25 CAN_FFA1R_FFA25_Msk                /*!<Filter FIFO Assignment bit 25 */
#define CAN_FFA1R_FFA26_Pos (26U)
#define CAN_FFA1R_FFA26_Msk (0x1UL << CAN_FFA1R_FFA26_Pos) /*!< 0x04000000 */
#define CAN_FFA1R_FFA26 CAN_FFA1R_FFA26_Msk                /*!<Filter FIFO Assignment bit 26 */
#define CAN_FFA1R_FFA27_Pos (27U)
#define CAN_FFA1R_FFA27_Msk (0x1UL << CAN_FFA1R_FFA27_Pos) /*!< 0x08000000 */
#define CAN_FFA1R_FFA27 CAN_FFA1R_FFA27_Msk                /*!<Filter FIFO Assignment bit 27 */

/*******************  Bit definition for CAN_FA1R register  *******************/
#define CAN_FA1R_FACT_Pos (0U)
#define CAN_FA1R_FACT_Msk (0xFFFFFFFUL << CAN_FA1R_FACT_Pos) /*!< 0x0FFFFFFF */
#define CAN_FA1R_FACT CAN_FA1R_FACT_Msk                      /*!<Filter Active */
#define CAN_FA1R_FACT0_Pos (0U)
#define CAN_FA1R_FACT0_Msk (0x1UL << CAN_FA1R_FACT0_Pos) /*!< 0x00000001 */
#define CAN_FA1R_FACT0 CAN_FA1R_FACT0_Msk                /*!<Filter Active bit 0 */
#define CAN_FA1R_FACT1_Pos (1U)
#define CAN_FA1R_FACT1_Msk (0x1UL << CAN_FA1R_FACT1_Pos) /*!< 0x00000002 */
#define CAN_FA1R_FACT1 CAN_FA1R_FACT1_Msk                /*!<Filter Active bit 1 */
#define CAN_FA1R_FACT2_Pos (2U)
#define CAN_FA1R_FACT2_Msk (0x1UL << CAN_FA1R_FACT2_Pos) /*!< 0x00000004 */
#define CAN_FA1R_FACT2 CAN_FA1R_FACT2_Msk                /*!<Filter Active bit 2 */
#define CAN_FA1R_FACT3_Pos (3U)
#define CAN_FA1R_FACT3_Msk (0x1UL << CAN_FA1R_FACT3_Pos) /*!< 0x00000008 */
#define CAN_FA1R_FACT3 CAN_FA1R_FACT3_Msk                /*!<Filter Active bit 3 */
#define CAN_FA1R_FACT4_Pos (4U)
#define CAN_FA1R_FACT4_Msk (0x1UL << CAN_FA1R_FACT4_Pos) /*!< 0x00000010 */
#define CAN_FA1R_FACT4 CAN_FA1R_FACT4_Msk                /*!<Filter Active bit 4 */
#define CAN_FA1R_FACT5_Pos (5U)
#define CAN_FA1R_FACT5_Msk (0x1UL << CAN_FA1R_FACT5_Pos) /*!< 0x00000020 */
#define CAN_FA1R_FACT5 CAN_FA1R_FACT5_Msk                /*!<Filter Active bit 5 */
#define CAN_FA1R_FACT6_Pos (6U)
#define CAN_FA1R_FACT6_Msk (0x1UL << CAN_FA1R_FACT6_Pos) /*!< 0x00000040 */
#define CAN_FA1R_FACT6 CAN_FA1R_FACT6_Msk                /*!<Filter Active bit 6 */
#define CAN_FA1R_FACT7_Pos (7U)
#define CAN_FA1R_FACT7_Msk (0x1UL << CAN_FA1R_FACT7_Pos) /*!< 0x00000080 */
#define CAN_FA1R_FACT7 CAN_FA1R_FACT7_Msk                /*!<Filter Active bit 7 */
#define CAN_FA1R_FACT8_Pos (8U)
#define CAN_FA1R_FACT8_Msk (0x1UL << CAN_FA1R_FACT8_Pos) /*!< 0x00000100 */
#define CAN_FA1R_FACT8 CAN_FA1R_FACT8_Msk                /*!<Filter Active bit 8 */
#define CAN_FA1R_FACT9_Pos (9U)
#define CAN_FA1R_FACT9_Msk (0x1UL << CAN_FA1R_FACT9_Pos) /*!< 0x00000200 */
#define CAN_FA1R_FACT9 CAN_FA1R_FACT9_Msk                /*!<Filter Active bit 9 */
#define CAN_FA1R_FACT10_Pos (10U)
#define CAN_FA1R_FACT10_Msk (0x1UL << CAN_FA1R_FACT10_Pos) /*!< 0x00000400 */
#define CAN_FA1R_FACT10 CAN_FA1R_FACT10_Msk                /*!<Filter Active bit 10 */
#define CAN_FA1R_FACT11_Pos (11U)
#define CAN_FA1R_FACT11_Msk (0x1UL << CAN_FA1R_FACT11_Pos) /*!< 0x00000800 */
#define CAN_FA1R_FACT11 CAN_FA1R_FACT11_Msk                /*!<Filter Active bit 11 */
#define CAN_FA1R_FACT12_Pos (12U)
#define CAN_FA1R_FACT12_Msk (0x1UL << CAN_FA1R_FACT12_Pos) /*!< 0x00001000 */
#define CAN_FA1R_FACT12 CAN_FA1R_FACT12_Msk                /*!<Filter Active bit 12 */
#define CAN_FA1R_FACT13_Pos (13U)
#define CAN_FA1R_FACT13_Msk (0x1UL << CAN_FA1R_FACT13_Pos) /*!< 0x00002000 */
#define CAN_FA1R_FACT13 CAN_FA1R_FACT13_Msk                /*!<Filter Active bit 13 */
#define CAN_FA1R_FACT14_Pos (14U)
#define CAN_FA1R_FACT14_Msk (0x1UL << CAN_FA1R_FACT14_Pos) /*!< 0x00004000 */
#define CAN_FA1R_FACT14 CAN_FA1R_FACT14_Msk                /*!<Filter Active bit 14 */
#define CAN_FA1R_FACT15_Pos (15U)
#define CAN_FA1R_FACT15_Msk (0x1UL << CAN_FA1R_FACT15_Pos) /*!< 0x00008000 */
#define CAN_FA1R_FACT15 CAN_FA1R_FACT15_Msk                /*!<Filter Active bit 15 */
#define CAN_FA1R_FACT16_Pos (16U)
#define CAN_FA1R_FACT16_Msk (0x1UL << CAN_FA1R_FACT16_Pos) /*!< 0x00010000 */
#define CAN_FA1R_FACT16 CAN_FA1R_FACT16_Msk                /*!<Filter Active bit 16 */
#define CAN_FA1R_FACT17_Pos (17U)
#define CAN_FA1R_FACT17_Msk (0x1UL << CAN_FA1R_FACT17_Pos) /*!< 0x00020000 */
#define CAN_FA1R_FACT17 CAN_FA1R_FACT17_Msk                /*!<Filter Active bit 17 */
#define CAN_FA1R_FACT18_Pos (18U)
#define CAN_FA1R_FACT18_Msk (0x1UL << CAN_FA1R_FACT18_Pos) /*!< 0x00040000 */
#define CAN_FA1R_FACT18 CAN_FA1R_FACT18_Msk                /*!<Filter Active bit 18 */
#define CAN_FA1R_FACT19_Pos (19U)
#define CAN_FA1R_FACT19_Msk (0x1UL << CAN_FA1R_FACT19_Pos) /*!< 0x00080000 */
#define CAN_FA1R_FACT19 CAN_FA1R_FACT19_Msk                /*!<Filter Active bit 19 */
#define CAN_FA1R_FACT20_Pos (20U)
#define CAN_FA1R_FACT20_Msk (0x1UL << CAN_FA1R_FACT20_Pos) /*!< 0x00100000 */
#define CAN_FA1R_FACT20 CAN_FA1R_FACT20_Msk                /*!<Filter Active bit 20 */
#define CAN_FA1R_FACT21_Pos (21U)
#define CAN_FA1R_FACT21_Msk (0x1UL << CAN_FA1R_FACT21_Pos) /*!< 0x00200000 */
#define CAN_FA1R_FACT21 CAN_FA1R_FACT21_Msk                /*!<Filter Active bit 21 */
#define CAN_FA1R_FACT22_Pos (22U)
#define CAN_FA1R_FACT22_Msk (0x1UL << CAN_FA1R_FACT22_Pos) /*!< 0x00400000 */
#define CAN_FA1R_FACT22 CAN_FA1R_FACT22_Msk                /*!<Filter Active bit 22 */
#define CAN_FA1R_FACT23_Pos (23U)
#define CAN_FA1R_FACT23_Msk (0x1UL << CAN_FA1R_FACT23_Pos) /*!< 0x00800000 */
#define CAN_FA1R_FACT23 CAN_FA1R_FACT23_Msk                /*!<Filter Active bit 23 */
#define CAN_FA1R_FACT24_Pos (24U)
#define CAN_FA1R_FACT24_Msk (0x1UL << CAN_FA1R_FACT24_Pos) /*!< 0x01000000 */
#define CAN_FA1R_FACT24 CAN_FA1R_FACT24_Msk                /*!<Filter Active bit 24 */
#define CAN_FA1R_FACT25_Pos (25U)
#define CAN_FA1R_FACT25_Msk (0x1UL << CAN_FA1R_FACT25_Pos) /*!< 0x02000000 */
#define CAN_FA1R_FACT25 CAN_FA1R_FACT25_Msk                /*!<Filter Active bit 25 */
#define CAN_FA1R_FACT26_Pos (26U)
#define CAN_FA1R_FACT26_Msk (0x1UL << CAN_FA1R_FACT26_Pos) /*!< 0x04000000 */
#define CAN_FA1R_FACT26 CAN_FA1R_FACT26_Msk                /*!<Filter Active bit 26 */
#define CAN_FA1R_FACT27_Pos (27U)
#define CAN_FA1R_FACT27_Msk (0x1UL << CAN_FA1R_FACT27_Pos) /*!< 0x08000000 */
#define CAN_FA1R_FACT27 CAN_FA1R_FACT27_Msk                /*!<Filter Active bit 27 */

/*******************  Bit definition for CAN_F0R1 register  *******************/
#define CAN_F0R1_FB0_Pos (0U)
#define CAN_F0R1_FB0_Msk (0x1UL << CAN_F0R1_FB0_Pos) /*!< 0x00000001 */
#define CAN_F0R1_FB0 CAN_F0R1_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F0R1_FB1_Pos (1U)
#define CAN_F0R1_FB1_Msk (0x1UL << CAN_F0R1_FB1_Pos) /*!< 0x00000002 */
#define CAN_F0R1_FB1 CAN_F0R1_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F0R1_FB2_Pos (2U)
#define CAN_F0R1_FB2_Msk (0x1UL << CAN_F0R1_FB2_Pos) /*!< 0x00000004 */
#define CAN_F0R1_FB2 CAN_F0R1_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F0R1_FB3_Pos (3U)
#define CAN_F0R1_FB3_Msk (0x1UL << CAN_F0R1_FB3_Pos) /*!< 0x00000008 */
#define CAN_F0R1_FB3 CAN_F0R1_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F0R1_FB4_Pos (4U)
#define CAN_F0R1_FB4_Msk (0x1UL << CAN_F0R1_FB4_Pos) /*!< 0x00000010 */
#define CAN_F0R1_FB4 CAN_F0R1_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F0R1_FB5_Pos (5U)
#define CAN_F0R1_FB5_Msk (0x1UL << CAN_F0R1_FB5_Pos) /*!< 0x00000020 */
#define CAN_F0R1_FB5 CAN_F0R1_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F0R1_FB6_Pos (6U)
#define CAN_F0R1_FB6_Msk (0x1UL << CAN_F0R1_FB6_Pos) /*!< 0x00000040 */
#define CAN_F0R1_FB6 CAN_F0R1_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F0R1_FB7_Pos (7U)
#define CAN_F0R1_FB7_Msk (0x1UL << CAN_F0R1_FB7_Pos) /*!< 0x00000080 */
#define CAN_F0R1_FB7 CAN_F0R1_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F0R1_FB8_Pos (8U)
#define CAN_F0R1_FB8_Msk (0x1UL << CAN_F0R1_FB8_Pos) /*!< 0x00000100 */
#define CAN_F0R1_FB8 CAN_F0R1_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F0R1_FB9_Pos (9U)
#define CAN_F0R1_FB9_Msk (0x1UL << CAN_F0R1_FB9_Pos) /*!< 0x00000200 */
#define CAN_F0R1_FB9 CAN_F0R1_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F0R1_FB10_Pos (10U)
#define CAN_F0R1_FB10_Msk (0x1UL << CAN_F0R1_FB10_Pos) /*!< 0x00000400 */
#define CAN_F0R1_FB10 CAN_F0R1_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F0R1_FB11_Pos (11U)
#define CAN_F0R1_FB11_Msk (0x1UL << CAN_F0R1_FB11_Pos) /*!< 0x00000800 */
#define CAN_F0R1_FB11 CAN_F0R1_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F0R1_FB12_Pos (12U)
#define CAN_F0R1_FB12_Msk (0x1UL << CAN_F0R1_FB12_Pos) /*!< 0x00001000 */
#define CAN_F0R1_FB12 CAN_F0R1_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F0R1_FB13_Pos (13U)
#define CAN_F0R1_FB13_Msk (0x1UL << CAN_F0R1_FB13_Pos) /*!< 0x00002000 */
#define CAN_F0R1_FB13 CAN_F0R1_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F0R1_FB14_Pos (14U)
#define CAN_F0R1_FB14_Msk (0x1UL << CAN_F0R1_FB14_Pos) /*!< 0x00004000 */
#define CAN_F0R1_FB14 CAN_F0R1_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F0R1_FB15_Pos (15U)
#define CAN_F0R1_FB15_Msk (0x1UL << CAN_F0R1_FB15_Pos) /*!< 0x00008000 */
#define CAN_F0R1_FB15 CAN_F0R1_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F0R1_FB16_Pos (16U)
#define CAN_F0R1_FB16_Msk (0x1UL << CAN_F0R1_FB16_Pos) /*!< 0x00010000 */
#define CAN_F0R1_FB16 CAN_F0R1_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F0R1_FB17_Pos (17U)
#define CAN_F0R1_FB17_Msk (0x1UL << CAN_F0R1_FB17_Pos) /*!< 0x00020000 */
#define CAN_F0R1_FB17 CAN_F0R1_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F0R1_FB18_Pos (18U)
#define CAN_F0R1_FB18_Msk (0x1UL << CAN_F0R1_FB18_Pos) /*!< 0x00040000 */
#define CAN_F0R1_FB18 CAN_F0R1_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F0R1_FB19_Pos (19U)
#define CAN_F0R1_FB19_Msk (0x1UL << CAN_F0R1_FB19_Pos) /*!< 0x00080000 */
#define CAN_F0R1_FB19 CAN_F0R1_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F0R1_FB20_Pos (20U)
#define CAN_F0R1_FB20_Msk (0x1UL << CAN_F0R1_FB20_Pos) /*!< 0x00100000 */
#define CAN_F0R1_FB20 CAN_F0R1_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F0R1_FB21_Pos (21U)
#define CAN_F0R1_FB21_Msk (0x1UL << CAN_F0R1_FB21_Pos) /*!< 0x00200000 */
#define CAN_F0R1_FB21 CAN_F0R1_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F0R1_FB22_Pos (22U)
#define CAN_F0R1_FB22_Msk (0x1UL << CAN_F0R1_FB22_Pos) /*!< 0x00400000 */
#define CAN_F0R1_FB22 CAN_F0R1_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F0R1_FB23_Pos (23U)
#define CAN_F0R1_FB23_Msk (0x1UL << CAN_F0R1_FB23_Pos) /*!< 0x00800000 */
#define CAN_F0R1_FB23 CAN_F0R1_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F0R1_FB24_Pos (24U)
#define CAN_F0R1_FB24_Msk (0x1UL << CAN_F0R1_FB24_Pos) /*!< 0x01000000 */
#define CAN_F0R1_FB24 CAN_F0R1_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F0R1_FB25_Pos (25U)
#define CAN_F0R1_FB25_Msk (0x1UL << CAN_F0R1_FB25_Pos) /*!< 0x02000000 */
#define CAN_F0R1_FB25 CAN_F0R1_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F0R1_FB26_Pos (26U)
#define CAN_F0R1_FB26_Msk (0x1UL << CAN_F0R1_FB26_Pos) /*!< 0x04000000 */
#define CAN_F0R1_FB26 CAN_F0R1_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F0R1_FB27_Pos (27U)
#define CAN_F0R1_FB27_Msk (0x1UL << CAN_F0R1_FB27_Pos) /*!< 0x08000000 */
#define CAN_F0R1_FB27 CAN_F0R1_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F0R1_FB28_Pos (28U)
#define CAN_F0R1_FB28_Msk (0x1UL << CAN_F0R1_FB28_Pos) /*!< 0x10000000 */
#define CAN_F0R1_FB28 CAN_F0R1_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F0R1_FB29_Pos (29U)
#define CAN_F0R1_FB29_Msk (0x1UL << CAN_F0R1_FB29_Pos) /*!< 0x20000000 */
#define CAN_F0R1_FB29 CAN_F0R1_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F0R1_FB30_Pos (30U)
#define CAN_F0R1_FB30_Msk (0x1UL << CAN_F0R1_FB30_Pos) /*!< 0x40000000 */
#define CAN_F0R1_FB30 CAN_F0R1_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F0R1_FB31_Pos (31U)
#define CAN_F0R1_FB31_Msk (0x1UL << CAN_F0R1_FB31_Pos) /*!< 0x80000000 */
#define CAN_F0R1_FB31 CAN_F0R1_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F1R1 register  *******************/
#define CAN_F1R1_FB0_Pos (0U)
#define CAN_F1R1_FB0_Msk (0x1UL << CAN_F1R1_FB0_Pos) /*!< 0x00000001 */
#define CAN_F1R1_FB0 CAN_F1R1_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F1R1_FB1_Pos (1U)
#define CAN_F1R1_FB1_Msk (0x1UL << CAN_F1R1_FB1_Pos) /*!< 0x00000002 */
#define CAN_F1R1_FB1 CAN_F1R1_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F1R1_FB2_Pos (2U)
#define CAN_F1R1_FB2_Msk (0x1UL << CAN_F1R1_FB2_Pos) /*!< 0x00000004 */
#define CAN_F1R1_FB2 CAN_F1R1_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F1R1_FB3_Pos (3U)
#define CAN_F1R1_FB3_Msk (0x1UL << CAN_F1R1_FB3_Pos) /*!< 0x00000008 */
#define CAN_F1R1_FB3 CAN_F1R1_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F1R1_FB4_Pos (4U)
#define CAN_F1R1_FB4_Msk (0x1UL << CAN_F1R1_FB4_Pos) /*!< 0x00000010 */
#define CAN_F1R1_FB4 CAN_F1R1_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F1R1_FB5_Pos (5U)
#define CAN_F1R1_FB5_Msk (0x1UL << CAN_F1R1_FB5_Pos) /*!< 0x00000020 */
#define CAN_F1R1_FB5 CAN_F1R1_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F1R1_FB6_Pos (6U)
#define CAN_F1R1_FB6_Msk (0x1UL << CAN_F1R1_FB6_Pos) /*!< 0x00000040 */
#define CAN_F1R1_FB6 CAN_F1R1_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F1R1_FB7_Pos (7U)
#define CAN_F1R1_FB7_Msk (0x1UL << CAN_F1R1_FB7_Pos) /*!< 0x00000080 */
#define CAN_F1R1_FB7 CAN_F1R1_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F1R1_FB8_Pos (8U)
#define CAN_F1R1_FB8_Msk (0x1UL << CAN_F1R1_FB8_Pos) /*!< 0x00000100 */
#define CAN_F1R1_FB8 CAN_F1R1_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F1R1_FB9_Pos (9U)
#define CAN_F1R1_FB9_Msk (0x1UL << CAN_F1R1_FB9_Pos) /*!< 0x00000200 */
#define CAN_F1R1_FB9 CAN_F1R1_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F1R1_FB10_Pos (10U)
#define CAN_F1R1_FB10_Msk (0x1UL << CAN_F1R1_FB10_Pos) /*!< 0x00000400 */
#define CAN_F1R1_FB10 CAN_F1R1_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F1R1_FB11_Pos (11U)
#define CAN_F1R1_FB11_Msk (0x1UL << CAN_F1R1_FB11_Pos) /*!< 0x00000800 */
#define CAN_F1R1_FB11 CAN_F1R1_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F1R1_FB12_Pos (12U)
#define CAN_F1R1_FB12_Msk (0x1UL << CAN_F1R1_FB12_Pos) /*!< 0x00001000 */
#define CAN_F1R1_FB12 CAN_F1R1_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F1R1_FB13_Pos (13U)
#define CAN_F1R1_FB13_Msk (0x1UL << CAN_F1R1_FB13_Pos) /*!< 0x00002000 */
#define CAN_F1R1_FB13 CAN_F1R1_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F1R1_FB14_Pos (14U)
#define CAN_F1R1_FB14_Msk (0x1UL << CAN_F1R1_FB14_Pos) /*!< 0x00004000 */
#define CAN_F1R1_FB14 CAN_F1R1_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F1R1_FB15_Pos (15U)
#define CAN_F1R1_FB15_Msk (0x1UL << CAN_F1R1_FB15_Pos) /*!< 0x00008000 */
#define CAN_F1R1_FB15 CAN_F1R1_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F1R1_FB16_Pos (16U)
#define CAN_F1R1_FB16_Msk (0x1UL << CAN_F1R1_FB16_Pos) /*!< 0x00010000 */
#define CAN_F1R1_FB16 CAN_F1R1_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F1R1_FB17_Pos (17U)
#define CAN_F1R1_FB17_Msk (0x1UL << CAN_F1R1_FB17_Pos) /*!< 0x00020000 */
#define CAN_F1R1_FB17 CAN_F1R1_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F1R1_FB18_Pos (18U)
#define CAN_F1R1_FB18_Msk (0x1UL << CAN_F1R1_FB18_Pos) /*!< 0x00040000 */
#define CAN_F1R1_FB18 CAN_F1R1_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F1R1_FB19_Pos (19U)
#define CAN_F1R1_FB19_Msk (0x1UL << CAN_F1R1_FB19_Pos) /*!< 0x00080000 */
#define CAN_F1R1_FB19 CAN_F1R1_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F1R1_FB20_Pos (20U)
#define CAN_F1R1_FB20_Msk (0x1UL << CAN_F1R1_FB20_Pos) /*!< 0x00100000 */
#define CAN_F1R1_FB20 CAN_F1R1_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F1R1_FB21_Pos (21U)
#define CAN_F1R1_FB21_Msk (0x1UL << CAN_F1R1_FB21_Pos) /*!< 0x00200000 */
#define CAN_F1R1_FB21 CAN_F1R1_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F1R1_FB22_Pos (22U)
#define CAN_F1R1_FB22_Msk (0x1UL << CAN_F1R1_FB22_Pos) /*!< 0x00400000 */
#define CAN_F1R1_FB22 CAN_F1R1_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F1R1_FB23_Pos (23U)
#define CAN_F1R1_FB23_Msk (0x1UL << CAN_F1R1_FB23_Pos) /*!< 0x00800000 */
#define CAN_F1R1_FB23 CAN_F1R1_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F1R1_FB24_Pos (24U)
#define CAN_F1R1_FB24_Msk (0x1UL << CAN_F1R1_FB24_Pos) /*!< 0x01000000 */
#define CAN_F1R1_FB24 CAN_F1R1_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F1R1_FB25_Pos (25U)
#define CAN_F1R1_FB25_Msk (0x1UL << CAN_F1R1_FB25_Pos) /*!< 0x02000000 */
#define CAN_F1R1_FB25 CAN_F1R1_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F1R1_FB26_Pos (26U)
#define CAN_F1R1_FB26_Msk (0x1UL << CAN_F1R1_FB26_Pos) /*!< 0x04000000 */
#define CAN_F1R1_FB26 CAN_F1R1_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F1R1_FB27_Pos (27U)
#define CAN_F1R1_FB27_Msk (0x1UL << CAN_F1R1_FB27_Pos) /*!< 0x08000000 */
#define CAN_F1R1_FB27 CAN_F1R1_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F1R1_FB28_Pos (28U)
#define CAN_F1R1_FB28_Msk (0x1UL << CAN_F1R1_FB28_Pos) /*!< 0x10000000 */
#define CAN_F1R1_FB28 CAN_F1R1_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F1R1_FB29_Pos (29U)
#define CAN_F1R1_FB29_Msk (0x1UL << CAN_F1R1_FB29_Pos) /*!< 0x20000000 */
#define CAN_F1R1_FB29 CAN_F1R1_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F1R1_FB30_Pos (30U)
#define CAN_F1R1_FB30_Msk (0x1UL << CAN_F1R1_FB30_Pos) /*!< 0x40000000 */
#define CAN_F1R1_FB30 CAN_F1R1_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F1R1_FB31_Pos (31U)
#define CAN_F1R1_FB31_Msk (0x1UL << CAN_F1R1_FB31_Pos) /*!< 0x80000000 */
#define CAN_F1R1_FB31 CAN_F1R1_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F2R1 register  *******************/
#define CAN_F2R1_FB0_Pos (0U)
#define CAN_F2R1_FB0_Msk (0x1UL << CAN_F2R1_FB0_Pos) /*!< 0x00000001 */
#define CAN_F2R1_FB0 CAN_F2R1_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F2R1_FB1_Pos (1U)
#define CAN_F2R1_FB1_Msk (0x1UL << CAN_F2R1_FB1_Pos) /*!< 0x00000002 */
#define CAN_F2R1_FB1 CAN_F2R1_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F2R1_FB2_Pos (2U)
#define CAN_F2R1_FB2_Msk (0x1UL << CAN_F2R1_FB2_Pos) /*!< 0x00000004 */
#define CAN_F2R1_FB2 CAN_F2R1_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F2R1_FB3_Pos (3U)
#define CAN_F2R1_FB3_Msk (0x1UL << CAN_F2R1_FB3_Pos) /*!< 0x00000008 */
#define CAN_F2R1_FB3 CAN_F2R1_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F2R1_FB4_Pos (4U)
#define CAN_F2R1_FB4_Msk (0x1UL << CAN_F2R1_FB4_Pos) /*!< 0x00000010 */
#define CAN_F2R1_FB4 CAN_F2R1_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F2R1_FB5_Pos (5U)
#define CAN_F2R1_FB5_Msk (0x1UL << CAN_F2R1_FB5_Pos) /*!< 0x00000020 */
#define CAN_F2R1_FB5 CAN_F2R1_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F2R1_FB6_Pos (6U)
#define CAN_F2R1_FB6_Msk (0x1UL << CAN_F2R1_FB6_Pos) /*!< 0x00000040 */
#define CAN_F2R1_FB6 CAN_F2R1_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F2R1_FB7_Pos (7U)
#define CAN_F2R1_FB7_Msk (0x1UL << CAN_F2R1_FB7_Pos) /*!< 0x00000080 */
#define CAN_F2R1_FB7 CAN_F2R1_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F2R1_FB8_Pos (8U)
#define CAN_F2R1_FB8_Msk (0x1UL << CAN_F2R1_FB8_Pos) /*!< 0x00000100 */
#define CAN_F2R1_FB8 CAN_F2R1_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F2R1_FB9_Pos (9U)
#define CAN_F2R1_FB9_Msk (0x1UL << CAN_F2R1_FB9_Pos) /*!< 0x00000200 */
#define CAN_F2R1_FB9 CAN_F2R1_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F2R1_FB10_Pos (10U)
#define CAN_F2R1_FB10_Msk (0x1UL << CAN_F2R1_FB10_Pos) /*!< 0x00000400 */
#define CAN_F2R1_FB10 CAN_F2R1_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F2R1_FB11_Pos (11U)
#define CAN_F2R1_FB11_Msk (0x1UL << CAN_F2R1_FB11_Pos) /*!< 0x00000800 */
#define CAN_F2R1_FB11 CAN_F2R1_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F2R1_FB12_Pos (12U)
#define CAN_F2R1_FB12_Msk (0x1UL << CAN_F2R1_FB12_Pos) /*!< 0x00001000 */
#define CAN_F2R1_FB12 CAN_F2R1_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F2R1_FB13_Pos (13U)
#define CAN_F2R1_FB13_Msk (0x1UL << CAN_F2R1_FB13_Pos) /*!< 0x00002000 */
#define CAN_F2R1_FB13 CAN_F2R1_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F2R1_FB14_Pos (14U)
#define CAN_F2R1_FB14_Msk (0x1UL << CAN_F2R1_FB14_Pos) /*!< 0x00004000 */
#define CAN_F2R1_FB14 CAN_F2R1_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F2R1_FB15_Pos (15U)
#define CAN_F2R1_FB15_Msk (0x1UL << CAN_F2R1_FB15_Pos) /*!< 0x00008000 */
#define CAN_F2R1_FB15 CAN_F2R1_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F2R1_FB16_Pos (16U)
#define CAN_F2R1_FB16_Msk (0x1UL << CAN_F2R1_FB16_Pos) /*!< 0x00010000 */
#define CAN_F2R1_FB16 CAN_F2R1_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F2R1_FB17_Pos (17U)
#define CAN_F2R1_FB17_Msk (0x1UL << CAN_F2R1_FB17_Pos) /*!< 0x00020000 */
#define CAN_F2R1_FB17 CAN_F2R1_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F2R1_FB18_Pos (18U)
#define CAN_F2R1_FB18_Msk (0x1UL << CAN_F2R1_FB18_Pos) /*!< 0x00040000 */
#define CAN_F2R1_FB18 CAN_F2R1_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F2R1_FB19_Pos (19U)
#define CAN_F2R1_FB19_Msk (0x1UL << CAN_F2R1_FB19_Pos) /*!< 0x00080000 */
#define CAN_F2R1_FB19 CAN_F2R1_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F2R1_FB20_Pos (20U)
#define CAN_F2R1_FB20_Msk (0x1UL << CAN_F2R1_FB20_Pos) /*!< 0x00100000 */
#define CAN_F2R1_FB20 CAN_F2R1_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F2R1_FB21_Pos (21U)
#define CAN_F2R1_FB21_Msk (0x1UL << CAN_F2R1_FB21_Pos) /*!< 0x00200000 */
#define CAN_F2R1_FB21 CAN_F2R1_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F2R1_FB22_Pos (22U)
#define CAN_F2R1_FB22_Msk (0x1UL << CAN_F2R1_FB22_Pos) /*!< 0x00400000 */
#define CAN_F2R1_FB22 CAN_F2R1_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F2R1_FB23_Pos (23U)
#define CAN_F2R1_FB23_Msk (0x1UL << CAN_F2R1_FB23_Pos) /*!< 0x00800000 */
#define CAN_F2R1_FB23 CAN_F2R1_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F2R1_FB24_Pos (24U)
#define CAN_F2R1_FB24_Msk (0x1UL << CAN_F2R1_FB24_Pos) /*!< 0x01000000 */
#define CAN_F2R1_FB24 CAN_F2R1_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F2R1_FB25_Pos (25U)
#define CAN_F2R1_FB25_Msk (0x1UL << CAN_F2R1_FB25_Pos) /*!< 0x02000000 */
#define CAN_F2R1_FB25 CAN_F2R1_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F2R1_FB26_Pos (26U)
#define CAN_F2R1_FB26_Msk (0x1UL << CAN_F2R1_FB26_Pos) /*!< 0x04000000 */
#define CAN_F2R1_FB26 CAN_F2R1_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F2R1_FB27_Pos (27U)
#define CAN_F2R1_FB27_Msk (0x1UL << CAN_F2R1_FB27_Pos) /*!< 0x08000000 */
#define CAN_F2R1_FB27 CAN_F2R1_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F2R1_FB28_Pos (28U)
#define CAN_F2R1_FB28_Msk (0x1UL << CAN_F2R1_FB28_Pos) /*!< 0x10000000 */
#define CAN_F2R1_FB28 CAN_F2R1_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F2R1_FB29_Pos (29U)
#define CAN_F2R1_FB29_Msk (0x1UL << CAN_F2R1_FB29_Pos) /*!< 0x20000000 */
#define CAN_F2R1_FB29 CAN_F2R1_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F2R1_FB30_Pos (30U)
#define CAN_F2R1_FB30_Msk (0x1UL << CAN_F2R1_FB30_Pos) /*!< 0x40000000 */
#define CAN_F2R1_FB30 CAN_F2R1_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F2R1_FB31_Pos (31U)
#define CAN_F2R1_FB31_Msk (0x1UL << CAN_F2R1_FB31_Pos) /*!< 0x80000000 */
#define CAN_F2R1_FB31 CAN_F2R1_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F3R1 register  *******************/
#define CAN_F3R1_FB0_Pos (0U)
#define CAN_F3R1_FB0_Msk (0x1UL << CAN_F3R1_FB0_Pos) /*!< 0x00000001 */
#define CAN_F3R1_FB0 CAN_F3R1_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F3R1_FB1_Pos (1U)
#define CAN_F3R1_FB1_Msk (0x1UL << CAN_F3R1_FB1_Pos) /*!< 0x00000002 */
#define CAN_F3R1_FB1 CAN_F3R1_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F3R1_FB2_Pos (2U)
#define CAN_F3R1_FB2_Msk (0x1UL << CAN_F3R1_FB2_Pos) /*!< 0x00000004 */
#define CAN_F3R1_FB2 CAN_F3R1_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F3R1_FB3_Pos (3U)
#define CAN_F3R1_FB3_Msk (0x1UL << CAN_F3R1_FB3_Pos) /*!< 0x00000008 */
#define CAN_F3R1_FB3 CAN_F3R1_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F3R1_FB4_Pos (4U)
#define CAN_F3R1_FB4_Msk (0x1UL << CAN_F3R1_FB4_Pos) /*!< 0x00000010 */
#define CAN_F3R1_FB4 CAN_F3R1_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F3R1_FB5_Pos (5U)
#define CAN_F3R1_FB5_Msk (0x1UL << CAN_F3R1_FB5_Pos) /*!< 0x00000020 */
#define CAN_F3R1_FB5 CAN_F3R1_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F3R1_FB6_Pos (6U)
#define CAN_F3R1_FB6_Msk (0x1UL << CAN_F3R1_FB6_Pos) /*!< 0x00000040 */
#define CAN_F3R1_FB6 CAN_F3R1_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F3R1_FB7_Pos (7U)
#define CAN_F3R1_FB7_Msk (0x1UL << CAN_F3R1_FB7_Pos) /*!< 0x00000080 */
#define CAN_F3R1_FB7 CAN_F3R1_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F3R1_FB8_Pos (8U)
#define CAN_F3R1_FB8_Msk (0x1UL << CAN_F3R1_FB8_Pos) /*!< 0x00000100 */
#define CAN_F3R1_FB8 CAN_F3R1_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F3R1_FB9_Pos (9U)
#define CAN_F3R1_FB9_Msk (0x1UL << CAN_F3R1_FB9_Pos) /*!< 0x00000200 */
#define CAN_F3R1_FB9 CAN_F3R1_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F3R1_FB10_Pos (10U)
#define CAN_F3R1_FB10_Msk (0x1UL << CAN_F3R1_FB10_Pos) /*!< 0x00000400 */
#define CAN_F3R1_FB10 CAN_F3R1_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F3R1_FB11_Pos (11U)
#define CAN_F3R1_FB11_Msk (0x1UL << CAN_F3R1_FB11_Pos) /*!< 0x00000800 */
#define CAN_F3R1_FB11 CAN_F3R1_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F3R1_FB12_Pos (12U)
#define CAN_F3R1_FB12_Msk (0x1UL << CAN_F3R1_FB12_Pos) /*!< 0x00001000 */
#define CAN_F3R1_FB12 CAN_F3R1_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F3R1_FB13_Pos (13U)
#define CAN_F3R1_FB13_Msk (0x1UL << CAN_F3R1_FB13_Pos) /*!< 0x00002000 */
#define CAN_F3R1_FB13 CAN_F3R1_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F3R1_FB14_Pos (14U)
#define CAN_F3R1_FB14_Msk (0x1UL << CAN_F3R1_FB14_Pos) /*!< 0x00004000 */
#define CAN_F3R1_FB14 CAN_F3R1_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F3R1_FB15_Pos (15U)
#define CAN_F3R1_FB15_Msk (0x1UL << CAN_F3R1_FB15_Pos) /*!< 0x00008000 */
#define CAN_F3R1_FB15 CAN_F3R1_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F3R1_FB16_Pos (16U)
#define CAN_F3R1_FB16_Msk (0x1UL << CAN_F3R1_FB16_Pos) /*!< 0x00010000 */
#define CAN_F3R1_FB16 CAN_F3R1_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F3R1_FB17_Pos (17U)
#define CAN_F3R1_FB17_Msk (0x1UL << CAN_F3R1_FB17_Pos) /*!< 0x00020000 */
#define CAN_F3R1_FB17 CAN_F3R1_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F3R1_FB18_Pos (18U)
#define CAN_F3R1_FB18_Msk (0x1UL << CAN_F3R1_FB18_Pos) /*!< 0x00040000 */
#define CAN_F3R1_FB18 CAN_F3R1_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F3R1_FB19_Pos (19U)
#define CAN_F3R1_FB19_Msk (0x1UL << CAN_F3R1_FB19_Pos) /*!< 0x00080000 */
#define CAN_F3R1_FB19 CAN_F3R1_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F3R1_FB20_Pos (20U)
#define CAN_F3R1_FB20_Msk (0x1UL << CAN_F3R1_FB20_Pos) /*!< 0x00100000 */
#define CAN_F3R1_FB20 CAN_F3R1_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F3R1_FB21_Pos (21U)
#define CAN_F3R1_FB21_Msk (0x1UL << CAN_F3R1_FB21_Pos) /*!< 0x00200000 */
#define CAN_F3R1_FB21 CAN_F3R1_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F3R1_FB22_Pos (22U)
#define CAN_F3R1_FB22_Msk (0x1UL << CAN_F3R1_FB22_Pos) /*!< 0x00400000 */
#define CAN_F3R1_FB22 CAN_F3R1_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F3R1_FB23_Pos (23U)
#define CAN_F3R1_FB23_Msk (0x1UL << CAN_F3R1_FB23_Pos) /*!< 0x00800000 */
#define CAN_F3R1_FB23 CAN_F3R1_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F3R1_FB24_Pos (24U)
#define CAN_F3R1_FB24_Msk (0x1UL << CAN_F3R1_FB24_Pos) /*!< 0x01000000 */
#define CAN_F3R1_FB24 CAN_F3R1_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F3R1_FB25_Pos (25U)
#define CAN_F3R1_FB25_Msk (0x1UL << CAN_F3R1_FB25_Pos) /*!< 0x02000000 */
#define CAN_F3R1_FB25 CAN_F3R1_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F3R1_FB26_Pos (26U)
#define CAN_F3R1_FB26_Msk (0x1UL << CAN_F3R1_FB26_Pos) /*!< 0x04000000 */
#define CAN_F3R1_FB26 CAN_F3R1_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F3R1_FB27_Pos (27U)
#define CAN_F3R1_FB27_Msk (0x1UL << CAN_F3R1_FB27_Pos) /*!< 0x08000000 */
#define CAN_F3R1_FB27 CAN_F3R1_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F3R1_FB28_Pos (28U)
#define CAN_F3R1_FB28_Msk (0x1UL << CAN_F3R1_FB28_Pos) /*!< 0x10000000 */
#define CAN_F3R1_FB28 CAN_F3R1_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F3R1_FB29_Pos (29U)
#define CAN_F3R1_FB29_Msk (0x1UL << CAN_F3R1_FB29_Pos) /*!< 0x20000000 */
#define CAN_F3R1_FB29 CAN_F3R1_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F3R1_FB30_Pos (30U)
#define CAN_F3R1_FB30_Msk (0x1UL << CAN_F3R1_FB30_Pos) /*!< 0x40000000 */
#define CAN_F3R1_FB30 CAN_F3R1_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F3R1_FB31_Pos (31U)
#define CAN_F3R1_FB31_Msk (0x1UL << CAN_F3R1_FB31_Pos) /*!< 0x80000000 */
#define CAN_F3R1_FB31 CAN_F3R1_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F4R1 register  *******************/
#define CAN_F4R1_FB0_Pos (0U)
#define CAN_F4R1_FB0_Msk (0x1UL << CAN_F4R1_FB0_Pos) /*!< 0x00000001 */
#define CAN_F4R1_FB0 CAN_F4R1_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F4R1_FB1_Pos (1U)
#define CAN_F4R1_FB1_Msk (0x1UL << CAN_F4R1_FB1_Pos) /*!< 0x00000002 */
#define CAN_F4R1_FB1 CAN_F4R1_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F4R1_FB2_Pos (2U)
#define CAN_F4R1_FB2_Msk (0x1UL << CAN_F4R1_FB2_Pos) /*!< 0x00000004 */
#define CAN_F4R1_FB2 CAN_F4R1_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F4R1_FB3_Pos (3U)
#define CAN_F4R1_FB3_Msk (0x1UL << CAN_F4R1_FB3_Pos) /*!< 0x00000008 */
#define CAN_F4R1_FB3 CAN_F4R1_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F4R1_FB4_Pos (4U)
#define CAN_F4R1_FB4_Msk (0x1UL << CAN_F4R1_FB4_Pos) /*!< 0x00000010 */
#define CAN_F4R1_FB4 CAN_F4R1_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F4R1_FB5_Pos (5U)
#define CAN_F4R1_FB5_Msk (0x1UL << CAN_F4R1_FB5_Pos) /*!< 0x00000020 */
#define CAN_F4R1_FB5 CAN_F4R1_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F4R1_FB6_Pos (6U)
#define CAN_F4R1_FB6_Msk (0x1UL << CAN_F4R1_FB6_Pos) /*!< 0x00000040 */
#define CAN_F4R1_FB6 CAN_F4R1_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F4R1_FB7_Pos (7U)
#define CAN_F4R1_FB7_Msk (0x1UL << CAN_F4R1_FB7_Pos) /*!< 0x00000080 */
#define CAN_F4R1_FB7 CAN_F4R1_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F4R1_FB8_Pos (8U)
#define CAN_F4R1_FB8_Msk (0x1UL << CAN_F4R1_FB8_Pos) /*!< 0x00000100 */
#define CAN_F4R1_FB8 CAN_F4R1_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F4R1_FB9_Pos (9U)
#define CAN_F4R1_FB9_Msk (0x1UL << CAN_F4R1_FB9_Pos) /*!< 0x00000200 */
#define CAN_F4R1_FB9 CAN_F4R1_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F4R1_FB10_Pos (10U)
#define CAN_F4R1_FB10_Msk (0x1UL << CAN_F4R1_FB10_Pos) /*!< 0x00000400 */
#define CAN_F4R1_FB10 CAN_F4R1_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F4R1_FB11_Pos (11U)
#define CAN_F4R1_FB11_Msk (0x1UL << CAN_F4R1_FB11_Pos) /*!< 0x00000800 */
#define CAN_F4R1_FB11 CAN_F4R1_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F4R1_FB12_Pos (12U)
#define CAN_F4R1_FB12_Msk (0x1UL << CAN_F4R1_FB12_Pos) /*!< 0x00001000 */
#define CAN_F4R1_FB12 CAN_F4R1_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F4R1_FB13_Pos (13U)
#define CAN_F4R1_FB13_Msk (0x1UL << CAN_F4R1_FB13_Pos) /*!< 0x00002000 */
#define CAN_F4R1_FB13 CAN_F4R1_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F4R1_FB14_Pos (14U)
#define CAN_F4R1_FB14_Msk (0x1UL << CAN_F4R1_FB14_Pos) /*!< 0x00004000 */
#define CAN_F4R1_FB14 CAN_F4R1_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F4R1_FB15_Pos (15U)
#define CAN_F4R1_FB15_Msk (0x1UL << CAN_F4R1_FB15_Pos) /*!< 0x00008000 */
#define CAN_F4R1_FB15 CAN_F4R1_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F4R1_FB16_Pos (16U)
#define CAN_F4R1_FB16_Msk (0x1UL << CAN_F4R1_FB16_Pos) /*!< 0x00010000 */
#define CAN_F4R1_FB16 CAN_F4R1_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F4R1_FB17_Pos (17U)
#define CAN_F4R1_FB17_Msk (0x1UL << CAN_F4R1_FB17_Pos) /*!< 0x00020000 */
#define CAN_F4R1_FB17 CAN_F4R1_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F4R1_FB18_Pos (18U)
#define CAN_F4R1_FB18_Msk (0x1UL << CAN_F4R1_FB18_Pos) /*!< 0x00040000 */
#define CAN_F4R1_FB18 CAN_F4R1_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F4R1_FB19_Pos (19U)
#define CAN_F4R1_FB19_Msk (0x1UL << CAN_F4R1_FB19_Pos) /*!< 0x00080000 */
#define CAN_F4R1_FB19 CAN_F4R1_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F4R1_FB20_Pos (20U)
#define CAN_F4R1_FB20_Msk (0x1UL << CAN_F4R1_FB20_Pos) /*!< 0x00100000 */
#define CAN_F4R1_FB20 CAN_F4R1_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F4R1_FB21_Pos (21U)
#define CAN_F4R1_FB21_Msk (0x1UL << CAN_F4R1_FB21_Pos) /*!< 0x00200000 */
#define CAN_F4R1_FB21 CAN_F4R1_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F4R1_FB22_Pos (22U)
#define CAN_F4R1_FB22_Msk (0x1UL << CAN_F4R1_FB22_Pos) /*!< 0x00400000 */
#define CAN_F4R1_FB22 CAN_F4R1_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F4R1_FB23_Pos (23U)
#define CAN_F4R1_FB23_Msk (0x1UL << CAN_F4R1_FB23_Pos) /*!< 0x00800000 */
#define CAN_F4R1_FB23 CAN_F4R1_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F4R1_FB24_Pos (24U)
#define CAN_F4R1_FB24_Msk (0x1UL << CAN_F4R1_FB24_Pos) /*!< 0x01000000 */
#define CAN_F4R1_FB24 CAN_F4R1_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F4R1_FB25_Pos (25U)
#define CAN_F4R1_FB25_Msk (0x1UL << CAN_F4R1_FB25_Pos) /*!< 0x02000000 */
#define CAN_F4R1_FB25 CAN_F4R1_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F4R1_FB26_Pos (26U)
#define CAN_F4R1_FB26_Msk (0x1UL << CAN_F4R1_FB26_Pos) /*!< 0x04000000 */
#define CAN_F4R1_FB26 CAN_F4R1_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F4R1_FB27_Pos (27U)
#define CAN_F4R1_FB27_Msk (0x1UL << CAN_F4R1_FB27_Pos) /*!< 0x08000000 */
#define CAN_F4R1_FB27 CAN_F4R1_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F4R1_FB28_Pos (28U)
#define CAN_F4R1_FB28_Msk (0x1UL << CAN_F4R1_FB28_Pos) /*!< 0x10000000 */
#define CAN_F4R1_FB28 CAN_F4R1_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F4R1_FB29_Pos (29U)
#define CAN_F4R1_FB29_Msk (0x1UL << CAN_F4R1_FB29_Pos) /*!< 0x20000000 */
#define CAN_F4R1_FB29 CAN_F4R1_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F4R1_FB30_Pos (30U)
#define CAN_F4R1_FB30_Msk (0x1UL << CAN_F4R1_FB30_Pos) /*!< 0x40000000 */
#define CAN_F4R1_FB30 CAN_F4R1_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F4R1_FB31_Pos (31U)
#define CAN_F4R1_FB31_Msk (0x1UL << CAN_F4R1_FB31_Pos) /*!< 0x80000000 */
#define CAN_F4R1_FB31 CAN_F4R1_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F5R1 register  *******************/
#define CAN_F5R1_FB0_Pos (0U)
#define CAN_F5R1_FB0_Msk (0x1UL << CAN_F5R1_FB0_Pos) /*!< 0x00000001 */
#define CAN_F5R1_FB0 CAN_F5R1_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F5R1_FB1_Pos (1U)
#define CAN_F5R1_FB1_Msk (0x1UL << CAN_F5R1_FB1_Pos) /*!< 0x00000002 */
#define CAN_F5R1_FB1 CAN_F5R1_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F5R1_FB2_Pos (2U)
#define CAN_F5R1_FB2_Msk (0x1UL << CAN_F5R1_FB2_Pos) /*!< 0x00000004 */
#define CAN_F5R1_FB2 CAN_F5R1_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F5R1_FB3_Pos (3U)
#define CAN_F5R1_FB3_Msk (0x1UL << CAN_F5R1_FB3_Pos) /*!< 0x00000008 */
#define CAN_F5R1_FB3 CAN_F5R1_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F5R1_FB4_Pos (4U)
#define CAN_F5R1_FB4_Msk (0x1UL << CAN_F5R1_FB4_Pos) /*!< 0x00000010 */
#define CAN_F5R1_FB4 CAN_F5R1_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F5R1_FB5_Pos (5U)
#define CAN_F5R1_FB5_Msk (0x1UL << CAN_F5R1_FB5_Pos) /*!< 0x00000020 */
#define CAN_F5R1_FB5 CAN_F5R1_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F5R1_FB6_Pos (6U)
#define CAN_F5R1_FB6_Msk (0x1UL << CAN_F5R1_FB6_Pos) /*!< 0x00000040 */
#define CAN_F5R1_FB6 CAN_F5R1_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F5R1_FB7_Pos (7U)
#define CAN_F5R1_FB7_Msk (0x1UL << CAN_F5R1_FB7_Pos) /*!< 0x00000080 */
#define CAN_F5R1_FB7 CAN_F5R1_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F5R1_FB8_Pos (8U)
#define CAN_F5R1_FB8_Msk (0x1UL << CAN_F5R1_FB8_Pos) /*!< 0x00000100 */
#define CAN_F5R1_FB8 CAN_F5R1_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F5R1_FB9_Pos (9U)
#define CAN_F5R1_FB9_Msk (0x1UL << CAN_F5R1_FB9_Pos) /*!< 0x00000200 */
#define CAN_F5R1_FB9 CAN_F5R1_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F5R1_FB10_Pos (10U)
#define CAN_F5R1_FB10_Msk (0x1UL << CAN_F5R1_FB10_Pos) /*!< 0x00000400 */
#define CAN_F5R1_FB10 CAN_F5R1_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F5R1_FB11_Pos (11U)
#define CAN_F5R1_FB11_Msk (0x1UL << CAN_F5R1_FB11_Pos) /*!< 0x00000800 */
#define CAN_F5R1_FB11 CAN_F5R1_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F5R1_FB12_Pos (12U)
#define CAN_F5R1_FB12_Msk (0x1UL << CAN_F5R1_FB12_Pos) /*!< 0x00001000 */
#define CAN_F5R1_FB12 CAN_F5R1_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F5R1_FB13_Pos (13U)
#define CAN_F5R1_FB13_Msk (0x1UL << CAN_F5R1_FB13_Pos) /*!< 0x00002000 */
#define CAN_F5R1_FB13 CAN_F5R1_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F5R1_FB14_Pos (14U)
#define CAN_F5R1_FB14_Msk (0x1UL << CAN_F5R1_FB14_Pos) /*!< 0x00004000 */
#define CAN_F5R1_FB14 CAN_F5R1_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F5R1_FB15_Pos (15U)
#define CAN_F5R1_FB15_Msk (0x1UL << CAN_F5R1_FB15_Pos) /*!< 0x00008000 */
#define CAN_F5R1_FB15 CAN_F5R1_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F5R1_FB16_Pos (16U)
#define CAN_F5R1_FB16_Msk (0x1UL << CAN_F5R1_FB16_Pos) /*!< 0x00010000 */
#define CAN_F5R1_FB16 CAN_F5R1_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F5R1_FB17_Pos (17U)
#define CAN_F5R1_FB17_Msk (0x1UL << CAN_F5R1_FB17_Pos) /*!< 0x00020000 */
#define CAN_F5R1_FB17 CAN_F5R1_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F5R1_FB18_Pos (18U)
#define CAN_F5R1_FB18_Msk (0x1UL << CAN_F5R1_FB18_Pos) /*!< 0x00040000 */
#define CAN_F5R1_FB18 CAN_F5R1_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F5R1_FB19_Pos (19U)
#define CAN_F5R1_FB19_Msk (0x1UL << CAN_F5R1_FB19_Pos) /*!< 0x00080000 */
#define CAN_F5R1_FB19 CAN_F5R1_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F5R1_FB20_Pos (20U)
#define CAN_F5R1_FB20_Msk (0x1UL << CAN_F5R1_FB20_Pos) /*!< 0x00100000 */
#define CAN_F5R1_FB20 CAN_F5R1_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F5R1_FB21_Pos (21U)
#define CAN_F5R1_FB21_Msk (0x1UL << CAN_F5R1_FB21_Pos) /*!< 0x00200000 */
#define CAN_F5R1_FB21 CAN_F5R1_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F5R1_FB22_Pos (22U)
#define CAN_F5R1_FB22_Msk (0x1UL << CAN_F5R1_FB22_Pos) /*!< 0x00400000 */
#define CAN_F5R1_FB22 CAN_F5R1_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F5R1_FB23_Pos (23U)
#define CAN_F5R1_FB23_Msk (0x1UL << CAN_F5R1_FB23_Pos) /*!< 0x00800000 */
#define CAN_F5R1_FB23 CAN_F5R1_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F5R1_FB24_Pos (24U)
#define CAN_F5R1_FB24_Msk (0x1UL << CAN_F5R1_FB24_Pos) /*!< 0x01000000 */
#define CAN_F5R1_FB24 CAN_F5R1_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F5R1_FB25_Pos (25U)
#define CAN_F5R1_FB25_Msk (0x1UL << CAN_F5R1_FB25_Pos) /*!< 0x02000000 */
#define CAN_F5R1_FB25 CAN_F5R1_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F5R1_FB26_Pos (26U)
#define CAN_F5R1_FB26_Msk (0x1UL << CAN_F5R1_FB26_Pos) /*!< 0x04000000 */
#define CAN_F5R1_FB26 CAN_F5R1_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F5R1_FB27_Pos (27U)
#define CAN_F5R1_FB27_Msk (0x1UL << CAN_F5R1_FB27_Pos) /*!< 0x08000000 */
#define CAN_F5R1_FB27 CAN_F5R1_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F5R1_FB28_Pos (28U)
#define CAN_F5R1_FB28_Msk (0x1UL << CAN_F5R1_FB28_Pos) /*!< 0x10000000 */
#define CAN_F5R1_FB28 CAN_F5R1_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F5R1_FB29_Pos (29U)
#define CAN_F5R1_FB29_Msk (0x1UL << CAN_F5R1_FB29_Pos) /*!< 0x20000000 */
#define CAN_F5R1_FB29 CAN_F5R1_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F5R1_FB30_Pos (30U)
#define CAN_F5R1_FB30_Msk (0x1UL << CAN_F5R1_FB30_Pos) /*!< 0x40000000 */
#define CAN_F5R1_FB30 CAN_F5R1_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F5R1_FB31_Pos (31U)
#define CAN_F5R1_FB31_Msk (0x1UL << CAN_F5R1_FB31_Pos) /*!< 0x80000000 */
#define CAN_F5R1_FB31 CAN_F5R1_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F6R1 register  *******************/
#define CAN_F6R1_FB0_Pos (0U)
#define CAN_F6R1_FB0_Msk (0x1UL << CAN_F6R1_FB0_Pos) /*!< 0x00000001 */
#define CAN_F6R1_FB0 CAN_F6R1_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F6R1_FB1_Pos (1U)
#define CAN_F6R1_FB1_Msk (0x1UL << CAN_F6R1_FB1_Pos) /*!< 0x00000002 */
#define CAN_F6R1_FB1 CAN_F6R1_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F6R1_FB2_Pos (2U)
#define CAN_F6R1_FB2_Msk (0x1UL << CAN_F6R1_FB2_Pos) /*!< 0x00000004 */
#define CAN_F6R1_FB2 CAN_F6R1_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F6R1_FB3_Pos (3U)
#define CAN_F6R1_FB3_Msk (0x1UL << CAN_F6R1_FB3_Pos) /*!< 0x00000008 */
#define CAN_F6R1_FB3 CAN_F6R1_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F6R1_FB4_Pos (4U)
#define CAN_F6R1_FB4_Msk (0x1UL << CAN_F6R1_FB4_Pos) /*!< 0x00000010 */
#define CAN_F6R1_FB4 CAN_F6R1_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F6R1_FB5_Pos (5U)
#define CAN_F6R1_FB5_Msk (0x1UL << CAN_F6R1_FB5_Pos) /*!< 0x00000020 */
#define CAN_F6R1_FB5 CAN_F6R1_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F6R1_FB6_Pos (6U)
#define CAN_F6R1_FB6_Msk (0x1UL << CAN_F6R1_FB6_Pos) /*!< 0x00000040 */
#define CAN_F6R1_FB6 CAN_F6R1_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F6R1_FB7_Pos (7U)
#define CAN_F6R1_FB7_Msk (0x1UL << CAN_F6R1_FB7_Pos) /*!< 0x00000080 */
#define CAN_F6R1_FB7 CAN_F6R1_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F6R1_FB8_Pos (8U)
#define CAN_F6R1_FB8_Msk (0x1UL << CAN_F6R1_FB8_Pos) /*!< 0x00000100 */
#define CAN_F6R1_FB8 CAN_F6R1_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F6R1_FB9_Pos (9U)
#define CAN_F6R1_FB9_Msk (0x1UL << CAN_F6R1_FB9_Pos) /*!< 0x00000200 */
#define CAN_F6R1_FB9 CAN_F6R1_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F6R1_FB10_Pos (10U)
#define CAN_F6R1_FB10_Msk (0x1UL << CAN_F6R1_FB10_Pos) /*!< 0x00000400 */
#define CAN_F6R1_FB10 CAN_F6R1_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F6R1_FB11_Pos (11U)
#define CAN_F6R1_FB11_Msk (0x1UL << CAN_F6R1_FB11_Pos) /*!< 0x00000800 */
#define CAN_F6R1_FB11 CAN_F6R1_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F6R1_FB12_Pos (12U)
#define CAN_F6R1_FB12_Msk (0x1UL << CAN_F6R1_FB12_Pos) /*!< 0x00001000 */
#define CAN_F6R1_FB12 CAN_F6R1_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F6R1_FB13_Pos (13U)
#define CAN_F6R1_FB13_Msk (0x1UL << CAN_F6R1_FB13_Pos) /*!< 0x00002000 */
#define CAN_F6R1_FB13 CAN_F6R1_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F6R1_FB14_Pos (14U)
#define CAN_F6R1_FB14_Msk (0x1UL << CAN_F6R1_FB14_Pos) /*!< 0x00004000 */
#define CAN_F6R1_FB14 CAN_F6R1_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F6R1_FB15_Pos (15U)
#define CAN_F6R1_FB15_Msk (0x1UL << CAN_F6R1_FB15_Pos) /*!< 0x00008000 */
#define CAN_F6R1_FB15 CAN_F6R1_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F6R1_FB16_Pos (16U)
#define CAN_F6R1_FB16_Msk (0x1UL << CAN_F6R1_FB16_Pos) /*!< 0x00010000 */
#define CAN_F6R1_FB16 CAN_F6R1_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F6R1_FB17_Pos (17U)
#define CAN_F6R1_FB17_Msk (0x1UL << CAN_F6R1_FB17_Pos) /*!< 0x00020000 */
#define CAN_F6R1_FB17 CAN_F6R1_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F6R1_FB18_Pos (18U)
#define CAN_F6R1_FB18_Msk (0x1UL << CAN_F6R1_FB18_Pos) /*!< 0x00040000 */
#define CAN_F6R1_FB18 CAN_F6R1_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F6R1_FB19_Pos (19U)
#define CAN_F6R1_FB19_Msk (0x1UL << CAN_F6R1_FB19_Pos) /*!< 0x00080000 */
#define CAN_F6R1_FB19 CAN_F6R1_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F6R1_FB20_Pos (20U)
#define CAN_F6R1_FB20_Msk (0x1UL << CAN_F6R1_FB20_Pos) /*!< 0x00100000 */
#define CAN_F6R1_FB20 CAN_F6R1_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F6R1_FB21_Pos (21U)
#define CAN_F6R1_FB21_Msk (0x1UL << CAN_F6R1_FB21_Pos) /*!< 0x00200000 */
#define CAN_F6R1_FB21 CAN_F6R1_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F6R1_FB22_Pos (22U)
#define CAN_F6R1_FB22_Msk (0x1UL << CAN_F6R1_FB22_Pos) /*!< 0x00400000 */
#define CAN_F6R1_FB22 CAN_F6R1_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F6R1_FB23_Pos (23U)
#define CAN_F6R1_FB23_Msk (0x1UL << CAN_F6R1_FB23_Pos) /*!< 0x00800000 */
#define CAN_F6R1_FB23 CAN_F6R1_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F6R1_FB24_Pos (24U)
#define CAN_F6R1_FB24_Msk (0x1UL << CAN_F6R1_FB24_Pos) /*!< 0x01000000 */
#define CAN_F6R1_FB24 CAN_F6R1_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F6R1_FB25_Pos (25U)
#define CAN_F6R1_FB25_Msk (0x1UL << CAN_F6R1_FB25_Pos) /*!< 0x02000000 */
#define CAN_F6R1_FB25 CAN_F6R1_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F6R1_FB26_Pos (26U)
#define CAN_F6R1_FB26_Msk (0x1UL << CAN_F6R1_FB26_Pos) /*!< 0x04000000 */
#define CAN_F6R1_FB26 CAN_F6R1_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F6R1_FB27_Pos (27U)
#define CAN_F6R1_FB27_Msk (0x1UL << CAN_F6R1_FB27_Pos) /*!< 0x08000000 */
#define CAN_F6R1_FB27 CAN_F6R1_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F6R1_FB28_Pos (28U)
#define CAN_F6R1_FB28_Msk (0x1UL << CAN_F6R1_FB28_Pos) /*!< 0x10000000 */
#define CAN_F6R1_FB28 CAN_F6R1_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F6R1_FB29_Pos (29U)
#define CAN_F6R1_FB29_Msk (0x1UL << CAN_F6R1_FB29_Pos) /*!< 0x20000000 */
#define CAN_F6R1_FB29 CAN_F6R1_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F6R1_FB30_Pos (30U)
#define CAN_F6R1_FB30_Msk (0x1UL << CAN_F6R1_FB30_Pos) /*!< 0x40000000 */
#define CAN_F6R1_FB30 CAN_F6R1_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F6R1_FB31_Pos (31U)
#define CAN_F6R1_FB31_Msk (0x1UL << CAN_F6R1_FB31_Pos) /*!< 0x80000000 */
#define CAN_F6R1_FB31 CAN_F6R1_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F7R1 register  *******************/
#define CAN_F7R1_FB0_Pos (0U)
#define CAN_F7R1_FB0_Msk (0x1UL << CAN_F7R1_FB0_Pos) /*!< 0x00000001 */
#define CAN_F7R1_FB0 CAN_F7R1_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F7R1_FB1_Pos (1U)
#define CAN_F7R1_FB1_Msk (0x1UL << CAN_F7R1_FB1_Pos) /*!< 0x00000002 */
#define CAN_F7R1_FB1 CAN_F7R1_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F7R1_FB2_Pos (2U)
#define CAN_F7R1_FB2_Msk (0x1UL << CAN_F7R1_FB2_Pos) /*!< 0x00000004 */
#define CAN_F7R1_FB2 CAN_F7R1_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F7R1_FB3_Pos (3U)
#define CAN_F7R1_FB3_Msk (0x1UL << CAN_F7R1_FB3_Pos) /*!< 0x00000008 */
#define CAN_F7R1_FB3 CAN_F7R1_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F7R1_FB4_Pos (4U)
#define CAN_F7R1_FB4_Msk (0x1UL << CAN_F7R1_FB4_Pos) /*!< 0x00000010 */
#define CAN_F7R1_FB4 CAN_F7R1_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F7R1_FB5_Pos (5U)
#define CAN_F7R1_FB5_Msk (0x1UL << CAN_F7R1_FB5_Pos) /*!< 0x00000020 */
#define CAN_F7R1_FB5 CAN_F7R1_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F7R1_FB6_Pos (6U)
#define CAN_F7R1_FB6_Msk (0x1UL << CAN_F7R1_FB6_Pos) /*!< 0x00000040 */
#define CAN_F7R1_FB6 CAN_F7R1_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F7R1_FB7_Pos (7U)
#define CAN_F7R1_FB7_Msk (0x1UL << CAN_F7R1_FB7_Pos) /*!< 0x00000080 */
#define CAN_F7R1_FB7 CAN_F7R1_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F7R1_FB8_Pos (8U)
#define CAN_F7R1_FB8_Msk (0x1UL << CAN_F7R1_FB8_Pos) /*!< 0x00000100 */
#define CAN_F7R1_FB8 CAN_F7R1_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F7R1_FB9_Pos (9U)
#define CAN_F7R1_FB9_Msk (0x1UL << CAN_F7R1_FB9_Pos) /*!< 0x00000200 */
#define CAN_F7R1_FB9 CAN_F7R1_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F7R1_FB10_Pos (10U)
#define CAN_F7R1_FB10_Msk (0x1UL << CAN_F7R1_FB10_Pos) /*!< 0x00000400 */
#define CAN_F7R1_FB10 CAN_F7R1_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F7R1_FB11_Pos (11U)
#define CAN_F7R1_FB11_Msk (0x1UL << CAN_F7R1_FB11_Pos) /*!< 0x00000800 */
#define CAN_F7R1_FB11 CAN_F7R1_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F7R1_FB12_Pos (12U)
#define CAN_F7R1_FB12_Msk (0x1UL << CAN_F7R1_FB12_Pos) /*!< 0x00001000 */
#define CAN_F7R1_FB12 CAN_F7R1_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F7R1_FB13_Pos (13U)
#define CAN_F7R1_FB13_Msk (0x1UL << CAN_F7R1_FB13_Pos) /*!< 0x00002000 */
#define CAN_F7R1_FB13 CAN_F7R1_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F7R1_FB14_Pos (14U)
#define CAN_F7R1_FB14_Msk (0x1UL << CAN_F7R1_FB14_Pos) /*!< 0x00004000 */
#define CAN_F7R1_FB14 CAN_F7R1_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F7R1_FB15_Pos (15U)
#define CAN_F7R1_FB15_Msk (0x1UL << CAN_F7R1_FB15_Pos) /*!< 0x00008000 */
#define CAN_F7R1_FB15 CAN_F7R1_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F7R1_FB16_Pos (16U)
#define CAN_F7R1_FB16_Msk (0x1UL << CAN_F7R1_FB16_Pos) /*!< 0x00010000 */
#define CAN_F7R1_FB16 CAN_F7R1_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F7R1_FB17_Pos (17U)
#define CAN_F7R1_FB17_Msk (0x1UL << CAN_F7R1_FB17_Pos) /*!< 0x00020000 */
#define CAN_F7R1_FB17 CAN_F7R1_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F7R1_FB18_Pos (18U)
#define CAN_F7R1_FB18_Msk (0x1UL << CAN_F7R1_FB18_Pos) /*!< 0x00040000 */
#define CAN_F7R1_FB18 CAN_F7R1_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F7R1_FB19_Pos (19U)
#define CAN_F7R1_FB19_Msk (0x1UL << CAN_F7R1_FB19_Pos) /*!< 0x00080000 */
#define CAN_F7R1_FB19 CAN_F7R1_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F7R1_FB20_Pos (20U)
#define CAN_F7R1_FB20_Msk (0x1UL << CAN_F7R1_FB20_Pos) /*!< 0x00100000 */
#define CAN_F7R1_FB20 CAN_F7R1_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F7R1_FB21_Pos (21U)
#define CAN_F7R1_FB21_Msk (0x1UL << CAN_F7R1_FB21_Pos) /*!< 0x00200000 */
#define CAN_F7R1_FB21 CAN_F7R1_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F7R1_FB22_Pos (22U)
#define CAN_F7R1_FB22_Msk (0x1UL << CAN_F7R1_FB22_Pos) /*!< 0x00400000 */
#define CAN_F7R1_FB22 CAN_F7R1_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F7R1_FB23_Pos (23U)
#define CAN_F7R1_FB23_Msk (0x1UL << CAN_F7R1_FB23_Pos) /*!< 0x00800000 */
#define CAN_F7R1_FB23 CAN_F7R1_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F7R1_FB24_Pos (24U)
#define CAN_F7R1_FB24_Msk (0x1UL << CAN_F7R1_FB24_Pos) /*!< 0x01000000 */
#define CAN_F7R1_FB24 CAN_F7R1_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F7R1_FB25_Pos (25U)
#define CAN_F7R1_FB25_Msk (0x1UL << CAN_F7R1_FB25_Pos) /*!< 0x02000000 */
#define CAN_F7R1_FB25 CAN_F7R1_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F7R1_FB26_Pos (26U)
#define CAN_F7R1_FB26_Msk (0x1UL << CAN_F7R1_FB26_Pos) /*!< 0x04000000 */
#define CAN_F7R1_FB26 CAN_F7R1_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F7R1_FB27_Pos (27U)
#define CAN_F7R1_FB27_Msk (0x1UL << CAN_F7R1_FB27_Pos) /*!< 0x08000000 */
#define CAN_F7R1_FB27 CAN_F7R1_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F7R1_FB28_Pos (28U)
#define CAN_F7R1_FB28_Msk (0x1UL << CAN_F7R1_FB28_Pos) /*!< 0x10000000 */
#define CAN_F7R1_FB28 CAN_F7R1_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F7R1_FB29_Pos (29U)
#define CAN_F7R1_FB29_Msk (0x1UL << CAN_F7R1_FB29_Pos) /*!< 0x20000000 */
#define CAN_F7R1_FB29 CAN_F7R1_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F7R1_FB30_Pos (30U)
#define CAN_F7R1_FB30_Msk (0x1UL << CAN_F7R1_FB30_Pos) /*!< 0x40000000 */
#define CAN_F7R1_FB30 CAN_F7R1_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F7R1_FB31_Pos (31U)
#define CAN_F7R1_FB31_Msk (0x1UL << CAN_F7R1_FB31_Pos) /*!< 0x80000000 */
#define CAN_F7R1_FB31 CAN_F7R1_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F8R1 register  *******************/
#define CAN_F8R1_FB0_Pos (0U)
#define CAN_F8R1_FB0_Msk (0x1UL << CAN_F8R1_FB0_Pos) /*!< 0x00000001 */
#define CAN_F8R1_FB0 CAN_F8R1_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F8R1_FB1_Pos (1U)
#define CAN_F8R1_FB1_Msk (0x1UL << CAN_F8R1_FB1_Pos) /*!< 0x00000002 */
#define CAN_F8R1_FB1 CAN_F8R1_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F8R1_FB2_Pos (2U)
#define CAN_F8R1_FB2_Msk (0x1UL << CAN_F8R1_FB2_Pos) /*!< 0x00000004 */
#define CAN_F8R1_FB2 CAN_F8R1_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F8R1_FB3_Pos (3U)
#define CAN_F8R1_FB3_Msk (0x1UL << CAN_F8R1_FB3_Pos) /*!< 0x00000008 */
#define CAN_F8R1_FB3 CAN_F8R1_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F8R1_FB4_Pos (4U)
#define CAN_F8R1_FB4_Msk (0x1UL << CAN_F8R1_FB4_Pos) /*!< 0x00000010 */
#define CAN_F8R1_FB4 CAN_F8R1_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F8R1_FB5_Pos (5U)
#define CAN_F8R1_FB5_Msk (0x1UL << CAN_F8R1_FB5_Pos) /*!< 0x00000020 */
#define CAN_F8R1_FB5 CAN_F8R1_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F8R1_FB6_Pos (6U)
#define CAN_F8R1_FB6_Msk (0x1UL << CAN_F8R1_FB6_Pos) /*!< 0x00000040 */
#define CAN_F8R1_FB6 CAN_F8R1_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F8R1_FB7_Pos (7U)
#define CAN_F8R1_FB7_Msk (0x1UL << CAN_F8R1_FB7_Pos) /*!< 0x00000080 */
#define CAN_F8R1_FB7 CAN_F8R1_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F8R1_FB8_Pos (8U)
#define CAN_F8R1_FB8_Msk (0x1UL << CAN_F8R1_FB8_Pos) /*!< 0x00000100 */
#define CAN_F8R1_FB8 CAN_F8R1_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F8R1_FB9_Pos (9U)
#define CAN_F8R1_FB9_Msk (0x1UL << CAN_F8R1_FB9_Pos) /*!< 0x00000200 */
#define CAN_F8R1_FB9 CAN_F8R1_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F8R1_FB10_Pos (10U)
#define CAN_F8R1_FB10_Msk (0x1UL << CAN_F8R1_FB10_Pos) /*!< 0x00000400 */
#define CAN_F8R1_FB10 CAN_F8R1_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F8R1_FB11_Pos (11U)
#define CAN_F8R1_FB11_Msk (0x1UL << CAN_F8R1_FB11_Pos) /*!< 0x00000800 */
#define CAN_F8R1_FB11 CAN_F8R1_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F8R1_FB12_Pos (12U)
#define CAN_F8R1_FB12_Msk (0x1UL << CAN_F8R1_FB12_Pos) /*!< 0x00001000 */
#define CAN_F8R1_FB12 CAN_F8R1_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F8R1_FB13_Pos (13U)
#define CAN_F8R1_FB13_Msk (0x1UL << CAN_F8R1_FB13_Pos) /*!< 0x00002000 */
#define CAN_F8R1_FB13 CAN_F8R1_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F8R1_FB14_Pos (14U)
#define CAN_F8R1_FB14_Msk (0x1UL << CAN_F8R1_FB14_Pos) /*!< 0x00004000 */
#define CAN_F8R1_FB14 CAN_F8R1_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F8R1_FB15_Pos (15U)
#define CAN_F8R1_FB15_Msk (0x1UL << CAN_F8R1_FB15_Pos) /*!< 0x00008000 */
#define CAN_F8R1_FB15 CAN_F8R1_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F8R1_FB16_Pos (16U)
#define CAN_F8R1_FB16_Msk (0x1UL << CAN_F8R1_FB16_Pos) /*!< 0x00010000 */
#define CAN_F8R1_FB16 CAN_F8R1_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F8R1_FB17_Pos (17U)
#define CAN_F8R1_FB17_Msk (0x1UL << CAN_F8R1_FB17_Pos) /*!< 0x00020000 */
#define CAN_F8R1_FB17 CAN_F8R1_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F8R1_FB18_Pos (18U)
#define CAN_F8R1_FB18_Msk (0x1UL << CAN_F8R1_FB18_Pos) /*!< 0x00040000 */
#define CAN_F8R1_FB18 CAN_F8R1_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F8R1_FB19_Pos (19U)
#define CAN_F8R1_FB19_Msk (0x1UL << CAN_F8R1_FB19_Pos) /*!< 0x00080000 */
#define CAN_F8R1_FB19 CAN_F8R1_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F8R1_FB20_Pos (20U)
#define CAN_F8R1_FB20_Msk (0x1UL << CAN_F8R1_FB20_Pos) /*!< 0x00100000 */
#define CAN_F8R1_FB20 CAN_F8R1_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F8R1_FB21_Pos (21U)
#define CAN_F8R1_FB21_Msk (0x1UL << CAN_F8R1_FB21_Pos) /*!< 0x00200000 */
#define CAN_F8R1_FB21 CAN_F8R1_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F8R1_FB22_Pos (22U)
#define CAN_F8R1_FB22_Msk (0x1UL << CAN_F8R1_FB22_Pos) /*!< 0x00400000 */
#define CAN_F8R1_FB22 CAN_F8R1_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F8R1_FB23_Pos (23U)
#define CAN_F8R1_FB23_Msk (0x1UL << CAN_F8R1_FB23_Pos) /*!< 0x00800000 */
#define CAN_F8R1_FB23 CAN_F8R1_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F8R1_FB24_Pos (24U)
#define CAN_F8R1_FB24_Msk (0x1UL << CAN_F8R1_FB24_Pos) /*!< 0x01000000 */
#define CAN_F8R1_FB24 CAN_F8R1_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F8R1_FB25_Pos (25U)
#define CAN_F8R1_FB25_Msk (0x1UL << CAN_F8R1_FB25_Pos) /*!< 0x02000000 */
#define CAN_F8R1_FB25 CAN_F8R1_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F8R1_FB26_Pos (26U)
#define CAN_F8R1_FB26_Msk (0x1UL << CAN_F8R1_FB26_Pos) /*!< 0x04000000 */
#define CAN_F8R1_FB26 CAN_F8R1_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F8R1_FB27_Pos (27U)
#define CAN_F8R1_FB27_Msk (0x1UL << CAN_F8R1_FB27_Pos) /*!< 0x08000000 */
#define CAN_F8R1_FB27 CAN_F8R1_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F8R1_FB28_Pos (28U)
#define CAN_F8R1_FB28_Msk (0x1UL << CAN_F8R1_FB28_Pos) /*!< 0x10000000 */
#define CAN_F8R1_FB28 CAN_F8R1_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F8R1_FB29_Pos (29U)
#define CAN_F8R1_FB29_Msk (0x1UL << CAN_F8R1_FB29_Pos) /*!< 0x20000000 */
#define CAN_F8R1_FB29 CAN_F8R1_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F8R1_FB30_Pos (30U)
#define CAN_F8R1_FB30_Msk (0x1UL << CAN_F8R1_FB30_Pos) /*!< 0x40000000 */
#define CAN_F8R1_FB30 CAN_F8R1_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F8R1_FB31_Pos (31U)
#define CAN_F8R1_FB31_Msk (0x1UL << CAN_F8R1_FB31_Pos) /*!< 0x80000000 */
#define CAN_F8R1_FB31 CAN_F8R1_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F9R1 register  *******************/
#define CAN_F9R1_FB0_Pos (0U)
#define CAN_F9R1_FB0_Msk (0x1UL << CAN_F9R1_FB0_Pos) /*!< 0x00000001 */
#define CAN_F9R1_FB0 CAN_F9R1_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F9R1_FB1_Pos (1U)
#define CAN_F9R1_FB1_Msk (0x1UL << CAN_F9R1_FB1_Pos) /*!< 0x00000002 */
#define CAN_F9R1_FB1 CAN_F9R1_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F9R1_FB2_Pos (2U)
#define CAN_F9R1_FB2_Msk (0x1UL << CAN_F9R1_FB2_Pos) /*!< 0x00000004 */
#define CAN_F9R1_FB2 CAN_F9R1_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F9R1_FB3_Pos (3U)
#define CAN_F9R1_FB3_Msk (0x1UL << CAN_F9R1_FB3_Pos) /*!< 0x00000008 */
#define CAN_F9R1_FB3 CAN_F9R1_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F9R1_FB4_Pos (4U)
#define CAN_F9R1_FB4_Msk (0x1UL << CAN_F9R1_FB4_Pos) /*!< 0x00000010 */
#define CAN_F9R1_FB4 CAN_F9R1_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F9R1_FB5_Pos (5U)
#define CAN_F9R1_FB5_Msk (0x1UL << CAN_F9R1_FB5_Pos) /*!< 0x00000020 */
#define CAN_F9R1_FB5 CAN_F9R1_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F9R1_FB6_Pos (6U)
#define CAN_F9R1_FB6_Msk (0x1UL << CAN_F9R1_FB6_Pos) /*!< 0x00000040 */
#define CAN_F9R1_FB6 CAN_F9R1_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F9R1_FB7_Pos (7U)
#define CAN_F9R1_FB7_Msk (0x1UL << CAN_F9R1_FB7_Pos) /*!< 0x00000080 */
#define CAN_F9R1_FB7 CAN_F9R1_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F9R1_FB8_Pos (8U)
#define CAN_F9R1_FB8_Msk (0x1UL << CAN_F9R1_FB8_Pos) /*!< 0x00000100 */
#define CAN_F9R1_FB8 CAN_F9R1_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F9R1_FB9_Pos (9U)
#define CAN_F9R1_FB9_Msk (0x1UL << CAN_F9R1_FB9_Pos) /*!< 0x00000200 */
#define CAN_F9R1_FB9 CAN_F9R1_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F9R1_FB10_Pos (10U)
#define CAN_F9R1_FB10_Msk (0x1UL << CAN_F9R1_FB10_Pos) /*!< 0x00000400 */
#define CAN_F9R1_FB10 CAN_F9R1_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F9R1_FB11_Pos (11U)
#define CAN_F9R1_FB11_Msk (0x1UL << CAN_F9R1_FB11_Pos) /*!< 0x00000800 */
#define CAN_F9R1_FB11 CAN_F9R1_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F9R1_FB12_Pos (12U)
#define CAN_F9R1_FB12_Msk (0x1UL << CAN_F9R1_FB12_Pos) /*!< 0x00001000 */
#define CAN_F9R1_FB12 CAN_F9R1_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F9R1_FB13_Pos (13U)
#define CAN_F9R1_FB13_Msk (0x1UL << CAN_F9R1_FB13_Pos) /*!< 0x00002000 */
#define CAN_F9R1_FB13 CAN_F9R1_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F9R1_FB14_Pos (14U)
#define CAN_F9R1_FB14_Msk (0x1UL << CAN_F9R1_FB14_Pos) /*!< 0x00004000 */
#define CAN_F9R1_FB14 CAN_F9R1_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F9R1_FB15_Pos (15U)
#define CAN_F9R1_FB15_Msk (0x1UL << CAN_F9R1_FB15_Pos) /*!< 0x00008000 */
#define CAN_F9R1_FB15 CAN_F9R1_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F9R1_FB16_Pos (16U)
#define CAN_F9R1_FB16_Msk (0x1UL << CAN_F9R1_FB16_Pos) /*!< 0x00010000 */
#define CAN_F9R1_FB16 CAN_F9R1_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F9R1_FB17_Pos (17U)
#define CAN_F9R1_FB17_Msk (0x1UL << CAN_F9R1_FB17_Pos) /*!< 0x00020000 */
#define CAN_F9R1_FB17 CAN_F9R1_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F9R1_FB18_Pos (18U)
#define CAN_F9R1_FB18_Msk (0x1UL << CAN_F9R1_FB18_Pos) /*!< 0x00040000 */
#define CAN_F9R1_FB18 CAN_F9R1_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F9R1_FB19_Pos (19U)
#define CAN_F9R1_FB19_Msk (0x1UL << CAN_F9R1_FB19_Pos) /*!< 0x00080000 */
#define CAN_F9R1_FB19 CAN_F9R1_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F9R1_FB20_Pos (20U)
#define CAN_F9R1_FB20_Msk (0x1UL << CAN_F9R1_FB20_Pos) /*!< 0x00100000 */
#define CAN_F9R1_FB20 CAN_F9R1_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F9R1_FB21_Pos (21U)
#define CAN_F9R1_FB21_Msk (0x1UL << CAN_F9R1_FB21_Pos) /*!< 0x00200000 */
#define CAN_F9R1_FB21 CAN_F9R1_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F9R1_FB22_Pos (22U)
#define CAN_F9R1_FB22_Msk (0x1UL << CAN_F9R1_FB22_Pos) /*!< 0x00400000 */
#define CAN_F9R1_FB22 CAN_F9R1_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F9R1_FB23_Pos (23U)
#define CAN_F9R1_FB23_Msk (0x1UL << CAN_F9R1_FB23_Pos) /*!< 0x00800000 */
#define CAN_F9R1_FB23 CAN_F9R1_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F9R1_FB24_Pos (24U)
#define CAN_F9R1_FB24_Msk (0x1UL << CAN_F9R1_FB24_Pos) /*!< 0x01000000 */
#define CAN_F9R1_FB24 CAN_F9R1_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F9R1_FB25_Pos (25U)
#define CAN_F9R1_FB25_Msk (0x1UL << CAN_F9R1_FB25_Pos) /*!< 0x02000000 */
#define CAN_F9R1_FB25 CAN_F9R1_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F9R1_FB26_Pos (26U)
#define CAN_F9R1_FB26_Msk (0x1UL << CAN_F9R1_FB26_Pos) /*!< 0x04000000 */
#define CAN_F9R1_FB26 CAN_F9R1_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F9R1_FB27_Pos (27U)
#define CAN_F9R1_FB27_Msk (0x1UL << CAN_F9R1_FB27_Pos) /*!< 0x08000000 */
#define CAN_F9R1_FB27 CAN_F9R1_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F9R1_FB28_Pos (28U)
#define CAN_F9R1_FB28_Msk (0x1UL << CAN_F9R1_FB28_Pos) /*!< 0x10000000 */
#define CAN_F9R1_FB28 CAN_F9R1_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F9R1_FB29_Pos (29U)
#define CAN_F9R1_FB29_Msk (0x1UL << CAN_F9R1_FB29_Pos) /*!< 0x20000000 */
#define CAN_F9R1_FB29 CAN_F9R1_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F9R1_FB30_Pos (30U)
#define CAN_F9R1_FB30_Msk (0x1UL << CAN_F9R1_FB30_Pos) /*!< 0x40000000 */
#define CAN_F9R1_FB30 CAN_F9R1_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F9R1_FB31_Pos (31U)
#define CAN_F9R1_FB31_Msk (0x1UL << CAN_F9R1_FB31_Pos) /*!< 0x80000000 */
#define CAN_F9R1_FB31 CAN_F9R1_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F10R1 register  ******************/
#define CAN_F10R1_FB0_Pos (0U)
#define CAN_F10R1_FB0_Msk (0x1UL << CAN_F10R1_FB0_Pos) /*!< 0x00000001 */
#define CAN_F10R1_FB0 CAN_F10R1_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F10R1_FB1_Pos (1U)
#define CAN_F10R1_FB1_Msk (0x1UL << CAN_F10R1_FB1_Pos) /*!< 0x00000002 */
#define CAN_F10R1_FB1 CAN_F10R1_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F10R1_FB2_Pos (2U)
#define CAN_F10R1_FB2_Msk (0x1UL << CAN_F10R1_FB2_Pos) /*!< 0x00000004 */
#define CAN_F10R1_FB2 CAN_F10R1_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F10R1_FB3_Pos (3U)
#define CAN_F10R1_FB3_Msk (0x1UL << CAN_F10R1_FB3_Pos) /*!< 0x00000008 */
#define CAN_F10R1_FB3 CAN_F10R1_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F10R1_FB4_Pos (4U)
#define CAN_F10R1_FB4_Msk (0x1UL << CAN_F10R1_FB4_Pos) /*!< 0x00000010 */
#define CAN_F10R1_FB4 CAN_F10R1_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F10R1_FB5_Pos (5U)
#define CAN_F10R1_FB5_Msk (0x1UL << CAN_F10R1_FB5_Pos) /*!< 0x00000020 */
#define CAN_F10R1_FB5 CAN_F10R1_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F10R1_FB6_Pos (6U)
#define CAN_F10R1_FB6_Msk (0x1UL << CAN_F10R1_FB6_Pos) /*!< 0x00000040 */
#define CAN_F10R1_FB6 CAN_F10R1_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F10R1_FB7_Pos (7U)
#define CAN_F10R1_FB7_Msk (0x1UL << CAN_F10R1_FB7_Pos) /*!< 0x00000080 */
#define CAN_F10R1_FB7 CAN_F10R1_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F10R1_FB8_Pos (8U)
#define CAN_F10R1_FB8_Msk (0x1UL << CAN_F10R1_FB8_Pos) /*!< 0x00000100 */
#define CAN_F10R1_FB8 CAN_F10R1_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F10R1_FB9_Pos (9U)
#define CAN_F10R1_FB9_Msk (0x1UL << CAN_F10R1_FB9_Pos) /*!< 0x00000200 */
#define CAN_F10R1_FB9 CAN_F10R1_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F10R1_FB10_Pos (10U)
#define CAN_F10R1_FB10_Msk (0x1UL << CAN_F10R1_FB10_Pos) /*!< 0x00000400 */
#define CAN_F10R1_FB10 CAN_F10R1_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F10R1_FB11_Pos (11U)
#define CAN_F10R1_FB11_Msk (0x1UL << CAN_F10R1_FB11_Pos) /*!< 0x00000800 */
#define CAN_F10R1_FB11 CAN_F10R1_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F10R1_FB12_Pos (12U)
#define CAN_F10R1_FB12_Msk (0x1UL << CAN_F10R1_FB12_Pos) /*!< 0x00001000 */
#define CAN_F10R1_FB12 CAN_F10R1_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F10R1_FB13_Pos (13U)
#define CAN_F10R1_FB13_Msk (0x1UL << CAN_F10R1_FB13_Pos) /*!< 0x00002000 */
#define CAN_F10R1_FB13 CAN_F10R1_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F10R1_FB14_Pos (14U)
#define CAN_F10R1_FB14_Msk (0x1UL << CAN_F10R1_FB14_Pos) /*!< 0x00004000 */
#define CAN_F10R1_FB14 CAN_F10R1_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F10R1_FB15_Pos (15U)
#define CAN_F10R1_FB15_Msk (0x1UL << CAN_F10R1_FB15_Pos) /*!< 0x00008000 */
#define CAN_F10R1_FB15 CAN_F10R1_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F10R1_FB16_Pos (16U)
#define CAN_F10R1_FB16_Msk (0x1UL << CAN_F10R1_FB16_Pos) /*!< 0x00010000 */
#define CAN_F10R1_FB16 CAN_F10R1_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F10R1_FB17_Pos (17U)
#define CAN_F10R1_FB17_Msk (0x1UL << CAN_F10R1_FB17_Pos) /*!< 0x00020000 */
#define CAN_F10R1_FB17 CAN_F10R1_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F10R1_FB18_Pos (18U)
#define CAN_F10R1_FB18_Msk (0x1UL << CAN_F10R1_FB18_Pos) /*!< 0x00040000 */
#define CAN_F10R1_FB18 CAN_F10R1_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F10R1_FB19_Pos (19U)
#define CAN_F10R1_FB19_Msk (0x1UL << CAN_F10R1_FB19_Pos) /*!< 0x00080000 */
#define CAN_F10R1_FB19 CAN_F10R1_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F10R1_FB20_Pos (20U)
#define CAN_F10R1_FB20_Msk (0x1UL << CAN_F10R1_FB20_Pos) /*!< 0x00100000 */
#define CAN_F10R1_FB20 CAN_F10R1_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F10R1_FB21_Pos (21U)
#define CAN_F10R1_FB21_Msk (0x1UL << CAN_F10R1_FB21_Pos) /*!< 0x00200000 */
#define CAN_F10R1_FB21 CAN_F10R1_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F10R1_FB22_Pos (22U)
#define CAN_F10R1_FB22_Msk (0x1UL << CAN_F10R1_FB22_Pos) /*!< 0x00400000 */
#define CAN_F10R1_FB22 CAN_F10R1_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F10R1_FB23_Pos (23U)
#define CAN_F10R1_FB23_Msk (0x1UL << CAN_F10R1_FB23_Pos) /*!< 0x00800000 */
#define CAN_F10R1_FB23 CAN_F10R1_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F10R1_FB24_Pos (24U)
#define CAN_F10R1_FB24_Msk (0x1UL << CAN_F10R1_FB24_Pos) /*!< 0x01000000 */
#define CAN_F10R1_FB24 CAN_F10R1_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F10R1_FB25_Pos (25U)
#define CAN_F10R1_FB25_Msk (0x1UL << CAN_F10R1_FB25_Pos) /*!< 0x02000000 */
#define CAN_F10R1_FB25 CAN_F10R1_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F10R1_FB26_Pos (26U)
#define CAN_F10R1_FB26_Msk (0x1UL << CAN_F10R1_FB26_Pos) /*!< 0x04000000 */
#define CAN_F10R1_FB26 CAN_F10R1_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F10R1_FB27_Pos (27U)
#define CAN_F10R1_FB27_Msk (0x1UL << CAN_F10R1_FB27_Pos) /*!< 0x08000000 */
#define CAN_F10R1_FB27 CAN_F10R1_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F10R1_FB28_Pos (28U)
#define CAN_F10R1_FB28_Msk (0x1UL << CAN_F10R1_FB28_Pos) /*!< 0x10000000 */
#define CAN_F10R1_FB28 CAN_F10R1_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F10R1_FB29_Pos (29U)
#define CAN_F10R1_FB29_Msk (0x1UL << CAN_F10R1_FB29_Pos) /*!< 0x20000000 */
#define CAN_F10R1_FB29 CAN_F10R1_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F10R1_FB30_Pos (30U)
#define CAN_F10R1_FB30_Msk (0x1UL << CAN_F10R1_FB30_Pos) /*!< 0x40000000 */
#define CAN_F10R1_FB30 CAN_F10R1_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F10R1_FB31_Pos (31U)
#define CAN_F10R1_FB31_Msk (0x1UL << CAN_F10R1_FB31_Pos) /*!< 0x80000000 */
#define CAN_F10R1_FB31 CAN_F10R1_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F11R1 register  ******************/
#define CAN_F11R1_FB0_Pos (0U)
#define CAN_F11R1_FB0_Msk (0x1UL << CAN_F11R1_FB0_Pos) /*!< 0x00000001 */
#define CAN_F11R1_FB0 CAN_F11R1_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F11R1_FB1_Pos (1U)
#define CAN_F11R1_FB1_Msk (0x1UL << CAN_F11R1_FB1_Pos) /*!< 0x00000002 */
#define CAN_F11R1_FB1 CAN_F11R1_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F11R1_FB2_Pos (2U)
#define CAN_F11R1_FB2_Msk (0x1UL << CAN_F11R1_FB2_Pos) /*!< 0x00000004 */
#define CAN_F11R1_FB2 CAN_F11R1_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F11R1_FB3_Pos (3U)
#define CAN_F11R1_FB3_Msk (0x1UL << CAN_F11R1_FB3_Pos) /*!< 0x00000008 */
#define CAN_F11R1_FB3 CAN_F11R1_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F11R1_FB4_Pos (4U)
#define CAN_F11R1_FB4_Msk (0x1UL << CAN_F11R1_FB4_Pos) /*!< 0x00000010 */
#define CAN_F11R1_FB4 CAN_F11R1_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F11R1_FB5_Pos (5U)
#define CAN_F11R1_FB5_Msk (0x1UL << CAN_F11R1_FB5_Pos) /*!< 0x00000020 */
#define CAN_F11R1_FB5 CAN_F11R1_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F11R1_FB6_Pos (6U)
#define CAN_F11R1_FB6_Msk (0x1UL << CAN_F11R1_FB6_Pos) /*!< 0x00000040 */
#define CAN_F11R1_FB6 CAN_F11R1_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F11R1_FB7_Pos (7U)
#define CAN_F11R1_FB7_Msk (0x1UL << CAN_F11R1_FB7_Pos) /*!< 0x00000080 */
#define CAN_F11R1_FB7 CAN_F11R1_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F11R1_FB8_Pos (8U)
#define CAN_F11R1_FB8_Msk (0x1UL << CAN_F11R1_FB8_Pos) /*!< 0x00000100 */
#define CAN_F11R1_FB8 CAN_F11R1_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F11R1_FB9_Pos (9U)
#define CAN_F11R1_FB9_Msk (0x1UL << CAN_F11R1_FB9_Pos) /*!< 0x00000200 */
#define CAN_F11R1_FB9 CAN_F11R1_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F11R1_FB10_Pos (10U)
#define CAN_F11R1_FB10_Msk (0x1UL << CAN_F11R1_FB10_Pos) /*!< 0x00000400 */
#define CAN_F11R1_FB10 CAN_F11R1_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F11R1_FB11_Pos (11U)
#define CAN_F11R1_FB11_Msk (0x1UL << CAN_F11R1_FB11_Pos) /*!< 0x00000800 */
#define CAN_F11R1_FB11 CAN_F11R1_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F11R1_FB12_Pos (12U)
#define CAN_F11R1_FB12_Msk (0x1UL << CAN_F11R1_FB12_Pos) /*!< 0x00001000 */
#define CAN_F11R1_FB12 CAN_F11R1_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F11R1_FB13_Pos (13U)
#define CAN_F11R1_FB13_Msk (0x1UL << CAN_F11R1_FB13_Pos) /*!< 0x00002000 */
#define CAN_F11R1_FB13 CAN_F11R1_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F11R1_FB14_Pos (14U)
#define CAN_F11R1_FB14_Msk (0x1UL << CAN_F11R1_FB14_Pos) /*!< 0x00004000 */
#define CAN_F11R1_FB14 CAN_F11R1_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F11R1_FB15_Pos (15U)
#define CAN_F11R1_FB15_Msk (0x1UL << CAN_F11R1_FB15_Pos) /*!< 0x00008000 */
#define CAN_F11R1_FB15 CAN_F11R1_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F11R1_FB16_Pos (16U)
#define CAN_F11R1_FB16_Msk (0x1UL << CAN_F11R1_FB16_Pos) /*!< 0x00010000 */
#define CAN_F11R1_FB16 CAN_F11R1_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F11R1_FB17_Pos (17U)
#define CAN_F11R1_FB17_Msk (0x1UL << CAN_F11R1_FB17_Pos) /*!< 0x00020000 */
#define CAN_F11R1_FB17 CAN_F11R1_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F11R1_FB18_Pos (18U)
#define CAN_F11R1_FB18_Msk (0x1UL << CAN_F11R1_FB18_Pos) /*!< 0x00040000 */
#define CAN_F11R1_FB18 CAN_F11R1_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F11R1_FB19_Pos (19U)
#define CAN_F11R1_FB19_Msk (0x1UL << CAN_F11R1_FB19_Pos) /*!< 0x00080000 */
#define CAN_F11R1_FB19 CAN_F11R1_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F11R1_FB20_Pos (20U)
#define CAN_F11R1_FB20_Msk (0x1UL << CAN_F11R1_FB20_Pos) /*!< 0x00100000 */
#define CAN_F11R1_FB20 CAN_F11R1_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F11R1_FB21_Pos (21U)
#define CAN_F11R1_FB21_Msk (0x1UL << CAN_F11R1_FB21_Pos) /*!< 0x00200000 */
#define CAN_F11R1_FB21 CAN_F11R1_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F11R1_FB22_Pos (22U)
#define CAN_F11R1_FB22_Msk (0x1UL << CAN_F11R1_FB22_Pos) /*!< 0x00400000 */
#define CAN_F11R1_FB22 CAN_F11R1_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F11R1_FB23_Pos (23U)
#define CAN_F11R1_FB23_Msk (0x1UL << CAN_F11R1_FB23_Pos) /*!< 0x00800000 */
#define CAN_F11R1_FB23 CAN_F11R1_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F11R1_FB24_Pos (24U)
#define CAN_F11R1_FB24_Msk (0x1UL << CAN_F11R1_FB24_Pos) /*!< 0x01000000 */
#define CAN_F11R1_FB24 CAN_F11R1_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F11R1_FB25_Pos (25U)
#define CAN_F11R1_FB25_Msk (0x1UL << CAN_F11R1_FB25_Pos) /*!< 0x02000000 */
#define CAN_F11R1_FB25 CAN_F11R1_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F11R1_FB26_Pos (26U)
#define CAN_F11R1_FB26_Msk (0x1UL << CAN_F11R1_FB26_Pos) /*!< 0x04000000 */
#define CAN_F11R1_FB26 CAN_F11R1_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F11R1_FB27_Pos (27U)
#define CAN_F11R1_FB27_Msk (0x1UL << CAN_F11R1_FB27_Pos) /*!< 0x08000000 */
#define CAN_F11R1_FB27 CAN_F11R1_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F11R1_FB28_Pos (28U)
#define CAN_F11R1_FB28_Msk (0x1UL << CAN_F11R1_FB28_Pos) /*!< 0x10000000 */
#define CAN_F11R1_FB28 CAN_F11R1_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F11R1_FB29_Pos (29U)
#define CAN_F11R1_FB29_Msk (0x1UL << CAN_F11R1_FB29_Pos) /*!< 0x20000000 */
#define CAN_F11R1_FB29 CAN_F11R1_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F11R1_FB30_Pos (30U)
#define CAN_F11R1_FB30_Msk (0x1UL << CAN_F11R1_FB30_Pos) /*!< 0x40000000 */
#define CAN_F11R1_FB30 CAN_F11R1_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F11R1_FB31_Pos (31U)
#define CAN_F11R1_FB31_Msk (0x1UL << CAN_F11R1_FB31_Pos) /*!< 0x80000000 */
#define CAN_F11R1_FB31 CAN_F11R1_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F12R1 register  ******************/
#define CAN_F12R1_FB0_Pos (0U)
#define CAN_F12R1_FB0_Msk (0x1UL << CAN_F12R1_FB0_Pos) /*!< 0x00000001 */
#define CAN_F12R1_FB0 CAN_F12R1_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F12R1_FB1_Pos (1U)
#define CAN_F12R1_FB1_Msk (0x1UL << CAN_F12R1_FB1_Pos) /*!< 0x00000002 */
#define CAN_F12R1_FB1 CAN_F12R1_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F12R1_FB2_Pos (2U)
#define CAN_F12R1_FB2_Msk (0x1UL << CAN_F12R1_FB2_Pos) /*!< 0x00000004 */
#define CAN_F12R1_FB2 CAN_F12R1_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F12R1_FB3_Pos (3U)
#define CAN_F12R1_FB3_Msk (0x1UL << CAN_F12R1_FB3_Pos) /*!< 0x00000008 */
#define CAN_F12R1_FB3 CAN_F12R1_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F12R1_FB4_Pos (4U)
#define CAN_F12R1_FB4_Msk (0x1UL << CAN_F12R1_FB4_Pos) /*!< 0x00000010 */
#define CAN_F12R1_FB4 CAN_F12R1_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F12R1_FB5_Pos (5U)
#define CAN_F12R1_FB5_Msk (0x1UL << CAN_F12R1_FB5_Pos) /*!< 0x00000020 */
#define CAN_F12R1_FB5 CAN_F12R1_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F12R1_FB6_Pos (6U)
#define CAN_F12R1_FB6_Msk (0x1UL << CAN_F12R1_FB6_Pos) /*!< 0x00000040 */
#define CAN_F12R1_FB6 CAN_F12R1_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F12R1_FB7_Pos (7U)
#define CAN_F12R1_FB7_Msk (0x1UL << CAN_F12R1_FB7_Pos) /*!< 0x00000080 */
#define CAN_F12R1_FB7 CAN_F12R1_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F12R1_FB8_Pos (8U)
#define CAN_F12R1_FB8_Msk (0x1UL << CAN_F12R1_FB8_Pos) /*!< 0x00000100 */
#define CAN_F12R1_FB8 CAN_F12R1_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F12R1_FB9_Pos (9U)
#define CAN_F12R1_FB9_Msk (0x1UL << CAN_F12R1_FB9_Pos) /*!< 0x00000200 */
#define CAN_F12R1_FB9 CAN_F12R1_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F12R1_FB10_Pos (10U)
#define CAN_F12R1_FB10_Msk (0x1UL << CAN_F12R1_FB10_Pos) /*!< 0x00000400 */
#define CAN_F12R1_FB10 CAN_F12R1_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F12R1_FB11_Pos (11U)
#define CAN_F12R1_FB11_Msk (0x1UL << CAN_F12R1_FB11_Pos) /*!< 0x00000800 */
#define CAN_F12R1_FB11 CAN_F12R1_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F12R1_FB12_Pos (12U)
#define CAN_F12R1_FB12_Msk (0x1UL << CAN_F12R1_FB12_Pos) /*!< 0x00001000 */
#define CAN_F12R1_FB12 CAN_F12R1_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F12R1_FB13_Pos (13U)
#define CAN_F12R1_FB13_Msk (0x1UL << CAN_F12R1_FB13_Pos) /*!< 0x00002000 */
#define CAN_F12R1_FB13 CAN_F12R1_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F12R1_FB14_Pos (14U)
#define CAN_F12R1_FB14_Msk (0x1UL << CAN_F12R1_FB14_Pos) /*!< 0x00004000 */
#define CAN_F12R1_FB14 CAN_F12R1_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F12R1_FB15_Pos (15U)
#define CAN_F12R1_FB15_Msk (0x1UL << CAN_F12R1_FB15_Pos) /*!< 0x00008000 */
#define CAN_F12R1_FB15 CAN_F12R1_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F12R1_FB16_Pos (16U)
#define CAN_F12R1_FB16_Msk (0x1UL << CAN_F12R1_FB16_Pos) /*!< 0x00010000 */
#define CAN_F12R1_FB16 CAN_F12R1_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F12R1_FB17_Pos (17U)
#define CAN_F12R1_FB17_Msk (0x1UL << CAN_F12R1_FB17_Pos) /*!< 0x00020000 */
#define CAN_F12R1_FB17 CAN_F12R1_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F12R1_FB18_Pos (18U)
#define CAN_F12R1_FB18_Msk (0x1UL << CAN_F12R1_FB18_Pos) /*!< 0x00040000 */
#define CAN_F12R1_FB18 CAN_F12R1_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F12R1_FB19_Pos (19U)
#define CAN_F12R1_FB19_Msk (0x1UL << CAN_F12R1_FB19_Pos) /*!< 0x00080000 */
#define CAN_F12R1_FB19 CAN_F12R1_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F12R1_FB20_Pos (20U)
#define CAN_F12R1_FB20_Msk (0x1UL << CAN_F12R1_FB20_Pos) /*!< 0x00100000 */
#define CAN_F12R1_FB20 CAN_F12R1_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F12R1_FB21_Pos (21U)
#define CAN_F12R1_FB21_Msk (0x1UL << CAN_F12R1_FB21_Pos) /*!< 0x00200000 */
#define CAN_F12R1_FB21 CAN_F12R1_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F12R1_FB22_Pos (22U)
#define CAN_F12R1_FB22_Msk (0x1UL << CAN_F12R1_FB22_Pos) /*!< 0x00400000 */
#define CAN_F12R1_FB22 CAN_F12R1_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F12R1_FB23_Pos (23U)
#define CAN_F12R1_FB23_Msk (0x1UL << CAN_F12R1_FB23_Pos) /*!< 0x00800000 */
#define CAN_F12R1_FB23 CAN_F12R1_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F12R1_FB24_Pos (24U)
#define CAN_F12R1_FB24_Msk (0x1UL << CAN_F12R1_FB24_Pos) /*!< 0x01000000 */
#define CAN_F12R1_FB24 CAN_F12R1_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F12R1_FB25_Pos (25U)
#define CAN_F12R1_FB25_Msk (0x1UL << CAN_F12R1_FB25_Pos) /*!< 0x02000000 */
#define CAN_F12R1_FB25 CAN_F12R1_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F12R1_FB26_Pos (26U)
#define CAN_F12R1_FB26_Msk (0x1UL << CAN_F12R1_FB26_Pos) /*!< 0x04000000 */
#define CAN_F12R1_FB26 CAN_F12R1_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F12R1_FB27_Pos (27U)
#define CAN_F12R1_FB27_Msk (0x1UL << CAN_F12R1_FB27_Pos) /*!< 0x08000000 */
#define CAN_F12R1_FB27 CAN_F12R1_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F12R1_FB28_Pos (28U)
#define CAN_F12R1_FB28_Msk (0x1UL << CAN_F12R1_FB28_Pos) /*!< 0x10000000 */
#define CAN_F12R1_FB28 CAN_F12R1_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F12R1_FB29_Pos (29U)
#define CAN_F12R1_FB29_Msk (0x1UL << CAN_F12R1_FB29_Pos) /*!< 0x20000000 */
#define CAN_F12R1_FB29 CAN_F12R1_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F12R1_FB30_Pos (30U)
#define CAN_F12R1_FB30_Msk (0x1UL << CAN_F12R1_FB30_Pos) /*!< 0x40000000 */
#define CAN_F12R1_FB30 CAN_F12R1_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F12R1_FB31_Pos (31U)
#define CAN_F12R1_FB31_Msk (0x1UL << CAN_F12R1_FB31_Pos) /*!< 0x80000000 */
#define CAN_F12R1_FB31 CAN_F12R1_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F13R1 register  ******************/
#define CAN_F13R1_FB0_Pos (0U)
#define CAN_F13R1_FB0_Msk (0x1UL << CAN_F13R1_FB0_Pos) /*!< 0x00000001 */
#define CAN_F13R1_FB0 CAN_F13R1_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F13R1_FB1_Pos (1U)
#define CAN_F13R1_FB1_Msk (0x1UL << CAN_F13R1_FB1_Pos) /*!< 0x00000002 */
#define CAN_F13R1_FB1 CAN_F13R1_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F13R1_FB2_Pos (2U)
#define CAN_F13R1_FB2_Msk (0x1UL << CAN_F13R1_FB2_Pos) /*!< 0x00000004 */
#define CAN_F13R1_FB2 CAN_F13R1_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F13R1_FB3_Pos (3U)
#define CAN_F13R1_FB3_Msk (0x1UL << CAN_F13R1_FB3_Pos) /*!< 0x00000008 */
#define CAN_F13R1_FB3 CAN_F13R1_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F13R1_FB4_Pos (4U)
#define CAN_F13R1_FB4_Msk (0x1UL << CAN_F13R1_FB4_Pos) /*!< 0x00000010 */
#define CAN_F13R1_FB4 CAN_F13R1_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F13R1_FB5_Pos (5U)
#define CAN_F13R1_FB5_Msk (0x1UL << CAN_F13R1_FB5_Pos) /*!< 0x00000020 */
#define CAN_F13R1_FB5 CAN_F13R1_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F13R1_FB6_Pos (6U)
#define CAN_F13R1_FB6_Msk (0x1UL << CAN_F13R1_FB6_Pos) /*!< 0x00000040 */
#define CAN_F13R1_FB6 CAN_F13R1_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F13R1_FB7_Pos (7U)
#define CAN_F13R1_FB7_Msk (0x1UL << CAN_F13R1_FB7_Pos) /*!< 0x00000080 */
#define CAN_F13R1_FB7 CAN_F13R1_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F13R1_FB8_Pos (8U)
#define CAN_F13R1_FB8_Msk (0x1UL << CAN_F13R1_FB8_Pos) /*!< 0x00000100 */
#define CAN_F13R1_FB8 CAN_F13R1_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F13R1_FB9_Pos (9U)
#define CAN_F13R1_FB9_Msk (0x1UL << CAN_F13R1_FB9_Pos) /*!< 0x00000200 */
#define CAN_F13R1_FB9 CAN_F13R1_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F13R1_FB10_Pos (10U)
#define CAN_F13R1_FB10_Msk (0x1UL << CAN_F13R1_FB10_Pos) /*!< 0x00000400 */
#define CAN_F13R1_FB10 CAN_F13R1_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F13R1_FB11_Pos (11U)
#define CAN_F13R1_FB11_Msk (0x1UL << CAN_F13R1_FB11_Pos) /*!< 0x00000800 */
#define CAN_F13R1_FB11 CAN_F13R1_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F13R1_FB12_Pos (12U)
#define CAN_F13R1_FB12_Msk (0x1UL << CAN_F13R1_FB12_Pos) /*!< 0x00001000 */
#define CAN_F13R1_FB12 CAN_F13R1_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F13R1_FB13_Pos (13U)
#define CAN_F13R1_FB13_Msk (0x1UL << CAN_F13R1_FB13_Pos) /*!< 0x00002000 */
#define CAN_F13R1_FB13 CAN_F13R1_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F13R1_FB14_Pos (14U)
#define CAN_F13R1_FB14_Msk (0x1UL << CAN_F13R1_FB14_Pos) /*!< 0x00004000 */
#define CAN_F13R1_FB14 CAN_F13R1_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F13R1_FB15_Pos (15U)
#define CAN_F13R1_FB15_Msk (0x1UL << CAN_F13R1_FB15_Pos) /*!< 0x00008000 */
#define CAN_F13R1_FB15 CAN_F13R1_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F13R1_FB16_Pos (16U)
#define CAN_F13R1_FB16_Msk (0x1UL << CAN_F13R1_FB16_Pos) /*!< 0x00010000 */
#define CAN_F13R1_FB16 CAN_F13R1_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F13R1_FB17_Pos (17U)
#define CAN_F13R1_FB17_Msk (0x1UL << CAN_F13R1_FB17_Pos) /*!< 0x00020000 */
#define CAN_F13R1_FB17 CAN_F13R1_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F13R1_FB18_Pos (18U)
#define CAN_F13R1_FB18_Msk (0x1UL << CAN_F13R1_FB18_Pos) /*!< 0x00040000 */
#define CAN_F13R1_FB18 CAN_F13R1_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F13R1_FB19_Pos (19U)
#define CAN_F13R1_FB19_Msk (0x1UL << CAN_F13R1_FB19_Pos) /*!< 0x00080000 */
#define CAN_F13R1_FB19 CAN_F13R1_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F13R1_FB20_Pos (20U)
#define CAN_F13R1_FB20_Msk (0x1UL << CAN_F13R1_FB20_Pos) /*!< 0x00100000 */
#define CAN_F13R1_FB20 CAN_F13R1_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F13R1_FB21_Pos (21U)
#define CAN_F13R1_FB21_Msk (0x1UL << CAN_F13R1_FB21_Pos) /*!< 0x00200000 */
#define CAN_F13R1_FB21 CAN_F13R1_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F13R1_FB22_Pos (22U)
#define CAN_F13R1_FB22_Msk (0x1UL << CAN_F13R1_FB22_Pos) /*!< 0x00400000 */
#define CAN_F13R1_FB22 CAN_F13R1_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F13R1_FB23_Pos (23U)
#define CAN_F13R1_FB23_Msk (0x1UL << CAN_F13R1_FB23_Pos) /*!< 0x00800000 */
#define CAN_F13R1_FB23 CAN_F13R1_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F13R1_FB24_Pos (24U)
#define CAN_F13R1_FB24_Msk (0x1UL << CAN_F13R1_FB24_Pos) /*!< 0x01000000 */
#define CAN_F13R1_FB24 CAN_F13R1_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F13R1_FB25_Pos (25U)
#define CAN_F13R1_FB25_Msk (0x1UL << CAN_F13R1_FB25_Pos) /*!< 0x02000000 */
#define CAN_F13R1_FB25 CAN_F13R1_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F13R1_FB26_Pos (26U)
#define CAN_F13R1_FB26_Msk (0x1UL << CAN_F13R1_FB26_Pos) /*!< 0x04000000 */
#define CAN_F13R1_FB26 CAN_F13R1_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F13R1_FB27_Pos (27U)
#define CAN_F13R1_FB27_Msk (0x1UL << CAN_F13R1_FB27_Pos) /*!< 0x08000000 */
#define CAN_F13R1_FB27 CAN_F13R1_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F13R1_FB28_Pos (28U)
#define CAN_F13R1_FB28_Msk (0x1UL << CAN_F13R1_FB28_Pos) /*!< 0x10000000 */
#define CAN_F13R1_FB28 CAN_F13R1_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F13R1_FB29_Pos (29U)
#define CAN_F13R1_FB29_Msk (0x1UL << CAN_F13R1_FB29_Pos) /*!< 0x20000000 */
#define CAN_F13R1_FB29 CAN_F13R1_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F13R1_FB30_Pos (30U)
#define CAN_F13R1_FB30_Msk (0x1UL << CAN_F13R1_FB30_Pos) /*!< 0x40000000 */
#define CAN_F13R1_FB30 CAN_F13R1_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F13R1_FB31_Pos (31U)
#define CAN_F13R1_FB31_Msk (0x1UL << CAN_F13R1_FB31_Pos) /*!< 0x80000000 */
#define CAN_F13R1_FB31 CAN_F13R1_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F0R2 register  *******************/
#define CAN_F0R2_FB0_Pos (0U)
#define CAN_F0R2_FB0_Msk (0x1UL << CAN_F0R2_FB0_Pos) /*!< 0x00000001 */
#define CAN_F0R2_FB0 CAN_F0R2_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F0R2_FB1_Pos (1U)
#define CAN_F0R2_FB1_Msk (0x1UL << CAN_F0R2_FB1_Pos) /*!< 0x00000002 */
#define CAN_F0R2_FB1 CAN_F0R2_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F0R2_FB2_Pos (2U)
#define CAN_F0R2_FB2_Msk (0x1UL << CAN_F0R2_FB2_Pos) /*!< 0x00000004 */
#define CAN_F0R2_FB2 CAN_F0R2_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F0R2_FB3_Pos (3U)
#define CAN_F0R2_FB3_Msk (0x1UL << CAN_F0R2_FB3_Pos) /*!< 0x00000008 */
#define CAN_F0R2_FB3 CAN_F0R2_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F0R2_FB4_Pos (4U)
#define CAN_F0R2_FB4_Msk (0x1UL << CAN_F0R2_FB4_Pos) /*!< 0x00000010 */
#define CAN_F0R2_FB4 CAN_F0R2_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F0R2_FB5_Pos (5U)
#define CAN_F0R2_FB5_Msk (0x1UL << CAN_F0R2_FB5_Pos) /*!< 0x00000020 */
#define CAN_F0R2_FB5 CAN_F0R2_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F0R2_FB6_Pos (6U)
#define CAN_F0R2_FB6_Msk (0x1UL << CAN_F0R2_FB6_Pos) /*!< 0x00000040 */
#define CAN_F0R2_FB6 CAN_F0R2_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F0R2_FB7_Pos (7U)
#define CAN_F0R2_FB7_Msk (0x1UL << CAN_F0R2_FB7_Pos) /*!< 0x00000080 */
#define CAN_F0R2_FB7 CAN_F0R2_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F0R2_FB8_Pos (8U)
#define CAN_F0R2_FB8_Msk (0x1UL << CAN_F0R2_FB8_Pos) /*!< 0x00000100 */
#define CAN_F0R2_FB8 CAN_F0R2_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F0R2_FB9_Pos (9U)
#define CAN_F0R2_FB9_Msk (0x1UL << CAN_F0R2_FB9_Pos) /*!< 0x00000200 */
#define CAN_F0R2_FB9 CAN_F0R2_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F0R2_FB10_Pos (10U)
#define CAN_F0R2_FB10_Msk (0x1UL << CAN_F0R2_FB10_Pos) /*!< 0x00000400 */
#define CAN_F0R2_FB10 CAN_F0R2_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F0R2_FB11_Pos (11U)
#define CAN_F0R2_FB11_Msk (0x1UL << CAN_F0R2_FB11_Pos) /*!< 0x00000800 */
#define CAN_F0R2_FB11 CAN_F0R2_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F0R2_FB12_Pos (12U)
#define CAN_F0R2_FB12_Msk (0x1UL << CAN_F0R2_FB12_Pos) /*!< 0x00001000 */
#define CAN_F0R2_FB12 CAN_F0R2_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F0R2_FB13_Pos (13U)
#define CAN_F0R2_FB13_Msk (0x1UL << CAN_F0R2_FB13_Pos) /*!< 0x00002000 */
#define CAN_F0R2_FB13 CAN_F0R2_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F0R2_FB14_Pos (14U)
#define CAN_F0R2_FB14_Msk (0x1UL << CAN_F0R2_FB14_Pos) /*!< 0x00004000 */
#define CAN_F0R2_FB14 CAN_F0R2_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F0R2_FB15_Pos (15U)
#define CAN_F0R2_FB15_Msk (0x1UL << CAN_F0R2_FB15_Pos) /*!< 0x00008000 */
#define CAN_F0R2_FB15 CAN_F0R2_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F0R2_FB16_Pos (16U)
#define CAN_F0R2_FB16_Msk (0x1UL << CAN_F0R2_FB16_Pos) /*!< 0x00010000 */
#define CAN_F0R2_FB16 CAN_F0R2_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F0R2_FB17_Pos (17U)
#define CAN_F0R2_FB17_Msk (0x1UL << CAN_F0R2_FB17_Pos) /*!< 0x00020000 */
#define CAN_F0R2_FB17 CAN_F0R2_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F0R2_FB18_Pos (18U)
#define CAN_F0R2_FB18_Msk (0x1UL << CAN_F0R2_FB18_Pos) /*!< 0x00040000 */
#define CAN_F0R2_FB18 CAN_F0R2_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F0R2_FB19_Pos (19U)
#define CAN_F0R2_FB19_Msk (0x1UL << CAN_F0R2_FB19_Pos) /*!< 0x00080000 */
#define CAN_F0R2_FB19 CAN_F0R2_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F0R2_FB20_Pos (20U)
#define CAN_F0R2_FB20_Msk (0x1UL << CAN_F0R2_FB20_Pos) /*!< 0x00100000 */
#define CAN_F0R2_FB20 CAN_F0R2_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F0R2_FB21_Pos (21U)
#define CAN_F0R2_FB21_Msk (0x1UL << CAN_F0R2_FB21_Pos) /*!< 0x00200000 */
#define CAN_F0R2_FB21 CAN_F0R2_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F0R2_FB22_Pos (22U)
#define CAN_F0R2_FB22_Msk (0x1UL << CAN_F0R2_FB22_Pos) /*!< 0x00400000 */
#define CAN_F0R2_FB22 CAN_F0R2_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F0R2_FB23_Pos (23U)
#define CAN_F0R2_FB23_Msk (0x1UL << CAN_F0R2_FB23_Pos) /*!< 0x00800000 */
#define CAN_F0R2_FB23 CAN_F0R2_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F0R2_FB24_Pos (24U)
#define CAN_F0R2_FB24_Msk (0x1UL << CAN_F0R2_FB24_Pos) /*!< 0x01000000 */
#define CAN_F0R2_FB24 CAN_F0R2_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F0R2_FB25_Pos (25U)
#define CAN_F0R2_FB25_Msk (0x1UL << CAN_F0R2_FB25_Pos) /*!< 0x02000000 */
#define CAN_F0R2_FB25 CAN_F0R2_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F0R2_FB26_Pos (26U)
#define CAN_F0R2_FB26_Msk (0x1UL << CAN_F0R2_FB26_Pos) /*!< 0x04000000 */
#define CAN_F0R2_FB26 CAN_F0R2_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F0R2_FB27_Pos (27U)
#define CAN_F0R2_FB27_Msk (0x1UL << CAN_F0R2_FB27_Pos) /*!< 0x08000000 */
#define CAN_F0R2_FB27 CAN_F0R2_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F0R2_FB28_Pos (28U)
#define CAN_F0R2_FB28_Msk (0x1UL << CAN_F0R2_FB28_Pos) /*!< 0x10000000 */
#define CAN_F0R2_FB28 CAN_F0R2_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F0R2_FB29_Pos (29U)
#define CAN_F0R2_FB29_Msk (0x1UL << CAN_F0R2_FB29_Pos) /*!< 0x20000000 */
#define CAN_F0R2_FB29 CAN_F0R2_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F0R2_FB30_Pos (30U)
#define CAN_F0R2_FB30_Msk (0x1UL << CAN_F0R2_FB30_Pos) /*!< 0x40000000 */
#define CAN_F0R2_FB30 CAN_F0R2_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F0R2_FB31_Pos (31U)
#define CAN_F0R2_FB31_Msk (0x1UL << CAN_F0R2_FB31_Pos) /*!< 0x80000000 */
#define CAN_F0R2_FB31 CAN_F0R2_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F1R2 register  *******************/
#define CAN_F1R2_FB0_Pos (0U)
#define CAN_F1R2_FB0_Msk (0x1UL << CAN_F1R2_FB0_Pos) /*!< 0x00000001 */
#define CAN_F1R2_FB0 CAN_F1R2_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F1R2_FB1_Pos (1U)
#define CAN_F1R2_FB1_Msk (0x1UL << CAN_F1R2_FB1_Pos) /*!< 0x00000002 */
#define CAN_F1R2_FB1 CAN_F1R2_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F1R2_FB2_Pos (2U)
#define CAN_F1R2_FB2_Msk (0x1UL << CAN_F1R2_FB2_Pos) /*!< 0x00000004 */
#define CAN_F1R2_FB2 CAN_F1R2_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F1R2_FB3_Pos (3U)
#define CAN_F1R2_FB3_Msk (0x1UL << CAN_F1R2_FB3_Pos) /*!< 0x00000008 */
#define CAN_F1R2_FB3 CAN_F1R2_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F1R2_FB4_Pos (4U)
#define CAN_F1R2_FB4_Msk (0x1UL << CAN_F1R2_FB4_Pos) /*!< 0x00000010 */
#define CAN_F1R2_FB4 CAN_F1R2_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F1R2_FB5_Pos (5U)
#define CAN_F1R2_FB5_Msk (0x1UL << CAN_F1R2_FB5_Pos) /*!< 0x00000020 */
#define CAN_F1R2_FB5 CAN_F1R2_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F1R2_FB6_Pos (6U)
#define CAN_F1R2_FB6_Msk (0x1UL << CAN_F1R2_FB6_Pos) /*!< 0x00000040 */
#define CAN_F1R2_FB6 CAN_F1R2_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F1R2_FB7_Pos (7U)
#define CAN_F1R2_FB7_Msk (0x1UL << CAN_F1R2_FB7_Pos) /*!< 0x00000080 */
#define CAN_F1R2_FB7 CAN_F1R2_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F1R2_FB8_Pos (8U)
#define CAN_F1R2_FB8_Msk (0x1UL << CAN_F1R2_FB8_Pos) /*!< 0x00000100 */
#define CAN_F1R2_FB8 CAN_F1R2_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F1R2_FB9_Pos (9U)
#define CAN_F1R2_FB9_Msk (0x1UL << CAN_F1R2_FB9_Pos) /*!< 0x00000200 */
#define CAN_F1R2_FB9 CAN_F1R2_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F1R2_FB10_Pos (10U)
#define CAN_F1R2_FB10_Msk (0x1UL << CAN_F1R2_FB10_Pos) /*!< 0x00000400 */
#define CAN_F1R2_FB10 CAN_F1R2_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F1R2_FB11_Pos (11U)
#define CAN_F1R2_FB11_Msk (0x1UL << CAN_F1R2_FB11_Pos) /*!< 0x00000800 */
#define CAN_F1R2_FB11 CAN_F1R2_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F1R2_FB12_Pos (12U)
#define CAN_F1R2_FB12_Msk (0x1UL << CAN_F1R2_FB12_Pos) /*!< 0x00001000 */
#define CAN_F1R2_FB12 CAN_F1R2_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F1R2_FB13_Pos (13U)
#define CAN_F1R2_FB13_Msk (0x1UL << CAN_F1R2_FB13_Pos) /*!< 0x00002000 */
#define CAN_F1R2_FB13 CAN_F1R2_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F1R2_FB14_Pos (14U)
#define CAN_F1R2_FB14_Msk (0x1UL << CAN_F1R2_FB14_Pos) /*!< 0x00004000 */
#define CAN_F1R2_FB14 CAN_F1R2_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F1R2_FB15_Pos (15U)
#define CAN_F1R2_FB15_Msk (0x1UL << CAN_F1R2_FB15_Pos) /*!< 0x00008000 */
#define CAN_F1R2_FB15 CAN_F1R2_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F1R2_FB16_Pos (16U)
#define CAN_F1R2_FB16_Msk (0x1UL << CAN_F1R2_FB16_Pos) /*!< 0x00010000 */
#define CAN_F1R2_FB16 CAN_F1R2_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F1R2_FB17_Pos (17U)
#define CAN_F1R2_FB17_Msk (0x1UL << CAN_F1R2_FB17_Pos) /*!< 0x00020000 */
#define CAN_F1R2_FB17 CAN_F1R2_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F1R2_FB18_Pos (18U)
#define CAN_F1R2_FB18_Msk (0x1UL << CAN_F1R2_FB18_Pos) /*!< 0x00040000 */
#define CAN_F1R2_FB18 CAN_F1R2_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F1R2_FB19_Pos (19U)
#define CAN_F1R2_FB19_Msk (0x1UL << CAN_F1R2_FB19_Pos) /*!< 0x00080000 */
#define CAN_F1R2_FB19 CAN_F1R2_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F1R2_FB20_Pos (20U)
#define CAN_F1R2_FB20_Msk (0x1UL << CAN_F1R2_FB20_Pos) /*!< 0x00100000 */
#define CAN_F1R2_FB20 CAN_F1R2_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F1R2_FB21_Pos (21U)
#define CAN_F1R2_FB21_Msk (0x1UL << CAN_F1R2_FB21_Pos) /*!< 0x00200000 */
#define CAN_F1R2_FB21 CAN_F1R2_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F1R2_FB22_Pos (22U)
#define CAN_F1R2_FB22_Msk (0x1UL << CAN_F1R2_FB22_Pos) /*!< 0x00400000 */
#define CAN_F1R2_FB22 CAN_F1R2_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F1R2_FB23_Pos (23U)
#define CAN_F1R2_FB23_Msk (0x1UL << CAN_F1R2_FB23_Pos) /*!< 0x00800000 */
#define CAN_F1R2_FB23 CAN_F1R2_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F1R2_FB24_Pos (24U)
#define CAN_F1R2_FB24_Msk (0x1UL << CAN_F1R2_FB24_Pos) /*!< 0x01000000 */
#define CAN_F1R2_FB24 CAN_F1R2_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F1R2_FB25_Pos (25U)
#define CAN_F1R2_FB25_Msk (0x1UL << CAN_F1R2_FB25_Pos) /*!< 0x02000000 */
#define CAN_F1R2_FB25 CAN_F1R2_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F1R2_FB26_Pos (26U)
#define CAN_F1R2_FB26_Msk (0x1UL << CAN_F1R2_FB26_Pos) /*!< 0x04000000 */
#define CAN_F1R2_FB26 CAN_F1R2_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F1R2_FB27_Pos (27U)
#define CAN_F1R2_FB27_Msk (0x1UL << CAN_F1R2_FB27_Pos) /*!< 0x08000000 */
#define CAN_F1R2_FB27 CAN_F1R2_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F1R2_FB28_Pos (28U)
#define CAN_F1R2_FB28_Msk (0x1UL << CAN_F1R2_FB28_Pos) /*!< 0x10000000 */
#define CAN_F1R2_FB28 CAN_F1R2_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F1R2_FB29_Pos (29U)
#define CAN_F1R2_FB29_Msk (0x1UL << CAN_F1R2_FB29_Pos) /*!< 0x20000000 */
#define CAN_F1R2_FB29 CAN_F1R2_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F1R2_FB30_Pos (30U)
#define CAN_F1R2_FB30_Msk (0x1UL << CAN_F1R2_FB30_Pos) /*!< 0x40000000 */
#define CAN_F1R2_FB30 CAN_F1R2_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F1R2_FB31_Pos (31U)
#define CAN_F1R2_FB31_Msk (0x1UL << CAN_F1R2_FB31_Pos) /*!< 0x80000000 */
#define CAN_F1R2_FB31 CAN_F1R2_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F2R2 register  *******************/
#define CAN_F2R2_FB0_Pos (0U)
#define CAN_F2R2_FB0_Msk (0x1UL << CAN_F2R2_FB0_Pos) /*!< 0x00000001 */
#define CAN_F2R2_FB0 CAN_F2R2_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F2R2_FB1_Pos (1U)
#define CAN_F2R2_FB1_Msk (0x1UL << CAN_F2R2_FB1_Pos) /*!< 0x00000002 */
#define CAN_F2R2_FB1 CAN_F2R2_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F2R2_FB2_Pos (2U)
#define CAN_F2R2_FB2_Msk (0x1UL << CAN_F2R2_FB2_Pos) /*!< 0x00000004 */
#define CAN_F2R2_FB2 CAN_F2R2_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F2R2_FB3_Pos (3U)
#define CAN_F2R2_FB3_Msk (0x1UL << CAN_F2R2_FB3_Pos) /*!< 0x00000008 */
#define CAN_F2R2_FB3 CAN_F2R2_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F2R2_FB4_Pos (4U)
#define CAN_F2R2_FB4_Msk (0x1UL << CAN_F2R2_FB4_Pos) /*!< 0x00000010 */
#define CAN_F2R2_FB4 CAN_F2R2_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F2R2_FB5_Pos (5U)
#define CAN_F2R2_FB5_Msk (0x1UL << CAN_F2R2_FB5_Pos) /*!< 0x00000020 */
#define CAN_F2R2_FB5 CAN_F2R2_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F2R2_FB6_Pos (6U)
#define CAN_F2R2_FB6_Msk (0x1UL << CAN_F2R2_FB6_Pos) /*!< 0x00000040 */
#define CAN_F2R2_FB6 CAN_F2R2_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F2R2_FB7_Pos (7U)
#define CAN_F2R2_FB7_Msk (0x1UL << CAN_F2R2_FB7_Pos) /*!< 0x00000080 */
#define CAN_F2R2_FB7 CAN_F2R2_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F2R2_FB8_Pos (8U)
#define CAN_F2R2_FB8_Msk (0x1UL << CAN_F2R2_FB8_Pos) /*!< 0x00000100 */
#define CAN_F2R2_FB8 CAN_F2R2_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F2R2_FB9_Pos (9U)
#define CAN_F2R2_FB9_Msk (0x1UL << CAN_F2R2_FB9_Pos) /*!< 0x00000200 */
#define CAN_F2R2_FB9 CAN_F2R2_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F2R2_FB10_Pos (10U)
#define CAN_F2R2_FB10_Msk (0x1UL << CAN_F2R2_FB10_Pos) /*!< 0x00000400 */
#define CAN_F2R2_FB10 CAN_F2R2_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F2R2_FB11_Pos (11U)
#define CAN_F2R2_FB11_Msk (0x1UL << CAN_F2R2_FB11_Pos) /*!< 0x00000800 */
#define CAN_F2R2_FB11 CAN_F2R2_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F2R2_FB12_Pos (12U)
#define CAN_F2R2_FB12_Msk (0x1UL << CAN_F2R2_FB12_Pos) /*!< 0x00001000 */
#define CAN_F2R2_FB12 CAN_F2R2_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F2R2_FB13_Pos (13U)
#define CAN_F2R2_FB13_Msk (0x1UL << CAN_F2R2_FB13_Pos) /*!< 0x00002000 */
#define CAN_F2R2_FB13 CAN_F2R2_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F2R2_FB14_Pos (14U)
#define CAN_F2R2_FB14_Msk (0x1UL << CAN_F2R2_FB14_Pos) /*!< 0x00004000 */
#define CAN_F2R2_FB14 CAN_F2R2_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F2R2_FB15_Pos (15U)
#define CAN_F2R2_FB15_Msk (0x1UL << CAN_F2R2_FB15_Pos) /*!< 0x00008000 */
#define CAN_F2R2_FB15 CAN_F2R2_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F2R2_FB16_Pos (16U)
#define CAN_F2R2_FB16_Msk (0x1UL << CAN_F2R2_FB16_Pos) /*!< 0x00010000 */
#define CAN_F2R2_FB16 CAN_F2R2_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F2R2_FB17_Pos (17U)
#define CAN_F2R2_FB17_Msk (0x1UL << CAN_F2R2_FB17_Pos) /*!< 0x00020000 */
#define CAN_F2R2_FB17 CAN_F2R2_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F2R2_FB18_Pos (18U)
#define CAN_F2R2_FB18_Msk (0x1UL << CAN_F2R2_FB18_Pos) /*!< 0x00040000 */
#define CAN_F2R2_FB18 CAN_F2R2_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F2R2_FB19_Pos (19U)
#define CAN_F2R2_FB19_Msk (0x1UL << CAN_F2R2_FB19_Pos) /*!< 0x00080000 */
#define CAN_F2R2_FB19 CAN_F2R2_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F2R2_FB20_Pos (20U)
#define CAN_F2R2_FB20_Msk (0x1UL << CAN_F2R2_FB20_Pos) /*!< 0x00100000 */
#define CAN_F2R2_FB20 CAN_F2R2_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F2R2_FB21_Pos (21U)
#define CAN_F2R2_FB21_Msk (0x1UL << CAN_F2R2_FB21_Pos) /*!< 0x00200000 */
#define CAN_F2R2_FB21 CAN_F2R2_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F2R2_FB22_Pos (22U)
#define CAN_F2R2_FB22_Msk (0x1UL << CAN_F2R2_FB22_Pos) /*!< 0x00400000 */
#define CAN_F2R2_FB22 CAN_F2R2_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F2R2_FB23_Pos (23U)
#define CAN_F2R2_FB23_Msk (0x1UL << CAN_F2R2_FB23_Pos) /*!< 0x00800000 */
#define CAN_F2R2_FB23 CAN_F2R2_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F2R2_FB24_Pos (24U)
#define CAN_F2R2_FB24_Msk (0x1UL << CAN_F2R2_FB24_Pos) /*!< 0x01000000 */
#define CAN_F2R2_FB24 CAN_F2R2_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F2R2_FB25_Pos (25U)
#define CAN_F2R2_FB25_Msk (0x1UL << CAN_F2R2_FB25_Pos) /*!< 0x02000000 */
#define CAN_F2R2_FB25 CAN_F2R2_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F2R2_FB26_Pos (26U)
#define CAN_F2R2_FB26_Msk (0x1UL << CAN_F2R2_FB26_Pos) /*!< 0x04000000 */
#define CAN_F2R2_FB26 CAN_F2R2_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F2R2_FB27_Pos (27U)
#define CAN_F2R2_FB27_Msk (0x1UL << CAN_F2R2_FB27_Pos) /*!< 0x08000000 */
#define CAN_F2R2_FB27 CAN_F2R2_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F2R2_FB28_Pos (28U)
#define CAN_F2R2_FB28_Msk (0x1UL << CAN_F2R2_FB28_Pos) /*!< 0x10000000 */
#define CAN_F2R2_FB28 CAN_F2R2_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F2R2_FB29_Pos (29U)
#define CAN_F2R2_FB29_Msk (0x1UL << CAN_F2R2_FB29_Pos) /*!< 0x20000000 */
#define CAN_F2R2_FB29 CAN_F2R2_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F2R2_FB30_Pos (30U)
#define CAN_F2R2_FB30_Msk (0x1UL << CAN_F2R2_FB30_Pos) /*!< 0x40000000 */
#define CAN_F2R2_FB30 CAN_F2R2_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F2R2_FB31_Pos (31U)
#define CAN_F2R2_FB31_Msk (0x1UL << CAN_F2R2_FB31_Pos) /*!< 0x80000000 */
#define CAN_F2R2_FB31 CAN_F2R2_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F3R2 register  *******************/
#define CAN_F3R2_FB0_Pos (0U)
#define CAN_F3R2_FB0_Msk (0x1UL << CAN_F3R2_FB0_Pos) /*!< 0x00000001 */
#define CAN_F3R2_FB0 CAN_F3R2_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F3R2_FB1_Pos (1U)
#define CAN_F3R2_FB1_Msk (0x1UL << CAN_F3R2_FB1_Pos) /*!< 0x00000002 */
#define CAN_F3R2_FB1 CAN_F3R2_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F3R2_FB2_Pos (2U)
#define CAN_F3R2_FB2_Msk (0x1UL << CAN_F3R2_FB2_Pos) /*!< 0x00000004 */
#define CAN_F3R2_FB2 CAN_F3R2_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F3R2_FB3_Pos (3U)
#define CAN_F3R2_FB3_Msk (0x1UL << CAN_F3R2_FB3_Pos) /*!< 0x00000008 */
#define CAN_F3R2_FB3 CAN_F3R2_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F3R2_FB4_Pos (4U)
#define CAN_F3R2_FB4_Msk (0x1UL << CAN_F3R2_FB4_Pos) /*!< 0x00000010 */
#define CAN_F3R2_FB4 CAN_F3R2_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F3R2_FB5_Pos (5U)
#define CAN_F3R2_FB5_Msk (0x1UL << CAN_F3R2_FB5_Pos) /*!< 0x00000020 */
#define CAN_F3R2_FB5 CAN_F3R2_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F3R2_FB6_Pos (6U)
#define CAN_F3R2_FB6_Msk (0x1UL << CAN_F3R2_FB6_Pos) /*!< 0x00000040 */
#define CAN_F3R2_FB6 CAN_F3R2_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F3R2_FB7_Pos (7U)
#define CAN_F3R2_FB7_Msk (0x1UL << CAN_F3R2_FB7_Pos) /*!< 0x00000080 */
#define CAN_F3R2_FB7 CAN_F3R2_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F3R2_FB8_Pos (8U)
#define CAN_F3R2_FB8_Msk (0x1UL << CAN_F3R2_FB8_Pos) /*!< 0x00000100 */
#define CAN_F3R2_FB8 CAN_F3R2_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F3R2_FB9_Pos (9U)
#define CAN_F3R2_FB9_Msk (0x1UL << CAN_F3R2_FB9_Pos) /*!< 0x00000200 */
#define CAN_F3R2_FB9 CAN_F3R2_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F3R2_FB10_Pos (10U)
#define CAN_F3R2_FB10_Msk (0x1UL << CAN_F3R2_FB10_Pos) /*!< 0x00000400 */
#define CAN_F3R2_FB10 CAN_F3R2_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F3R2_FB11_Pos (11U)
#define CAN_F3R2_FB11_Msk (0x1UL << CAN_F3R2_FB11_Pos) /*!< 0x00000800 */
#define CAN_F3R2_FB11 CAN_F3R2_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F3R2_FB12_Pos (12U)
#define CAN_F3R2_FB12_Msk (0x1UL << CAN_F3R2_FB12_Pos) /*!< 0x00001000 */
#define CAN_F3R2_FB12 CAN_F3R2_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F3R2_FB13_Pos (13U)
#define CAN_F3R2_FB13_Msk (0x1UL << CAN_F3R2_FB13_Pos) /*!< 0x00002000 */
#define CAN_F3R2_FB13 CAN_F3R2_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F3R2_FB14_Pos (14U)
#define CAN_F3R2_FB14_Msk (0x1UL << CAN_F3R2_FB14_Pos) /*!< 0x00004000 */
#define CAN_F3R2_FB14 CAN_F3R2_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F3R2_FB15_Pos (15U)
#define CAN_F3R2_FB15_Msk (0x1UL << CAN_F3R2_FB15_Pos) /*!< 0x00008000 */
#define CAN_F3R2_FB15 CAN_F3R2_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F3R2_FB16_Pos (16U)
#define CAN_F3R2_FB16_Msk (0x1UL << CAN_F3R2_FB16_Pos) /*!< 0x00010000 */
#define CAN_F3R2_FB16 CAN_F3R2_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F3R2_FB17_Pos (17U)
#define CAN_F3R2_FB17_Msk (0x1UL << CAN_F3R2_FB17_Pos) /*!< 0x00020000 */
#define CAN_F3R2_FB17 CAN_F3R2_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F3R2_FB18_Pos (18U)
#define CAN_F3R2_FB18_Msk (0x1UL << CAN_F3R2_FB18_Pos) /*!< 0x00040000 */
#define CAN_F3R2_FB18 CAN_F3R2_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F3R2_FB19_Pos (19U)
#define CAN_F3R2_FB19_Msk (0x1UL << CAN_F3R2_FB19_Pos) /*!< 0x00080000 */
#define CAN_F3R2_FB19 CAN_F3R2_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F3R2_FB20_Pos (20U)
#define CAN_F3R2_FB20_Msk (0x1UL << CAN_F3R2_FB20_Pos) /*!< 0x00100000 */
#define CAN_F3R2_FB20 CAN_F3R2_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F3R2_FB21_Pos (21U)
#define CAN_F3R2_FB21_Msk (0x1UL << CAN_F3R2_FB21_Pos) /*!< 0x00200000 */
#define CAN_F3R2_FB21 CAN_F3R2_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F3R2_FB22_Pos (22U)
#define CAN_F3R2_FB22_Msk (0x1UL << CAN_F3R2_FB22_Pos) /*!< 0x00400000 */
#define CAN_F3R2_FB22 CAN_F3R2_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F3R2_FB23_Pos (23U)
#define CAN_F3R2_FB23_Msk (0x1UL << CAN_F3R2_FB23_Pos) /*!< 0x00800000 */
#define CAN_F3R2_FB23 CAN_F3R2_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F3R2_FB24_Pos (24U)
#define CAN_F3R2_FB24_Msk (0x1UL << CAN_F3R2_FB24_Pos) /*!< 0x01000000 */
#define CAN_F3R2_FB24 CAN_F3R2_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F3R2_FB25_Pos (25U)
#define CAN_F3R2_FB25_Msk (0x1UL << CAN_F3R2_FB25_Pos) /*!< 0x02000000 */
#define CAN_F3R2_FB25 CAN_F3R2_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F3R2_FB26_Pos (26U)
#define CAN_F3R2_FB26_Msk (0x1UL << CAN_F3R2_FB26_Pos) /*!< 0x04000000 */
#define CAN_F3R2_FB26 CAN_F3R2_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F3R2_FB27_Pos (27U)
#define CAN_F3R2_FB27_Msk (0x1UL << CAN_F3R2_FB27_Pos) /*!< 0x08000000 */
#define CAN_F3R2_FB27 CAN_F3R2_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F3R2_FB28_Pos (28U)
#define CAN_F3R2_FB28_Msk (0x1UL << CAN_F3R2_FB28_Pos) /*!< 0x10000000 */
#define CAN_F3R2_FB28 CAN_F3R2_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F3R2_FB29_Pos (29U)
#define CAN_F3R2_FB29_Msk (0x1UL << CAN_F3R2_FB29_Pos) /*!< 0x20000000 */
#define CAN_F3R2_FB29 CAN_F3R2_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F3R2_FB30_Pos (30U)
#define CAN_F3R2_FB30_Msk (0x1UL << CAN_F3R2_FB30_Pos) /*!< 0x40000000 */
#define CAN_F3R2_FB30 CAN_F3R2_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F3R2_FB31_Pos (31U)
#define CAN_F3R2_FB31_Msk (0x1UL << CAN_F3R2_FB31_Pos) /*!< 0x80000000 */
#define CAN_F3R2_FB31 CAN_F3R2_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F4R2 register  *******************/
#define CAN_F4R2_FB0_Pos (0U)
#define CAN_F4R2_FB0_Msk (0x1UL << CAN_F4R2_FB0_Pos) /*!< 0x00000001 */
#define CAN_F4R2_FB0 CAN_F4R2_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F4R2_FB1_Pos (1U)
#define CAN_F4R2_FB1_Msk (0x1UL << CAN_F4R2_FB1_Pos) /*!< 0x00000002 */
#define CAN_F4R2_FB1 CAN_F4R2_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F4R2_FB2_Pos (2U)
#define CAN_F4R2_FB2_Msk (0x1UL << CAN_F4R2_FB2_Pos) /*!< 0x00000004 */
#define CAN_F4R2_FB2 CAN_F4R2_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F4R2_FB3_Pos (3U)
#define CAN_F4R2_FB3_Msk (0x1UL << CAN_F4R2_FB3_Pos) /*!< 0x00000008 */
#define CAN_F4R2_FB3 CAN_F4R2_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F4R2_FB4_Pos (4U)
#define CAN_F4R2_FB4_Msk (0x1UL << CAN_F4R2_FB4_Pos) /*!< 0x00000010 */
#define CAN_F4R2_FB4 CAN_F4R2_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F4R2_FB5_Pos (5U)
#define CAN_F4R2_FB5_Msk (0x1UL << CAN_F4R2_FB5_Pos) /*!< 0x00000020 */
#define CAN_F4R2_FB5 CAN_F4R2_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F4R2_FB6_Pos (6U)
#define CAN_F4R2_FB6_Msk (0x1UL << CAN_F4R2_FB6_Pos) /*!< 0x00000040 */
#define CAN_F4R2_FB6 CAN_F4R2_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F4R2_FB7_Pos (7U)
#define CAN_F4R2_FB7_Msk (0x1UL << CAN_F4R2_FB7_Pos) /*!< 0x00000080 */
#define CAN_F4R2_FB7 CAN_F4R2_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F4R2_FB8_Pos (8U)
#define CAN_F4R2_FB8_Msk (0x1UL << CAN_F4R2_FB8_Pos) /*!< 0x00000100 */
#define CAN_F4R2_FB8 CAN_F4R2_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F4R2_FB9_Pos (9U)
#define CAN_F4R2_FB9_Msk (0x1UL << CAN_F4R2_FB9_Pos) /*!< 0x00000200 */
#define CAN_F4R2_FB9 CAN_F4R2_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F4R2_FB10_Pos (10U)
#define CAN_F4R2_FB10_Msk (0x1UL << CAN_F4R2_FB10_Pos) /*!< 0x00000400 */
#define CAN_F4R2_FB10 CAN_F4R2_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F4R2_FB11_Pos (11U)
#define CAN_F4R2_FB11_Msk (0x1UL << CAN_F4R2_FB11_Pos) /*!< 0x00000800 */
#define CAN_F4R2_FB11 CAN_F4R2_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F4R2_FB12_Pos (12U)
#define CAN_F4R2_FB12_Msk (0x1UL << CAN_F4R2_FB12_Pos) /*!< 0x00001000 */
#define CAN_F4R2_FB12 CAN_F4R2_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F4R2_FB13_Pos (13U)
#define CAN_F4R2_FB13_Msk (0x1UL << CAN_F4R2_FB13_Pos) /*!< 0x00002000 */
#define CAN_F4R2_FB13 CAN_F4R2_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F4R2_FB14_Pos (14U)
#define CAN_F4R2_FB14_Msk (0x1UL << CAN_F4R2_FB14_Pos) /*!< 0x00004000 */
#define CAN_F4R2_FB14 CAN_F4R2_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F4R2_FB15_Pos (15U)
#define CAN_F4R2_FB15_Msk (0x1UL << CAN_F4R2_FB15_Pos) /*!< 0x00008000 */
#define CAN_F4R2_FB15 CAN_F4R2_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F4R2_FB16_Pos (16U)
#define CAN_F4R2_FB16_Msk (0x1UL << CAN_F4R2_FB16_Pos) /*!< 0x00010000 */
#define CAN_F4R2_FB16 CAN_F4R2_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F4R2_FB17_Pos (17U)
#define CAN_F4R2_FB17_Msk (0x1UL << CAN_F4R2_FB17_Pos) /*!< 0x00020000 */
#define CAN_F4R2_FB17 CAN_F4R2_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F4R2_FB18_Pos (18U)
#define CAN_F4R2_FB18_Msk (0x1UL << CAN_F4R2_FB18_Pos) /*!< 0x00040000 */
#define CAN_F4R2_FB18 CAN_F4R2_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F4R2_FB19_Pos (19U)
#define CAN_F4R2_FB19_Msk (0x1UL << CAN_F4R2_FB19_Pos) /*!< 0x00080000 */
#define CAN_F4R2_FB19 CAN_F4R2_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F4R2_FB20_Pos (20U)
#define CAN_F4R2_FB20_Msk (0x1UL << CAN_F4R2_FB20_Pos) /*!< 0x00100000 */
#define CAN_F4R2_FB20 CAN_F4R2_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F4R2_FB21_Pos (21U)
#define CAN_F4R2_FB21_Msk (0x1UL << CAN_F4R2_FB21_Pos) /*!< 0x00200000 */
#define CAN_F4R2_FB21 CAN_F4R2_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F4R2_FB22_Pos (22U)
#define CAN_F4R2_FB22_Msk (0x1UL << CAN_F4R2_FB22_Pos) /*!< 0x00400000 */
#define CAN_F4R2_FB22 CAN_F4R2_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F4R2_FB23_Pos (23U)
#define CAN_F4R2_FB23_Msk (0x1UL << CAN_F4R2_FB23_Pos) /*!< 0x00800000 */
#define CAN_F4R2_FB23 CAN_F4R2_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F4R2_FB24_Pos (24U)
#define CAN_F4R2_FB24_Msk (0x1UL << CAN_F4R2_FB24_Pos) /*!< 0x01000000 */
#define CAN_F4R2_FB24 CAN_F4R2_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F4R2_FB25_Pos (25U)
#define CAN_F4R2_FB25_Msk (0x1UL << CAN_F4R2_FB25_Pos) /*!< 0x02000000 */
#define CAN_F4R2_FB25 CAN_F4R2_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F4R2_FB26_Pos (26U)
#define CAN_F4R2_FB26_Msk (0x1UL << CAN_F4R2_FB26_Pos) /*!< 0x04000000 */
#define CAN_F4R2_FB26 CAN_F4R2_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F4R2_FB27_Pos (27U)
#define CAN_F4R2_FB27_Msk (0x1UL << CAN_F4R2_FB27_Pos) /*!< 0x08000000 */
#define CAN_F4R2_FB27 CAN_F4R2_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F4R2_FB28_Pos (28U)
#define CAN_F4R2_FB28_Msk (0x1UL << CAN_F4R2_FB28_Pos) /*!< 0x10000000 */
#define CAN_F4R2_FB28 CAN_F4R2_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F4R2_FB29_Pos (29U)
#define CAN_F4R2_FB29_Msk (0x1UL << CAN_F4R2_FB29_Pos) /*!< 0x20000000 */
#define CAN_F4R2_FB29 CAN_F4R2_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F4R2_FB30_Pos (30U)
#define CAN_F4R2_FB30_Msk (0x1UL << CAN_F4R2_FB30_Pos) /*!< 0x40000000 */
#define CAN_F4R2_FB30 CAN_F4R2_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F4R2_FB31_Pos (31U)
#define CAN_F4R2_FB31_Msk (0x1UL << CAN_F4R2_FB31_Pos) /*!< 0x80000000 */
#define CAN_F4R2_FB31 CAN_F4R2_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F5R2 register  *******************/
#define CAN_F5R2_FB0_Pos (0U)
#define CAN_F5R2_FB0_Msk (0x1UL << CAN_F5R2_FB0_Pos) /*!< 0x00000001 */
#define CAN_F5R2_FB0 CAN_F5R2_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F5R2_FB1_Pos (1U)
#define CAN_F5R2_FB1_Msk (0x1UL << CAN_F5R2_FB1_Pos) /*!< 0x00000002 */
#define CAN_F5R2_FB1 CAN_F5R2_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F5R2_FB2_Pos (2U)
#define CAN_F5R2_FB2_Msk (0x1UL << CAN_F5R2_FB2_Pos) /*!< 0x00000004 */
#define CAN_F5R2_FB2 CAN_F5R2_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F5R2_FB3_Pos (3U)
#define CAN_F5R2_FB3_Msk (0x1UL << CAN_F5R2_FB3_Pos) /*!< 0x00000008 */
#define CAN_F5R2_FB3 CAN_F5R2_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F5R2_FB4_Pos (4U)
#define CAN_F5R2_FB4_Msk (0x1UL << CAN_F5R2_FB4_Pos) /*!< 0x00000010 */
#define CAN_F5R2_FB4 CAN_F5R2_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F5R2_FB5_Pos (5U)
#define CAN_F5R2_FB5_Msk (0x1UL << CAN_F5R2_FB5_Pos) /*!< 0x00000020 */
#define CAN_F5R2_FB5 CAN_F5R2_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F5R2_FB6_Pos (6U)
#define CAN_F5R2_FB6_Msk (0x1UL << CAN_F5R2_FB6_Pos) /*!< 0x00000040 */
#define CAN_F5R2_FB6 CAN_F5R2_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F5R2_FB7_Pos (7U)
#define CAN_F5R2_FB7_Msk (0x1UL << CAN_F5R2_FB7_Pos) /*!< 0x00000080 */
#define CAN_F5R2_FB7 CAN_F5R2_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F5R2_FB8_Pos (8U)
#define CAN_F5R2_FB8_Msk (0x1UL << CAN_F5R2_FB8_Pos) /*!< 0x00000100 */
#define CAN_F5R2_FB8 CAN_F5R2_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F5R2_FB9_Pos (9U)
#define CAN_F5R2_FB9_Msk (0x1UL << CAN_F5R2_FB9_Pos) /*!< 0x00000200 */
#define CAN_F5R2_FB9 CAN_F5R2_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F5R2_FB10_Pos (10U)
#define CAN_F5R2_FB10_Msk (0x1UL << CAN_F5R2_FB10_Pos) /*!< 0x00000400 */
#define CAN_F5R2_FB10 CAN_F5R2_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F5R2_FB11_Pos (11U)
#define CAN_F5R2_FB11_Msk (0x1UL << CAN_F5R2_FB11_Pos) /*!< 0x00000800 */
#define CAN_F5R2_FB11 CAN_F5R2_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F5R2_FB12_Pos (12U)
#define CAN_F5R2_FB12_Msk (0x1UL << CAN_F5R2_FB12_Pos) /*!< 0x00001000 */
#define CAN_F5R2_FB12 CAN_F5R2_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F5R2_FB13_Pos (13U)
#define CAN_F5R2_FB13_Msk (0x1UL << CAN_F5R2_FB13_Pos) /*!< 0x00002000 */
#define CAN_F5R2_FB13 CAN_F5R2_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F5R2_FB14_Pos (14U)
#define CAN_F5R2_FB14_Msk (0x1UL << CAN_F5R2_FB14_Pos) /*!< 0x00004000 */
#define CAN_F5R2_FB14 CAN_F5R2_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F5R2_FB15_Pos (15U)
#define CAN_F5R2_FB15_Msk (0x1UL << CAN_F5R2_FB15_Pos) /*!< 0x00008000 */
#define CAN_F5R2_FB15 CAN_F5R2_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F5R2_FB16_Pos (16U)
#define CAN_F5R2_FB16_Msk (0x1UL << CAN_F5R2_FB16_Pos) /*!< 0x00010000 */
#define CAN_F5R2_FB16 CAN_F5R2_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F5R2_FB17_Pos (17U)
#define CAN_F5R2_FB17_Msk (0x1UL << CAN_F5R2_FB17_Pos) /*!< 0x00020000 */
#define CAN_F5R2_FB17 CAN_F5R2_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F5R2_FB18_Pos (18U)
#define CAN_F5R2_FB18_Msk (0x1UL << CAN_F5R2_FB18_Pos) /*!< 0x00040000 */
#define CAN_F5R2_FB18 CAN_F5R2_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F5R2_FB19_Pos (19U)
#define CAN_F5R2_FB19_Msk (0x1UL << CAN_F5R2_FB19_Pos) /*!< 0x00080000 */
#define CAN_F5R2_FB19 CAN_F5R2_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F5R2_FB20_Pos (20U)
#define CAN_F5R2_FB20_Msk (0x1UL << CAN_F5R2_FB20_Pos) /*!< 0x00100000 */
#define CAN_F5R2_FB20 CAN_F5R2_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F5R2_FB21_Pos (21U)
#define CAN_F5R2_FB21_Msk (0x1UL << CAN_F5R2_FB21_Pos) /*!< 0x00200000 */
#define CAN_F5R2_FB21 CAN_F5R2_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F5R2_FB22_Pos (22U)
#define CAN_F5R2_FB22_Msk (0x1UL << CAN_F5R2_FB22_Pos) /*!< 0x00400000 */
#define CAN_F5R2_FB22 CAN_F5R2_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F5R2_FB23_Pos (23U)
#define CAN_F5R2_FB23_Msk (0x1UL << CAN_F5R2_FB23_Pos) /*!< 0x00800000 */
#define CAN_F5R2_FB23 CAN_F5R2_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F5R2_FB24_Pos (24U)
#define CAN_F5R2_FB24_Msk (0x1UL << CAN_F5R2_FB24_Pos) /*!< 0x01000000 */
#define CAN_F5R2_FB24 CAN_F5R2_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F5R2_FB25_Pos (25U)
#define CAN_F5R2_FB25_Msk (0x1UL << CAN_F5R2_FB25_Pos) /*!< 0x02000000 */
#define CAN_F5R2_FB25 CAN_F5R2_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F5R2_FB26_Pos (26U)
#define CAN_F5R2_FB26_Msk (0x1UL << CAN_F5R2_FB26_Pos) /*!< 0x04000000 */
#define CAN_F5R2_FB26 CAN_F5R2_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F5R2_FB27_Pos (27U)
#define CAN_F5R2_FB27_Msk (0x1UL << CAN_F5R2_FB27_Pos) /*!< 0x08000000 */
#define CAN_F5R2_FB27 CAN_F5R2_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F5R2_FB28_Pos (28U)
#define CAN_F5R2_FB28_Msk (0x1UL << CAN_F5R2_FB28_Pos) /*!< 0x10000000 */
#define CAN_F5R2_FB28 CAN_F5R2_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F5R2_FB29_Pos (29U)
#define CAN_F5R2_FB29_Msk (0x1UL << CAN_F5R2_FB29_Pos) /*!< 0x20000000 */
#define CAN_F5R2_FB29 CAN_F5R2_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F5R2_FB30_Pos (30U)
#define CAN_F5R2_FB30_Msk (0x1UL << CAN_F5R2_FB30_Pos) /*!< 0x40000000 */
#define CAN_F5R2_FB30 CAN_F5R2_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F5R2_FB31_Pos (31U)
#define CAN_F5R2_FB31_Msk (0x1UL << CAN_F5R2_FB31_Pos) /*!< 0x80000000 */
#define CAN_F5R2_FB31 CAN_F5R2_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F6R2 register  *******************/
#define CAN_F6R2_FB0_Pos (0U)
#define CAN_F6R2_FB0_Msk (0x1UL << CAN_F6R2_FB0_Pos) /*!< 0x00000001 */
#define CAN_F6R2_FB0 CAN_F6R2_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F6R2_FB1_Pos (1U)
#define CAN_F6R2_FB1_Msk (0x1UL << CAN_F6R2_FB1_Pos) /*!< 0x00000002 */
#define CAN_F6R2_FB1 CAN_F6R2_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F6R2_FB2_Pos (2U)
#define CAN_F6R2_FB2_Msk (0x1UL << CAN_F6R2_FB2_Pos) /*!< 0x00000004 */
#define CAN_F6R2_FB2 CAN_F6R2_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F6R2_FB3_Pos (3U)
#define CAN_F6R2_FB3_Msk (0x1UL << CAN_F6R2_FB3_Pos) /*!< 0x00000008 */
#define CAN_F6R2_FB3 CAN_F6R2_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F6R2_FB4_Pos (4U)
#define CAN_F6R2_FB4_Msk (0x1UL << CAN_F6R2_FB4_Pos) /*!< 0x00000010 */
#define CAN_F6R2_FB4 CAN_F6R2_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F6R2_FB5_Pos (5U)
#define CAN_F6R2_FB5_Msk (0x1UL << CAN_F6R2_FB5_Pos) /*!< 0x00000020 */
#define CAN_F6R2_FB5 CAN_F6R2_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F6R2_FB6_Pos (6U)
#define CAN_F6R2_FB6_Msk (0x1UL << CAN_F6R2_FB6_Pos) /*!< 0x00000040 */
#define CAN_F6R2_FB6 CAN_F6R2_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F6R2_FB7_Pos (7U)
#define CAN_F6R2_FB7_Msk (0x1UL << CAN_F6R2_FB7_Pos) /*!< 0x00000080 */
#define CAN_F6R2_FB7 CAN_F6R2_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F6R2_FB8_Pos (8U)
#define CAN_F6R2_FB8_Msk (0x1UL << CAN_F6R2_FB8_Pos) /*!< 0x00000100 */
#define CAN_F6R2_FB8 CAN_F6R2_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F6R2_FB9_Pos (9U)
#define CAN_F6R2_FB9_Msk (0x1UL << CAN_F6R2_FB9_Pos) /*!< 0x00000200 */
#define CAN_F6R2_FB9 CAN_F6R2_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F6R2_FB10_Pos (10U)
#define CAN_F6R2_FB10_Msk (0x1UL << CAN_F6R2_FB10_Pos) /*!< 0x00000400 */
#define CAN_F6R2_FB10 CAN_F6R2_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F6R2_FB11_Pos (11U)
#define CAN_F6R2_FB11_Msk (0x1UL << CAN_F6R2_FB11_Pos) /*!< 0x00000800 */
#define CAN_F6R2_FB11 CAN_F6R2_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F6R2_FB12_Pos (12U)
#define CAN_F6R2_FB12_Msk (0x1UL << CAN_F6R2_FB12_Pos) /*!< 0x00001000 */
#define CAN_F6R2_FB12 CAN_F6R2_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F6R2_FB13_Pos (13U)
#define CAN_F6R2_FB13_Msk (0x1UL << CAN_F6R2_FB13_Pos) /*!< 0x00002000 */
#define CAN_F6R2_FB13 CAN_F6R2_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F6R2_FB14_Pos (14U)
#define CAN_F6R2_FB14_Msk (0x1UL << CAN_F6R2_FB14_Pos) /*!< 0x00004000 */
#define CAN_F6R2_FB14 CAN_F6R2_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F6R2_FB15_Pos (15U)
#define CAN_F6R2_FB15_Msk (0x1UL << CAN_F6R2_FB15_Pos) /*!< 0x00008000 */
#define CAN_F6R2_FB15 CAN_F6R2_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F6R2_FB16_Pos (16U)
#define CAN_F6R2_FB16_Msk (0x1UL << CAN_F6R2_FB16_Pos) /*!< 0x00010000 */
#define CAN_F6R2_FB16 CAN_F6R2_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F6R2_FB17_Pos (17U)
#define CAN_F6R2_FB17_Msk (0x1UL << CAN_F6R2_FB17_Pos) /*!< 0x00020000 */
#define CAN_F6R2_FB17 CAN_F6R2_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F6R2_FB18_Pos (18U)
#define CAN_F6R2_FB18_Msk (0x1UL << CAN_F6R2_FB18_Pos) /*!< 0x00040000 */
#define CAN_F6R2_FB18 CAN_F6R2_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F6R2_FB19_Pos (19U)
#define CAN_F6R2_FB19_Msk (0x1UL << CAN_F6R2_FB19_Pos) /*!< 0x00080000 */
#define CAN_F6R2_FB19 CAN_F6R2_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F6R2_FB20_Pos (20U)
#define CAN_F6R2_FB20_Msk (0x1UL << CAN_F6R2_FB20_Pos) /*!< 0x00100000 */
#define CAN_F6R2_FB20 CAN_F6R2_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F6R2_FB21_Pos (21U)
#define CAN_F6R2_FB21_Msk (0x1UL << CAN_F6R2_FB21_Pos) /*!< 0x00200000 */
#define CAN_F6R2_FB21 CAN_F6R2_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F6R2_FB22_Pos (22U)
#define CAN_F6R2_FB22_Msk (0x1UL << CAN_F6R2_FB22_Pos) /*!< 0x00400000 */
#define CAN_F6R2_FB22 CAN_F6R2_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F6R2_FB23_Pos (23U)
#define CAN_F6R2_FB23_Msk (0x1UL << CAN_F6R2_FB23_Pos) /*!< 0x00800000 */
#define CAN_F6R2_FB23 CAN_F6R2_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F6R2_FB24_Pos (24U)
#define CAN_F6R2_FB24_Msk (0x1UL << CAN_F6R2_FB24_Pos) /*!< 0x01000000 */
#define CAN_F6R2_FB24 CAN_F6R2_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F6R2_FB25_Pos (25U)
#define CAN_F6R2_FB25_Msk (0x1UL << CAN_F6R2_FB25_Pos) /*!< 0x02000000 */
#define CAN_F6R2_FB25 CAN_F6R2_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F6R2_FB26_Pos (26U)
#define CAN_F6R2_FB26_Msk (0x1UL << CAN_F6R2_FB26_Pos) /*!< 0x04000000 */
#define CAN_F6R2_FB26 CAN_F6R2_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F6R2_FB27_Pos (27U)
#define CAN_F6R2_FB27_Msk (0x1UL << CAN_F6R2_FB27_Pos) /*!< 0x08000000 */
#define CAN_F6R2_FB27 CAN_F6R2_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F6R2_FB28_Pos (28U)
#define CAN_F6R2_FB28_Msk (0x1UL << CAN_F6R2_FB28_Pos) /*!< 0x10000000 */
#define CAN_F6R2_FB28 CAN_F6R2_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F6R2_FB29_Pos (29U)
#define CAN_F6R2_FB29_Msk (0x1UL << CAN_F6R2_FB29_Pos) /*!< 0x20000000 */
#define CAN_F6R2_FB29 CAN_F6R2_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F6R2_FB30_Pos (30U)
#define CAN_F6R2_FB30_Msk (0x1UL << CAN_F6R2_FB30_Pos) /*!< 0x40000000 */
#define CAN_F6R2_FB30 CAN_F6R2_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F6R2_FB31_Pos (31U)
#define CAN_F6R2_FB31_Msk (0x1UL << CAN_F6R2_FB31_Pos) /*!< 0x80000000 */
#define CAN_F6R2_FB31 CAN_F6R2_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F7R2 register  *******************/
#define CAN_F7R2_FB0_Pos (0U)
#define CAN_F7R2_FB0_Msk (0x1UL << CAN_F7R2_FB0_Pos) /*!< 0x00000001 */
#define CAN_F7R2_FB0 CAN_F7R2_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F7R2_FB1_Pos (1U)
#define CAN_F7R2_FB1_Msk (0x1UL << CAN_F7R2_FB1_Pos) /*!< 0x00000002 */
#define CAN_F7R2_FB1 CAN_F7R2_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F7R2_FB2_Pos (2U)
#define CAN_F7R2_FB2_Msk (0x1UL << CAN_F7R2_FB2_Pos) /*!< 0x00000004 */
#define CAN_F7R2_FB2 CAN_F7R2_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F7R2_FB3_Pos (3U)
#define CAN_F7R2_FB3_Msk (0x1UL << CAN_F7R2_FB3_Pos) /*!< 0x00000008 */
#define CAN_F7R2_FB3 CAN_F7R2_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F7R2_FB4_Pos (4U)
#define CAN_F7R2_FB4_Msk (0x1UL << CAN_F7R2_FB4_Pos) /*!< 0x00000010 */
#define CAN_F7R2_FB4 CAN_F7R2_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F7R2_FB5_Pos (5U)
#define CAN_F7R2_FB5_Msk (0x1UL << CAN_F7R2_FB5_Pos) /*!< 0x00000020 */
#define CAN_F7R2_FB5 CAN_F7R2_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F7R2_FB6_Pos (6U)
#define CAN_F7R2_FB6_Msk (0x1UL << CAN_F7R2_FB6_Pos) /*!< 0x00000040 */
#define CAN_F7R2_FB6 CAN_F7R2_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F7R2_FB7_Pos (7U)
#define CAN_F7R2_FB7_Msk (0x1UL << CAN_F7R2_FB7_Pos) /*!< 0x00000080 */
#define CAN_F7R2_FB7 CAN_F7R2_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F7R2_FB8_Pos (8U)
#define CAN_F7R2_FB8_Msk (0x1UL << CAN_F7R2_FB8_Pos) /*!< 0x00000100 */
#define CAN_F7R2_FB8 CAN_F7R2_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F7R2_FB9_Pos (9U)
#define CAN_F7R2_FB9_Msk (0x1UL << CAN_F7R2_FB9_Pos) /*!< 0x00000200 */
#define CAN_F7R2_FB9 CAN_F7R2_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F7R2_FB10_Pos (10U)
#define CAN_F7R2_FB10_Msk (0x1UL << CAN_F7R2_FB10_Pos) /*!< 0x00000400 */
#define CAN_F7R2_FB10 CAN_F7R2_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F7R2_FB11_Pos (11U)
#define CAN_F7R2_FB11_Msk (0x1UL << CAN_F7R2_FB11_Pos) /*!< 0x00000800 */
#define CAN_F7R2_FB11 CAN_F7R2_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F7R2_FB12_Pos (12U)
#define CAN_F7R2_FB12_Msk (0x1UL << CAN_F7R2_FB12_Pos) /*!< 0x00001000 */
#define CAN_F7R2_FB12 CAN_F7R2_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F7R2_FB13_Pos (13U)
#define CAN_F7R2_FB13_Msk (0x1UL << CAN_F7R2_FB13_Pos) /*!< 0x00002000 */
#define CAN_F7R2_FB13 CAN_F7R2_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F7R2_FB14_Pos (14U)
#define CAN_F7R2_FB14_Msk (0x1UL << CAN_F7R2_FB14_Pos) /*!< 0x00004000 */
#define CAN_F7R2_FB14 CAN_F7R2_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F7R2_FB15_Pos (15U)
#define CAN_F7R2_FB15_Msk (0x1UL << CAN_F7R2_FB15_Pos) /*!< 0x00008000 */
#define CAN_F7R2_FB15 CAN_F7R2_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F7R2_FB16_Pos (16U)
#define CAN_F7R2_FB16_Msk (0x1UL << CAN_F7R2_FB16_Pos) /*!< 0x00010000 */
#define CAN_F7R2_FB16 CAN_F7R2_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F7R2_FB17_Pos (17U)
#define CAN_F7R2_FB17_Msk (0x1UL << CAN_F7R2_FB17_Pos) /*!< 0x00020000 */
#define CAN_F7R2_FB17 CAN_F7R2_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F7R2_FB18_Pos (18U)
#define CAN_F7R2_FB18_Msk (0x1UL << CAN_F7R2_FB18_Pos) /*!< 0x00040000 */
#define CAN_F7R2_FB18 CAN_F7R2_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F7R2_FB19_Pos (19U)
#define CAN_F7R2_FB19_Msk (0x1UL << CAN_F7R2_FB19_Pos) /*!< 0x00080000 */
#define CAN_F7R2_FB19 CAN_F7R2_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F7R2_FB20_Pos (20U)
#define CAN_F7R2_FB20_Msk (0x1UL << CAN_F7R2_FB20_Pos) /*!< 0x00100000 */
#define CAN_F7R2_FB20 CAN_F7R2_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F7R2_FB21_Pos (21U)
#define CAN_F7R2_FB21_Msk (0x1UL << CAN_F7R2_FB21_Pos) /*!< 0x00200000 */
#define CAN_F7R2_FB21 CAN_F7R2_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F7R2_FB22_Pos (22U)
#define CAN_F7R2_FB22_Msk (0x1UL << CAN_F7R2_FB22_Pos) /*!< 0x00400000 */
#define CAN_F7R2_FB22 CAN_F7R2_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F7R2_FB23_Pos (23U)
#define CAN_F7R2_FB23_Msk (0x1UL << CAN_F7R2_FB23_Pos) /*!< 0x00800000 */
#define CAN_F7R2_FB23 CAN_F7R2_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F7R2_FB24_Pos (24U)
#define CAN_F7R2_FB24_Msk (0x1UL << CAN_F7R2_FB24_Pos) /*!< 0x01000000 */
#define CAN_F7R2_FB24 CAN_F7R2_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F7R2_FB25_Pos (25U)
#define CAN_F7R2_FB25_Msk (0x1UL << CAN_F7R2_FB25_Pos) /*!< 0x02000000 */
#define CAN_F7R2_FB25 CAN_F7R2_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F7R2_FB26_Pos (26U)
#define CAN_F7R2_FB26_Msk (0x1UL << CAN_F7R2_FB26_Pos) /*!< 0x04000000 */
#define CAN_F7R2_FB26 CAN_F7R2_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F7R2_FB27_Pos (27U)
#define CAN_F7R2_FB27_Msk (0x1UL << CAN_F7R2_FB27_Pos) /*!< 0x08000000 */
#define CAN_F7R2_FB27 CAN_F7R2_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F7R2_FB28_Pos (28U)
#define CAN_F7R2_FB28_Msk (0x1UL << CAN_F7R2_FB28_Pos) /*!< 0x10000000 */
#define CAN_F7R2_FB28 CAN_F7R2_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F7R2_FB29_Pos (29U)
#define CAN_F7R2_FB29_Msk (0x1UL << CAN_F7R2_FB29_Pos) /*!< 0x20000000 */
#define CAN_F7R2_FB29 CAN_F7R2_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F7R2_FB30_Pos (30U)
#define CAN_F7R2_FB30_Msk (0x1UL << CAN_F7R2_FB30_Pos) /*!< 0x40000000 */
#define CAN_F7R2_FB30 CAN_F7R2_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F7R2_FB31_Pos (31U)
#define CAN_F7R2_FB31_Msk (0x1UL << CAN_F7R2_FB31_Pos) /*!< 0x80000000 */
#define CAN_F7R2_FB31 CAN_F7R2_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F8R2 register  *******************/
#define CAN_F8R2_FB0_Pos (0U)
#define CAN_F8R2_FB0_Msk (0x1UL << CAN_F8R2_FB0_Pos) /*!< 0x00000001 */
#define CAN_F8R2_FB0 CAN_F8R2_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F8R2_FB1_Pos (1U)
#define CAN_F8R2_FB1_Msk (0x1UL << CAN_F8R2_FB1_Pos) /*!< 0x00000002 */
#define CAN_F8R2_FB1 CAN_F8R2_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F8R2_FB2_Pos (2U)
#define CAN_F8R2_FB2_Msk (0x1UL << CAN_F8R2_FB2_Pos) /*!< 0x00000004 */
#define CAN_F8R2_FB2 CAN_F8R2_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F8R2_FB3_Pos (3U)
#define CAN_F8R2_FB3_Msk (0x1UL << CAN_F8R2_FB3_Pos) /*!< 0x00000008 */
#define CAN_F8R2_FB3 CAN_F8R2_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F8R2_FB4_Pos (4U)
#define CAN_F8R2_FB4_Msk (0x1UL << CAN_F8R2_FB4_Pos) /*!< 0x00000010 */
#define CAN_F8R2_FB4 CAN_F8R2_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F8R2_FB5_Pos (5U)
#define CAN_F8R2_FB5_Msk (0x1UL << CAN_F8R2_FB5_Pos) /*!< 0x00000020 */
#define CAN_F8R2_FB5 CAN_F8R2_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F8R2_FB6_Pos (6U)
#define CAN_F8R2_FB6_Msk (0x1UL << CAN_F8R2_FB6_Pos) /*!< 0x00000040 */
#define CAN_F8R2_FB6 CAN_F8R2_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F8R2_FB7_Pos (7U)
#define CAN_F8R2_FB7_Msk (0x1UL << CAN_F8R2_FB7_Pos) /*!< 0x00000080 */
#define CAN_F8R2_FB7 CAN_F8R2_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F8R2_FB8_Pos (8U)
#define CAN_F8R2_FB8_Msk (0x1UL << CAN_F8R2_FB8_Pos) /*!< 0x00000100 */
#define CAN_F8R2_FB8 CAN_F8R2_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F8R2_FB9_Pos (9U)
#define CAN_F8R2_FB9_Msk (0x1UL << CAN_F8R2_FB9_Pos) /*!< 0x00000200 */
#define CAN_F8R2_FB9 CAN_F8R2_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F8R2_FB10_Pos (10U)
#define CAN_F8R2_FB10_Msk (0x1UL << CAN_F8R2_FB10_Pos) /*!< 0x00000400 */
#define CAN_F8R2_FB10 CAN_F8R2_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F8R2_FB11_Pos (11U)
#define CAN_F8R2_FB11_Msk (0x1UL << CAN_F8R2_FB11_Pos) /*!< 0x00000800 */
#define CAN_F8R2_FB11 CAN_F8R2_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F8R2_FB12_Pos (12U)
#define CAN_F8R2_FB12_Msk (0x1UL << CAN_F8R2_FB12_Pos) /*!< 0x00001000 */
#define CAN_F8R2_FB12 CAN_F8R2_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F8R2_FB13_Pos (13U)
#define CAN_F8R2_FB13_Msk (0x1UL << CAN_F8R2_FB13_Pos) /*!< 0x00002000 */
#define CAN_F8R2_FB13 CAN_F8R2_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F8R2_FB14_Pos (14U)
#define CAN_F8R2_FB14_Msk (0x1UL << CAN_F8R2_FB14_Pos) /*!< 0x00004000 */
#define CAN_F8R2_FB14 CAN_F8R2_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F8R2_FB15_Pos (15U)
#define CAN_F8R2_FB15_Msk (0x1UL << CAN_F8R2_FB15_Pos) /*!< 0x00008000 */
#define CAN_F8R2_FB15 CAN_F8R2_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F8R2_FB16_Pos (16U)
#define CAN_F8R2_FB16_Msk (0x1UL << CAN_F8R2_FB16_Pos) /*!< 0x00010000 */
#define CAN_F8R2_FB16 CAN_F8R2_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F8R2_FB17_Pos (17U)
#define CAN_F8R2_FB17_Msk (0x1UL << CAN_F8R2_FB17_Pos) /*!< 0x00020000 */
#define CAN_F8R2_FB17 CAN_F8R2_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F8R2_FB18_Pos (18U)
#define CAN_F8R2_FB18_Msk (0x1UL << CAN_F8R2_FB18_Pos) /*!< 0x00040000 */
#define CAN_F8R2_FB18 CAN_F8R2_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F8R2_FB19_Pos (19U)
#define CAN_F8R2_FB19_Msk (0x1UL << CAN_F8R2_FB19_Pos) /*!< 0x00080000 */
#define CAN_F8R2_FB19 CAN_F8R2_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F8R2_FB20_Pos (20U)
#define CAN_F8R2_FB20_Msk (0x1UL << CAN_F8R2_FB20_Pos) /*!< 0x00100000 */
#define CAN_F8R2_FB20 CAN_F8R2_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F8R2_FB21_Pos (21U)
#define CAN_F8R2_FB21_Msk (0x1UL << CAN_F8R2_FB21_Pos) /*!< 0x00200000 */
#define CAN_F8R2_FB21 CAN_F8R2_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F8R2_FB22_Pos (22U)
#define CAN_F8R2_FB22_Msk (0x1UL << CAN_F8R2_FB22_Pos) /*!< 0x00400000 */
#define CAN_F8R2_FB22 CAN_F8R2_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F8R2_FB23_Pos (23U)
#define CAN_F8R2_FB23_Msk (0x1UL << CAN_F8R2_FB23_Pos) /*!< 0x00800000 */
#define CAN_F8R2_FB23 CAN_F8R2_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F8R2_FB24_Pos (24U)
#define CAN_F8R2_FB24_Msk (0x1UL << CAN_F8R2_FB24_Pos) /*!< 0x01000000 */
#define CAN_F8R2_FB24 CAN_F8R2_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F8R2_FB25_Pos (25U)
#define CAN_F8R2_FB25_Msk (0x1UL << CAN_F8R2_FB25_Pos) /*!< 0x02000000 */
#define CAN_F8R2_FB25 CAN_F8R2_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F8R2_FB26_Pos (26U)
#define CAN_F8R2_FB26_Msk (0x1UL << CAN_F8R2_FB26_Pos) /*!< 0x04000000 */
#define CAN_F8R2_FB26 CAN_F8R2_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F8R2_FB27_Pos (27U)
#define CAN_F8R2_FB27_Msk (0x1UL << CAN_F8R2_FB27_Pos) /*!< 0x08000000 */
#define CAN_F8R2_FB27 CAN_F8R2_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F8R2_FB28_Pos (28U)
#define CAN_F8R2_FB28_Msk (0x1UL << CAN_F8R2_FB28_Pos) /*!< 0x10000000 */
#define CAN_F8R2_FB28 CAN_F8R2_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F8R2_FB29_Pos (29U)
#define CAN_F8R2_FB29_Msk (0x1UL << CAN_F8R2_FB29_Pos) /*!< 0x20000000 */
#define CAN_F8R2_FB29 CAN_F8R2_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F8R2_FB30_Pos (30U)
#define CAN_F8R2_FB30_Msk (0x1UL << CAN_F8R2_FB30_Pos) /*!< 0x40000000 */
#define CAN_F8R2_FB30 CAN_F8R2_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F8R2_FB31_Pos (31U)
#define CAN_F8R2_FB31_Msk (0x1UL << CAN_F8R2_FB31_Pos) /*!< 0x80000000 */
#define CAN_F8R2_FB31 CAN_F8R2_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F9R2 register  *******************/
#define CAN_F9R2_FB0_Pos (0U)
#define CAN_F9R2_FB0_Msk (0x1UL << CAN_F9R2_FB0_Pos) /*!< 0x00000001 */
#define CAN_F9R2_FB0 CAN_F9R2_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F9R2_FB1_Pos (1U)
#define CAN_F9R2_FB1_Msk (0x1UL << CAN_F9R2_FB1_Pos) /*!< 0x00000002 */
#define CAN_F9R2_FB1 CAN_F9R2_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F9R2_FB2_Pos (2U)
#define CAN_F9R2_FB2_Msk (0x1UL << CAN_F9R2_FB2_Pos) /*!< 0x00000004 */
#define CAN_F9R2_FB2 CAN_F9R2_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F9R2_FB3_Pos (3U)
#define CAN_F9R2_FB3_Msk (0x1UL << CAN_F9R2_FB3_Pos) /*!< 0x00000008 */
#define CAN_F9R2_FB3 CAN_F9R2_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F9R2_FB4_Pos (4U)
#define CAN_F9R2_FB4_Msk (0x1UL << CAN_F9R2_FB4_Pos) /*!< 0x00000010 */
#define CAN_F9R2_FB4 CAN_F9R2_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F9R2_FB5_Pos (5U)
#define CAN_F9R2_FB5_Msk (0x1UL << CAN_F9R2_FB5_Pos) /*!< 0x00000020 */
#define CAN_F9R2_FB5 CAN_F9R2_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F9R2_FB6_Pos (6U)
#define CAN_F9R2_FB6_Msk (0x1UL << CAN_F9R2_FB6_Pos) /*!< 0x00000040 */
#define CAN_F9R2_FB6 CAN_F9R2_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F9R2_FB7_Pos (7U)
#define CAN_F9R2_FB7_Msk (0x1UL << CAN_F9R2_FB7_Pos) /*!< 0x00000080 */
#define CAN_F9R2_FB7 CAN_F9R2_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F9R2_FB8_Pos (8U)
#define CAN_F9R2_FB8_Msk (0x1UL << CAN_F9R2_FB8_Pos) /*!< 0x00000100 */
#define CAN_F9R2_FB8 CAN_F9R2_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F9R2_FB9_Pos (9U)
#define CAN_F9R2_FB9_Msk (0x1UL << CAN_F9R2_FB9_Pos) /*!< 0x00000200 */
#define CAN_F9R2_FB9 CAN_F9R2_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F9R2_FB10_Pos (10U)
#define CAN_F9R2_FB10_Msk (0x1UL << CAN_F9R2_FB10_Pos) /*!< 0x00000400 */
#define CAN_F9R2_FB10 CAN_F9R2_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F9R2_FB11_Pos (11U)
#define CAN_F9R2_FB11_Msk (0x1UL << CAN_F9R2_FB11_Pos) /*!< 0x00000800 */
#define CAN_F9R2_FB11 CAN_F9R2_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F9R2_FB12_Pos (12U)
#define CAN_F9R2_FB12_Msk (0x1UL << CAN_F9R2_FB12_Pos) /*!< 0x00001000 */
#define CAN_F9R2_FB12 CAN_F9R2_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F9R2_FB13_Pos (13U)
#define CAN_F9R2_FB13_Msk (0x1UL << CAN_F9R2_FB13_Pos) /*!< 0x00002000 */
#define CAN_F9R2_FB13 CAN_F9R2_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F9R2_FB14_Pos (14U)
#define CAN_F9R2_FB14_Msk (0x1UL << CAN_F9R2_FB14_Pos) /*!< 0x00004000 */
#define CAN_F9R2_FB14 CAN_F9R2_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F9R2_FB15_Pos (15U)
#define CAN_F9R2_FB15_Msk (0x1UL << CAN_F9R2_FB15_Pos) /*!< 0x00008000 */
#define CAN_F9R2_FB15 CAN_F9R2_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F9R2_FB16_Pos (16U)
#define CAN_F9R2_FB16_Msk (0x1UL << CAN_F9R2_FB16_Pos) /*!< 0x00010000 */
#define CAN_F9R2_FB16 CAN_F9R2_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F9R2_FB17_Pos (17U)
#define CAN_F9R2_FB17_Msk (0x1UL << CAN_F9R2_FB17_Pos) /*!< 0x00020000 */
#define CAN_F9R2_FB17 CAN_F9R2_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F9R2_FB18_Pos (18U)
#define CAN_F9R2_FB18_Msk (0x1UL << CAN_F9R2_FB18_Pos) /*!< 0x00040000 */
#define CAN_F9R2_FB18 CAN_F9R2_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F9R2_FB19_Pos (19U)
#define CAN_F9R2_FB19_Msk (0x1UL << CAN_F9R2_FB19_Pos) /*!< 0x00080000 */
#define CAN_F9R2_FB19 CAN_F9R2_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F9R2_FB20_Pos (20U)
#define CAN_F9R2_FB20_Msk (0x1UL << CAN_F9R2_FB20_Pos) /*!< 0x00100000 */
#define CAN_F9R2_FB20 CAN_F9R2_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F9R2_FB21_Pos (21U)
#define CAN_F9R2_FB21_Msk (0x1UL << CAN_F9R2_FB21_Pos) /*!< 0x00200000 */
#define CAN_F9R2_FB21 CAN_F9R2_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F9R2_FB22_Pos (22U)
#define CAN_F9R2_FB22_Msk (0x1UL << CAN_F9R2_FB22_Pos) /*!< 0x00400000 */
#define CAN_F9R2_FB22 CAN_F9R2_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F9R2_FB23_Pos (23U)
#define CAN_F9R2_FB23_Msk (0x1UL << CAN_F9R2_FB23_Pos) /*!< 0x00800000 */
#define CAN_F9R2_FB23 CAN_F9R2_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F9R2_FB24_Pos (24U)
#define CAN_F9R2_FB24_Msk (0x1UL << CAN_F9R2_FB24_Pos) /*!< 0x01000000 */
#define CAN_F9R2_FB24 CAN_F9R2_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F9R2_FB25_Pos (25U)
#define CAN_F9R2_FB25_Msk (0x1UL << CAN_F9R2_FB25_Pos) /*!< 0x02000000 */
#define CAN_F9R2_FB25 CAN_F9R2_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F9R2_FB26_Pos (26U)
#define CAN_F9R2_FB26_Msk (0x1UL << CAN_F9R2_FB26_Pos) /*!< 0x04000000 */
#define CAN_F9R2_FB26 CAN_F9R2_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F9R2_FB27_Pos (27U)
#define CAN_F9R2_FB27_Msk (0x1UL << CAN_F9R2_FB27_Pos) /*!< 0x08000000 */
#define CAN_F9R2_FB27 CAN_F9R2_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F9R2_FB28_Pos (28U)
#define CAN_F9R2_FB28_Msk (0x1UL << CAN_F9R2_FB28_Pos) /*!< 0x10000000 */
#define CAN_F9R2_FB28 CAN_F9R2_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F9R2_FB29_Pos (29U)
#define CAN_F9R2_FB29_Msk (0x1UL << CAN_F9R2_FB29_Pos) /*!< 0x20000000 */
#define CAN_F9R2_FB29 CAN_F9R2_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F9R2_FB30_Pos (30U)
#define CAN_F9R2_FB30_Msk (0x1UL << CAN_F9R2_FB30_Pos) /*!< 0x40000000 */
#define CAN_F9R2_FB30 CAN_F9R2_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F9R2_FB31_Pos (31U)
#define CAN_F9R2_FB31_Msk (0x1UL << CAN_F9R2_FB31_Pos) /*!< 0x80000000 */
#define CAN_F9R2_FB31 CAN_F9R2_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F10R2 register  ******************/
#define CAN_F10R2_FB0_Pos (0U)
#define CAN_F10R2_FB0_Msk (0x1UL << CAN_F10R2_FB0_Pos) /*!< 0x00000001 */
#define CAN_F10R2_FB0 CAN_F10R2_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F10R2_FB1_Pos (1U)
#define CAN_F10R2_FB1_Msk (0x1UL << CAN_F10R2_FB1_Pos) /*!< 0x00000002 */
#define CAN_F10R2_FB1 CAN_F10R2_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F10R2_FB2_Pos (2U)
#define CAN_F10R2_FB2_Msk (0x1UL << CAN_F10R2_FB2_Pos) /*!< 0x00000004 */
#define CAN_F10R2_FB2 CAN_F10R2_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F10R2_FB3_Pos (3U)
#define CAN_F10R2_FB3_Msk (0x1UL << CAN_F10R2_FB3_Pos) /*!< 0x00000008 */
#define CAN_F10R2_FB3 CAN_F10R2_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F10R2_FB4_Pos (4U)
#define CAN_F10R2_FB4_Msk (0x1UL << CAN_F10R2_FB4_Pos) /*!< 0x00000010 */
#define CAN_F10R2_FB4 CAN_F10R2_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F10R2_FB5_Pos (5U)
#define CAN_F10R2_FB5_Msk (0x1UL << CAN_F10R2_FB5_Pos) /*!< 0x00000020 */
#define CAN_F10R2_FB5 CAN_F10R2_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F10R2_FB6_Pos (6U)
#define CAN_F10R2_FB6_Msk (0x1UL << CAN_F10R2_FB6_Pos) /*!< 0x00000040 */
#define CAN_F10R2_FB6 CAN_F10R2_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F10R2_FB7_Pos (7U)
#define CAN_F10R2_FB7_Msk (0x1UL << CAN_F10R2_FB7_Pos) /*!< 0x00000080 */
#define CAN_F10R2_FB7 CAN_F10R2_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F10R2_FB8_Pos (8U)
#define CAN_F10R2_FB8_Msk (0x1UL << CAN_F10R2_FB8_Pos) /*!< 0x00000100 */
#define CAN_F10R2_FB8 CAN_F10R2_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F10R2_FB9_Pos (9U)
#define CAN_F10R2_FB9_Msk (0x1UL << CAN_F10R2_FB9_Pos) /*!< 0x00000200 */
#define CAN_F10R2_FB9 CAN_F10R2_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F10R2_FB10_Pos (10U)
#define CAN_F10R2_FB10_Msk (0x1UL << CAN_F10R2_FB10_Pos) /*!< 0x00000400 */
#define CAN_F10R2_FB10 CAN_F10R2_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F10R2_FB11_Pos (11U)
#define CAN_F10R2_FB11_Msk (0x1UL << CAN_F10R2_FB11_Pos) /*!< 0x00000800 */
#define CAN_F10R2_FB11 CAN_F10R2_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F10R2_FB12_Pos (12U)
#define CAN_F10R2_FB12_Msk (0x1UL << CAN_F10R2_FB12_Pos) /*!< 0x00001000 */
#define CAN_F10R2_FB12 CAN_F10R2_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F10R2_FB13_Pos (13U)
#define CAN_F10R2_FB13_Msk (0x1UL << CAN_F10R2_FB13_Pos) /*!< 0x00002000 */
#define CAN_F10R2_FB13 CAN_F10R2_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F10R2_FB14_Pos (14U)
#define CAN_F10R2_FB14_Msk (0x1UL << CAN_F10R2_FB14_Pos) /*!< 0x00004000 */
#define CAN_F10R2_FB14 CAN_F10R2_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F10R2_FB15_Pos (15U)
#define CAN_F10R2_FB15_Msk (0x1UL << CAN_F10R2_FB15_Pos) /*!< 0x00008000 */
#define CAN_F10R2_FB15 CAN_F10R2_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F10R2_FB16_Pos (16U)
#define CAN_F10R2_FB16_Msk (0x1UL << CAN_F10R2_FB16_Pos) /*!< 0x00010000 */
#define CAN_F10R2_FB16 CAN_F10R2_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F10R2_FB17_Pos (17U)
#define CAN_F10R2_FB17_Msk (0x1UL << CAN_F10R2_FB17_Pos) /*!< 0x00020000 */
#define CAN_F10R2_FB17 CAN_F10R2_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F10R2_FB18_Pos (18U)
#define CAN_F10R2_FB18_Msk (0x1UL << CAN_F10R2_FB18_Pos) /*!< 0x00040000 */
#define CAN_F10R2_FB18 CAN_F10R2_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F10R2_FB19_Pos (19U)
#define CAN_F10R2_FB19_Msk (0x1UL << CAN_F10R2_FB19_Pos) /*!< 0x00080000 */
#define CAN_F10R2_FB19 CAN_F10R2_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F10R2_FB20_Pos (20U)
#define CAN_F10R2_FB20_Msk (0x1UL << CAN_F10R2_FB20_Pos) /*!< 0x00100000 */
#define CAN_F10R2_FB20 CAN_F10R2_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F10R2_FB21_Pos (21U)
#define CAN_F10R2_FB21_Msk (0x1UL << CAN_F10R2_FB21_Pos) /*!< 0x00200000 */
#define CAN_F10R2_FB21 CAN_F10R2_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F10R2_FB22_Pos (22U)
#define CAN_F10R2_FB22_Msk (0x1UL << CAN_F10R2_FB22_Pos) /*!< 0x00400000 */
#define CAN_F10R2_FB22 CAN_F10R2_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F10R2_FB23_Pos (23U)
#define CAN_F10R2_FB23_Msk (0x1UL << CAN_F10R2_FB23_Pos) /*!< 0x00800000 */
#define CAN_F10R2_FB23 CAN_F10R2_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F10R2_FB24_Pos (24U)
#define CAN_F10R2_FB24_Msk (0x1UL << CAN_F10R2_FB24_Pos) /*!< 0x01000000 */
#define CAN_F10R2_FB24 CAN_F10R2_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F10R2_FB25_Pos (25U)
#define CAN_F10R2_FB25_Msk (0x1UL << CAN_F10R2_FB25_Pos) /*!< 0x02000000 */
#define CAN_F10R2_FB25 CAN_F10R2_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F10R2_FB26_Pos (26U)
#define CAN_F10R2_FB26_Msk (0x1UL << CAN_F10R2_FB26_Pos) /*!< 0x04000000 */
#define CAN_F10R2_FB26 CAN_F10R2_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F10R2_FB27_Pos (27U)
#define CAN_F10R2_FB27_Msk (0x1UL << CAN_F10R2_FB27_Pos) /*!< 0x08000000 */
#define CAN_F10R2_FB27 CAN_F10R2_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F10R2_FB28_Pos (28U)
#define CAN_F10R2_FB28_Msk (0x1UL << CAN_F10R2_FB28_Pos) /*!< 0x10000000 */
#define CAN_F10R2_FB28 CAN_F10R2_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F10R2_FB29_Pos (29U)
#define CAN_F10R2_FB29_Msk (0x1UL << CAN_F10R2_FB29_Pos) /*!< 0x20000000 */
#define CAN_F10R2_FB29 CAN_F10R2_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F10R2_FB30_Pos (30U)
#define CAN_F10R2_FB30_Msk (0x1UL << CAN_F10R2_FB30_Pos) /*!< 0x40000000 */
#define CAN_F10R2_FB30 CAN_F10R2_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F10R2_FB31_Pos (31U)
#define CAN_F10R2_FB31_Msk (0x1UL << CAN_F10R2_FB31_Pos) /*!< 0x80000000 */
#define CAN_F10R2_FB31 CAN_F10R2_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F11R2 register  ******************/
#define CAN_F11R2_FB0_Pos (0U)
#define CAN_F11R2_FB0_Msk (0x1UL << CAN_F11R2_FB0_Pos) /*!< 0x00000001 */
#define CAN_F11R2_FB0 CAN_F11R2_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F11R2_FB1_Pos (1U)
#define CAN_F11R2_FB1_Msk (0x1UL << CAN_F11R2_FB1_Pos) /*!< 0x00000002 */
#define CAN_F11R2_FB1 CAN_F11R2_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F11R2_FB2_Pos (2U)
#define CAN_F11R2_FB2_Msk (0x1UL << CAN_F11R2_FB2_Pos) /*!< 0x00000004 */
#define CAN_F11R2_FB2 CAN_F11R2_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F11R2_FB3_Pos (3U)
#define CAN_F11R2_FB3_Msk (0x1UL << CAN_F11R2_FB3_Pos) /*!< 0x00000008 */
#define CAN_F11R2_FB3 CAN_F11R2_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F11R2_FB4_Pos (4U)
#define CAN_F11R2_FB4_Msk (0x1UL << CAN_F11R2_FB4_Pos) /*!< 0x00000010 */
#define CAN_F11R2_FB4 CAN_F11R2_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F11R2_FB5_Pos (5U)
#define CAN_F11R2_FB5_Msk (0x1UL << CAN_F11R2_FB5_Pos) /*!< 0x00000020 */
#define CAN_F11R2_FB5 CAN_F11R2_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F11R2_FB6_Pos (6U)
#define CAN_F11R2_FB6_Msk (0x1UL << CAN_F11R2_FB6_Pos) /*!< 0x00000040 */
#define CAN_F11R2_FB6 CAN_F11R2_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F11R2_FB7_Pos (7U)
#define CAN_F11R2_FB7_Msk (0x1UL << CAN_F11R2_FB7_Pos) /*!< 0x00000080 */
#define CAN_F11R2_FB7 CAN_F11R2_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F11R2_FB8_Pos (8U)
#define CAN_F11R2_FB8_Msk (0x1UL << CAN_F11R2_FB8_Pos) /*!< 0x00000100 */
#define CAN_F11R2_FB8 CAN_F11R2_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F11R2_FB9_Pos (9U)
#define CAN_F11R2_FB9_Msk (0x1UL << CAN_F11R2_FB9_Pos) /*!< 0x00000200 */
#define CAN_F11R2_FB9 CAN_F11R2_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F11R2_FB10_Pos (10U)
#define CAN_F11R2_FB10_Msk (0x1UL << CAN_F11R2_FB10_Pos) /*!< 0x00000400 */
#define CAN_F11R2_FB10 CAN_F11R2_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F11R2_FB11_Pos (11U)
#define CAN_F11R2_FB11_Msk (0x1UL << CAN_F11R2_FB11_Pos) /*!< 0x00000800 */
#define CAN_F11R2_FB11 CAN_F11R2_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F11R2_FB12_Pos (12U)
#define CAN_F11R2_FB12_Msk (0x1UL << CAN_F11R2_FB12_Pos) /*!< 0x00001000 */
#define CAN_F11R2_FB12 CAN_F11R2_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F11R2_FB13_Pos (13U)
#define CAN_F11R2_FB13_Msk (0x1UL << CAN_F11R2_FB13_Pos) /*!< 0x00002000 */
#define CAN_F11R2_FB13 CAN_F11R2_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F11R2_FB14_Pos (14U)
#define CAN_F11R2_FB14_Msk (0x1UL << CAN_F11R2_FB14_Pos) /*!< 0x00004000 */
#define CAN_F11R2_FB14 CAN_F11R2_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F11R2_FB15_Pos (15U)
#define CAN_F11R2_FB15_Msk (0x1UL << CAN_F11R2_FB15_Pos) /*!< 0x00008000 */
#define CAN_F11R2_FB15 CAN_F11R2_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F11R2_FB16_Pos (16U)
#define CAN_F11R2_FB16_Msk (0x1UL << CAN_F11R2_FB16_Pos) /*!< 0x00010000 */
#define CAN_F11R2_FB16 CAN_F11R2_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F11R2_FB17_Pos (17U)
#define CAN_F11R2_FB17_Msk (0x1UL << CAN_F11R2_FB17_Pos) /*!< 0x00020000 */
#define CAN_F11R2_FB17 CAN_F11R2_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F11R2_FB18_Pos (18U)
#define CAN_F11R2_FB18_Msk (0x1UL << CAN_F11R2_FB18_Pos) /*!< 0x00040000 */
#define CAN_F11R2_FB18 CAN_F11R2_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F11R2_FB19_Pos (19U)
#define CAN_F11R2_FB19_Msk (0x1UL << CAN_F11R2_FB19_Pos) /*!< 0x00080000 */
#define CAN_F11R2_FB19 CAN_F11R2_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F11R2_FB20_Pos (20U)
#define CAN_F11R2_FB20_Msk (0x1UL << CAN_F11R2_FB20_Pos) /*!< 0x00100000 */
#define CAN_F11R2_FB20 CAN_F11R2_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F11R2_FB21_Pos (21U)
#define CAN_F11R2_FB21_Msk (0x1UL << CAN_F11R2_FB21_Pos) /*!< 0x00200000 */
#define CAN_F11R2_FB21 CAN_F11R2_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F11R2_FB22_Pos (22U)
#define CAN_F11R2_FB22_Msk (0x1UL << CAN_F11R2_FB22_Pos) /*!< 0x00400000 */
#define CAN_F11R2_FB22 CAN_F11R2_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F11R2_FB23_Pos (23U)
#define CAN_F11R2_FB23_Msk (0x1UL << CAN_F11R2_FB23_Pos) /*!< 0x00800000 */
#define CAN_F11R2_FB23 CAN_F11R2_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F11R2_FB24_Pos (24U)
#define CAN_F11R2_FB24_Msk (0x1UL << CAN_F11R2_FB24_Pos) /*!< 0x01000000 */
#define CAN_F11R2_FB24 CAN_F11R2_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F11R2_FB25_Pos (25U)
#define CAN_F11R2_FB25_Msk (0x1UL << CAN_F11R2_FB25_Pos) /*!< 0x02000000 */
#define CAN_F11R2_FB25 CAN_F11R2_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F11R2_FB26_Pos (26U)
#define CAN_F11R2_FB26_Msk (0x1UL << CAN_F11R2_FB26_Pos) /*!< 0x04000000 */
#define CAN_F11R2_FB26 CAN_F11R2_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F11R2_FB27_Pos (27U)
#define CAN_F11R2_FB27_Msk (0x1UL << CAN_F11R2_FB27_Pos) /*!< 0x08000000 */
#define CAN_F11R2_FB27 CAN_F11R2_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F11R2_FB28_Pos (28U)
#define CAN_F11R2_FB28_Msk (0x1UL << CAN_F11R2_FB28_Pos) /*!< 0x10000000 */
#define CAN_F11R2_FB28 CAN_F11R2_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F11R2_FB29_Pos (29U)
#define CAN_F11R2_FB29_Msk (0x1UL << CAN_F11R2_FB29_Pos) /*!< 0x20000000 */
#define CAN_F11R2_FB29 CAN_F11R2_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F11R2_FB30_Pos (30U)
#define CAN_F11R2_FB30_Msk (0x1UL << CAN_F11R2_FB30_Pos) /*!< 0x40000000 */
#define CAN_F11R2_FB30 CAN_F11R2_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F11R2_FB31_Pos (31U)
#define CAN_F11R2_FB31_Msk (0x1UL << CAN_F11R2_FB31_Pos) /*!< 0x80000000 */
#define CAN_F11R2_FB31 CAN_F11R2_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F12R2 register  ******************/
#define CAN_F12R2_FB0_Pos (0U)
#define CAN_F12R2_FB0_Msk (0x1UL << CAN_F12R2_FB0_Pos) /*!< 0x00000001 */
#define CAN_F12R2_FB0 CAN_F12R2_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F12R2_FB1_Pos (1U)
#define CAN_F12R2_FB1_Msk (0x1UL << CAN_F12R2_FB1_Pos) /*!< 0x00000002 */
#define CAN_F12R2_FB1 CAN_F12R2_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F12R2_FB2_Pos (2U)
#define CAN_F12R2_FB2_Msk (0x1UL << CAN_F12R2_FB2_Pos) /*!< 0x00000004 */
#define CAN_F12R2_FB2 CAN_F12R2_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F12R2_FB3_Pos (3U)
#define CAN_F12R2_FB3_Msk (0x1UL << CAN_F12R2_FB3_Pos) /*!< 0x00000008 */
#define CAN_F12R2_FB3 CAN_F12R2_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F12R2_FB4_Pos (4U)
#define CAN_F12R2_FB4_Msk (0x1UL << CAN_F12R2_FB4_Pos) /*!< 0x00000010 */
#define CAN_F12R2_FB4 CAN_F12R2_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F12R2_FB5_Pos (5U)
#define CAN_F12R2_FB5_Msk (0x1UL << CAN_F12R2_FB5_Pos) /*!< 0x00000020 */
#define CAN_F12R2_FB5 CAN_F12R2_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F12R2_FB6_Pos (6U)
#define CAN_F12R2_FB6_Msk (0x1UL << CAN_F12R2_FB6_Pos) /*!< 0x00000040 */
#define CAN_F12R2_FB6 CAN_F12R2_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F12R2_FB7_Pos (7U)
#define CAN_F12R2_FB7_Msk (0x1UL << CAN_F12R2_FB7_Pos) /*!< 0x00000080 */
#define CAN_F12R2_FB7 CAN_F12R2_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F12R2_FB8_Pos (8U)
#define CAN_F12R2_FB8_Msk (0x1UL << CAN_F12R2_FB8_Pos) /*!< 0x00000100 */
#define CAN_F12R2_FB8 CAN_F12R2_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F12R2_FB9_Pos (9U)
#define CAN_F12R2_FB9_Msk (0x1UL << CAN_F12R2_FB9_Pos) /*!< 0x00000200 */
#define CAN_F12R2_FB9 CAN_F12R2_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F12R2_FB10_Pos (10U)
#define CAN_F12R2_FB10_Msk (0x1UL << CAN_F12R2_FB10_Pos) /*!< 0x00000400 */
#define CAN_F12R2_FB10 CAN_F12R2_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F12R2_FB11_Pos (11U)
#define CAN_F12R2_FB11_Msk (0x1UL << CAN_F12R2_FB11_Pos) /*!< 0x00000800 */
#define CAN_F12R2_FB11 CAN_F12R2_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F12R2_FB12_Pos (12U)
#define CAN_F12R2_FB12_Msk (0x1UL << CAN_F12R2_FB12_Pos) /*!< 0x00001000 */
#define CAN_F12R2_FB12 CAN_F12R2_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F12R2_FB13_Pos (13U)
#define CAN_F12R2_FB13_Msk (0x1UL << CAN_F12R2_FB13_Pos) /*!< 0x00002000 */
#define CAN_F12R2_FB13 CAN_F12R2_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F12R2_FB14_Pos (14U)
#define CAN_F12R2_FB14_Msk (0x1UL << CAN_F12R2_FB14_Pos) /*!< 0x00004000 */
#define CAN_F12R2_FB14 CAN_F12R2_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F12R2_FB15_Pos (15U)
#define CAN_F12R2_FB15_Msk (0x1UL << CAN_F12R2_FB15_Pos) /*!< 0x00008000 */
#define CAN_F12R2_FB15 CAN_F12R2_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F12R2_FB16_Pos (16U)
#define CAN_F12R2_FB16_Msk (0x1UL << CAN_F12R2_FB16_Pos) /*!< 0x00010000 */
#define CAN_F12R2_FB16 CAN_F12R2_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F12R2_FB17_Pos (17U)
#define CAN_F12R2_FB17_Msk (0x1UL << CAN_F12R2_FB17_Pos) /*!< 0x00020000 */
#define CAN_F12R2_FB17 CAN_F12R2_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F12R2_FB18_Pos (18U)
#define CAN_F12R2_FB18_Msk (0x1UL << CAN_F12R2_FB18_Pos) /*!< 0x00040000 */
#define CAN_F12R2_FB18 CAN_F12R2_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F12R2_FB19_Pos (19U)
#define CAN_F12R2_FB19_Msk (0x1UL << CAN_F12R2_FB19_Pos) /*!< 0x00080000 */
#define CAN_F12R2_FB19 CAN_F12R2_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F12R2_FB20_Pos (20U)
#define CAN_F12R2_FB20_Msk (0x1UL << CAN_F12R2_FB20_Pos) /*!< 0x00100000 */
#define CAN_F12R2_FB20 CAN_F12R2_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F12R2_FB21_Pos (21U)
#define CAN_F12R2_FB21_Msk (0x1UL << CAN_F12R2_FB21_Pos) /*!< 0x00200000 */
#define CAN_F12R2_FB21 CAN_F12R2_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F12R2_FB22_Pos (22U)
#define CAN_F12R2_FB22_Msk (0x1UL << CAN_F12R2_FB22_Pos) /*!< 0x00400000 */
#define CAN_F12R2_FB22 CAN_F12R2_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F12R2_FB23_Pos (23U)
#define CAN_F12R2_FB23_Msk (0x1UL << CAN_F12R2_FB23_Pos) /*!< 0x00800000 */
#define CAN_F12R2_FB23 CAN_F12R2_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F12R2_FB24_Pos (24U)
#define CAN_F12R2_FB24_Msk (0x1UL << CAN_F12R2_FB24_Pos) /*!< 0x01000000 */
#define CAN_F12R2_FB24 CAN_F12R2_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F12R2_FB25_Pos (25U)
#define CAN_F12R2_FB25_Msk (0x1UL << CAN_F12R2_FB25_Pos) /*!< 0x02000000 */
#define CAN_F12R2_FB25 CAN_F12R2_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F12R2_FB26_Pos (26U)
#define CAN_F12R2_FB26_Msk (0x1UL << CAN_F12R2_FB26_Pos) /*!< 0x04000000 */
#define CAN_F12R2_FB26 CAN_F12R2_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F12R2_FB27_Pos (27U)
#define CAN_F12R2_FB27_Msk (0x1UL << CAN_F12R2_FB27_Pos) /*!< 0x08000000 */
#define CAN_F12R2_FB27 CAN_F12R2_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F12R2_FB28_Pos (28U)
#define CAN_F12R2_FB28_Msk (0x1UL << CAN_F12R2_FB28_Pos) /*!< 0x10000000 */
#define CAN_F12R2_FB28 CAN_F12R2_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F12R2_FB29_Pos (29U)
#define CAN_F12R2_FB29_Msk (0x1UL << CAN_F12R2_FB29_Pos) /*!< 0x20000000 */
#define CAN_F12R2_FB29 CAN_F12R2_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F12R2_FB30_Pos (30U)
#define CAN_F12R2_FB30_Msk (0x1UL << CAN_F12R2_FB30_Pos) /*!< 0x40000000 */
#define CAN_F12R2_FB30 CAN_F12R2_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F12R2_FB31_Pos (31U)
#define CAN_F12R2_FB31_Msk (0x1UL << CAN_F12R2_FB31_Pos) /*!< 0x80000000 */
#define CAN_F12R2_FB31 CAN_F12R2_FB31_Msk                /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F13R2 register  ******************/
#define CAN_F13R2_FB0_Pos (0U)
#define CAN_F13R2_FB0_Msk (0x1UL << CAN_F13R2_FB0_Pos) /*!< 0x00000001 */
#define CAN_F13R2_FB0 CAN_F13R2_FB0_Msk                /*!<Filter bit 0 */
#define CAN_F13R2_FB1_Pos (1U)
#define CAN_F13R2_FB1_Msk (0x1UL << CAN_F13R2_FB1_Pos) /*!< 0x00000002 */
#define CAN_F13R2_FB1 CAN_F13R2_FB1_Msk                /*!<Filter bit 1 */
#define CAN_F13R2_FB2_Pos (2U)
#define CAN_F13R2_FB2_Msk (0x1UL << CAN_F13R2_FB2_Pos) /*!< 0x00000004 */
#define CAN_F13R2_FB2 CAN_F13R2_FB2_Msk                /*!<Filter bit 2 */
#define CAN_F13R2_FB3_Pos (3U)
#define CAN_F13R2_FB3_Msk (0x1UL << CAN_F13R2_FB3_Pos) /*!< 0x00000008 */
#define CAN_F13R2_FB3 CAN_F13R2_FB3_Msk                /*!<Filter bit 3 */
#define CAN_F13R2_FB4_Pos (4U)
#define CAN_F13R2_FB4_Msk (0x1UL << CAN_F13R2_FB4_Pos) /*!< 0x00000010 */
#define CAN_F13R2_FB4 CAN_F13R2_FB4_Msk                /*!<Filter bit 4 */
#define CAN_F13R2_FB5_Pos (5U)
#define CAN_F13R2_FB5_Msk (0x1UL << CAN_F13R2_FB5_Pos) /*!< 0x00000020 */
#define CAN_F13R2_FB5 CAN_F13R2_FB5_Msk                /*!<Filter bit 5 */
#define CAN_F13R2_FB6_Pos (6U)
#define CAN_F13R2_FB6_Msk (0x1UL << CAN_F13R2_FB6_Pos) /*!< 0x00000040 */
#define CAN_F13R2_FB6 CAN_F13R2_FB6_Msk                /*!<Filter bit 6 */
#define CAN_F13R2_FB7_Pos (7U)
#define CAN_F13R2_FB7_Msk (0x1UL << CAN_F13R2_FB7_Pos) /*!< 0x00000080 */
#define CAN_F13R2_FB7 CAN_F13R2_FB7_Msk                /*!<Filter bit 7 */
#define CAN_F13R2_FB8_Pos (8U)
#define CAN_F13R2_FB8_Msk (0x1UL << CAN_F13R2_FB8_Pos) /*!< 0x00000100 */
#define CAN_F13R2_FB8 CAN_F13R2_FB8_Msk                /*!<Filter bit 8 */
#define CAN_F13R2_FB9_Pos (9U)
#define CAN_F13R2_FB9_Msk (0x1UL << CAN_F13R2_FB9_Pos) /*!< 0x00000200 */
#define CAN_F13R2_FB9 CAN_F13R2_FB9_Msk                /*!<Filter bit 9 */
#define CAN_F13R2_FB10_Pos (10U)
#define CAN_F13R2_FB10_Msk (0x1UL << CAN_F13R2_FB10_Pos) /*!< 0x00000400 */
#define CAN_F13R2_FB10 CAN_F13R2_FB10_Msk                /*!<Filter bit 10 */
#define CAN_F13R2_FB11_Pos (11U)
#define CAN_F13R2_FB11_Msk (0x1UL << CAN_F13R2_FB11_Pos) /*!< 0x00000800 */
#define CAN_F13R2_FB11 CAN_F13R2_FB11_Msk                /*!<Filter bit 11 */
#define CAN_F13R2_FB12_Pos (12U)
#define CAN_F13R2_FB12_Msk (0x1UL << CAN_F13R2_FB12_Pos) /*!< 0x00001000 */
#define CAN_F13R2_FB12 CAN_F13R2_FB12_Msk                /*!<Filter bit 12 */
#define CAN_F13R2_FB13_Pos (13U)
#define CAN_F13R2_FB13_Msk (0x1UL << CAN_F13R2_FB13_Pos) /*!< 0x00002000 */
#define CAN_F13R2_FB13 CAN_F13R2_FB13_Msk                /*!<Filter bit 13 */
#define CAN_F13R2_FB14_Pos (14U)
#define CAN_F13R2_FB14_Msk (0x1UL << CAN_F13R2_FB14_Pos) /*!< 0x00004000 */
#define CAN_F13R2_FB14 CAN_F13R2_FB14_Msk                /*!<Filter bit 14 */
#define CAN_F13R2_FB15_Pos (15U)
#define CAN_F13R2_FB15_Msk (0x1UL << CAN_F13R2_FB15_Pos) /*!< 0x00008000 */
#define CAN_F13R2_FB15 CAN_F13R2_FB15_Msk                /*!<Filter bit 15 */
#define CAN_F13R2_FB16_Pos (16U)
#define CAN_F13R2_FB16_Msk (0x1UL << CAN_F13R2_FB16_Pos) /*!< 0x00010000 */
#define CAN_F13R2_FB16 CAN_F13R2_FB16_Msk                /*!<Filter bit 16 */
#define CAN_F13R2_FB17_Pos (17U)
#define CAN_F13R2_FB17_Msk (0x1UL << CAN_F13R2_FB17_Pos) /*!< 0x00020000 */
#define CAN_F13R2_FB17 CAN_F13R2_FB17_Msk                /*!<Filter bit 17 */
#define CAN_F13R2_FB18_Pos (18U)
#define CAN_F13R2_FB18_Msk (0x1UL << CAN_F13R2_FB18_Pos) /*!< 0x00040000 */
#define CAN_F13R2_FB18 CAN_F13R2_FB18_Msk                /*!<Filter bit 18 */
#define CAN_F13R2_FB19_Pos (19U)
#define CAN_F13R2_FB19_Msk (0x1UL << CAN_F13R2_FB19_Pos) /*!< 0x00080000 */
#define CAN_F13R2_FB19 CAN_F13R2_FB19_Msk                /*!<Filter bit 19 */
#define CAN_F13R2_FB20_Pos (20U)
#define CAN_F13R2_FB20_Msk (0x1UL << CAN_F13R2_FB20_Pos) /*!< 0x00100000 */
#define CAN_F13R2_FB20 CAN_F13R2_FB20_Msk                /*!<Filter bit 20 */
#define CAN_F13R2_FB21_Pos (21U)
#define CAN_F13R2_FB21_Msk (0x1UL << CAN_F13R2_FB21_Pos) /*!< 0x00200000 */
#define CAN_F13R2_FB21 CAN_F13R2_FB21_Msk                /*!<Filter bit 21 */
#define CAN_F13R2_FB22_Pos (22U)
#define CAN_F13R2_FB22_Msk (0x1UL << CAN_F13R2_FB22_Pos) /*!< 0x00400000 */
#define CAN_F13R2_FB22 CAN_F13R2_FB22_Msk                /*!<Filter bit 22 */
#define CAN_F13R2_FB23_Pos (23U)
#define CAN_F13R2_FB23_Msk (0x1UL << CAN_F13R2_FB23_Pos) /*!< 0x00800000 */
#define CAN_F13R2_FB23 CAN_F13R2_FB23_Msk                /*!<Filter bit 23 */
#define CAN_F13R2_FB24_Pos (24U)
#define CAN_F13R2_FB24_Msk (0x1UL << CAN_F13R2_FB24_Pos) /*!< 0x01000000 */
#define CAN_F13R2_FB24 CAN_F13R2_FB24_Msk                /*!<Filter bit 24 */
#define CAN_F13R2_FB25_Pos (25U)
#define CAN_F13R2_FB25_Msk (0x1UL << CAN_F13R2_FB25_Pos) /*!< 0x02000000 */
#define CAN_F13R2_FB25 CAN_F13R2_FB25_Msk                /*!<Filter bit 25 */
#define CAN_F13R2_FB26_Pos (26U)
#define CAN_F13R2_FB26_Msk (0x1UL << CAN_F13R2_FB26_Pos) /*!< 0x04000000 */
#define CAN_F13R2_FB26 CAN_F13R2_FB26_Msk                /*!<Filter bit 26 */
#define CAN_F13R2_FB27_Pos (27U)
#define CAN_F13R2_FB27_Msk (0x1UL << CAN_F13R2_FB27_Pos) /*!< 0x08000000 */
#define CAN_F13R2_FB27 CAN_F13R2_FB27_Msk                /*!<Filter bit 27 */
#define CAN_F13R2_FB28_Pos (28U)
#define CAN_F13R2_FB28_Msk (0x1UL << CAN_F13R2_FB28_Pos) /*!< 0x10000000 */
#define CAN_F13R2_FB28 CAN_F13R2_FB28_Msk                /*!<Filter bit 28 */
#define CAN_F13R2_FB29_Pos (29U)
#define CAN_F13R2_FB29_Msk (0x1UL << CAN_F13R2_FB29_Pos) /*!< 0x20000000 */
#define CAN_F13R2_FB29 CAN_F13R2_FB29_Msk                /*!<Filter bit 29 */
#define CAN_F13R2_FB30_Pos (30U)
#define CAN_F13R2_FB30_Msk (0x1UL << CAN_F13R2_FB30_Pos) /*!< 0x40000000 */
#define CAN_F13R2_FB30 CAN_F13R2_FB30_Msk                /*!<Filter bit 30 */
#define CAN_F13R2_FB31_Pos (31U)
#define CAN_F13R2_FB31_Msk (0x1UL << CAN_F13R2_FB31_Pos) /*!< 0x80000000 */
#define CAN_F13R2_FB31 CAN_F13R2_FB31_Msk

#endif
