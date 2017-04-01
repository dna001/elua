// Platform-dependent functions

#include "platform.h"
#include "type.h"
#include "devman.h"
#include "genstd.h"
#include <reent.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include "uip_arp.h"
#include "elua_uip.h"
#include "elua_adc.h"
#include "uip-conf.h"
#include "platform_conf.h"
#include "diskio.h"
#include "common.h"
#include "buf.h"
#include "utils.h"
#include "lua.h"
#include "lauxlib.h"
#include "lrotable.h"

// Platform specific includes
#include "pll_config.h"
#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#if defined( ELUA_BOARD_INTERNAL_CLOCK_HZ )
#define HCLK        ( (HSI_VALUE / PLL_M) * PLL_N / PLL_P)
#else
#define HCLK        ( (HSE_VALUE / PLL_M) * PLL_N / PLL_P)
#endif

#define PCLK1_DIV   2
#define PCLK2_DIV   1

// SysTick Config Data
// NOTE: when using virtual timers, SYSTICKHZ and VTMR_FREQ_HZ should have the
// same value, as they're served by the same timer (the systick)
// Max SysTick preload value is 16777215, for STM32F103RET6 @ 72 MHz, lowest acceptable rate would be about 5 Hz
#define SYSTICKHZ               20
#define SYSTICKMS               (1000 / SYSTICKHZ)

//#if ( (HCLK / SYSTICKHZ)  > SysTick_LOAD_RELOAD_Msk)
//#error  "Sys tick reload value out of range"
//#endif

//#define WATCHDOG_ENABLE
#define WATCH_COUNTER_RESET     127

// ****************************************************************************
// Platform initialization

// forward dcls
static void CPU_CACHE_Enable(void);
static void setsysclock(void);
static void NVIC_Configuration(void);

static void timers_init();
static void uarts_init();
static void pios_init();

int platform_init()
{
  // Enable the CPU Cache
  CPU_CACHE_Enable();

  HAL_Init();

  setsysclock();

  // Setup IRQ's
  NVIC_Configuration();

  // Setup PIO
  pios_init();

  // Setup UARTs
  uarts_init();

  // Setup timers
  timers_init();

  // Setup system timer
  cmn_systimer_set_base_freq( HCLK );
  cmn_systimer_set_interrupt_freq( SYSTICKHZ );

  // Enable SysTick
  //if ( SysTick_Config( HCLK / SYSTICKHZ ) )
  //{
    /* Capture error */
  //  while (1);
  //}

#if defined( WATCHDOG_ENABLE )
  // Enable Watchdog
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
  WWDG_SetPrescaler(WWDG_Prescaler_8);
  WWDG_SetWindowValue( WATCH_COUNTER_RESET );
  WWDG_Enable( WATCH_COUNTER_RESET );
#endif

#ifdef BUILD_WOFS
  // Flash initialization (for WOFS)
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

  FLASH_Unlock();
#endif

  cmn_platform_init();

  // All done
  return PLATFORM_OK;
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            PLL_R                          = 7  
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void setsysclock(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 7;
  
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Activate the OverDrive to reach the 216 MHz Frequency */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; 
  
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }  
}

// ****************************************************************************
// NVIC
// Shared by all STM32 devices.

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures the nested vectored interrupt controller.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/* This struct is used for later reconfiguration of ADC interrupt */
//NVIC_InitTypeDef nvic_init_structure_adc;

static void NVIC_Configuration(void)
{
#if 0
  NVIC_InitTypeDef nvic_init_structure;

#ifdef  VECT_TAB_RAM
  /* Set the Vector Table base location at 0x20000000 */
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

  /* Configure the NVIC Preemption Priority Bits */
  /* Priority group 0 disables interrupt nesting completely */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  // Lower the priority of the SysTick interrupt to let the
  // UART interrupt preempt it
  nvic_init_structure.NVIC_IRQChannel = SysTick_IRQn;
  nvic_init_structure.NVIC_IRQChannelPreemptionPriority = 0;
  nvic_init_structure.NVIC_IRQChannelSubPriority = 1;
  nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic_init_structure);

#ifdef BUILD_ADC
  nvic_init_structure_adc.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  nvic_init_structure_adc.NVIC_IRQChannelPreemptionPriority = 0;
  nvic_init_structure_adc.NVIC_IRQChannelSubPriority = 2;
  nvic_init_structure_adc.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&nvic_init_structure_adc);
#endif
#endif
}

// ****************************************************************************
// PIO
// This is pretty much common code to all STM32 devices.
// todo: Needs updates to support different processor lines.
GPIO_TypeDef * const pio_port[] = { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF,
  GPIOG, GPIOH, GPIOI, GPIOJ, GPIOK };

static void pios_init()
{
  //GPIO_InitTypeDef GPIO_InitStructure;
  int port;

  // Enable all GPIO port clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();

  for( port = 0; port < NUM_PIO; port++ )
  {
    // Default all port pins to input and enable port.
    //GPIO_StructInit(&GPIO_InitStructure);
    //GPIO_Init(pio_port[port], &GPIO_InitStructure);
  }

#if 0
#if defined(ENABLE_JTAG_SWD) || defined(ENABLE_TRACE)
  //Mapping JTAG / SWD pins
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4,  GPIO_AF_SWJ); // PB4  TRST
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource3,  GPIO_AF_SWJ); // PB3  TDO   / SWO

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource13, GPIO_AF_SWJ); // PA13 TMS   / SWDIO
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource14, GPIO_AF_SWJ); // PA14 TCK   / SWDCLK
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_SWJ); // PA15 TDI

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

#endif

#ifdef ENABLE_TRACE
  //Mapping TRACE pins, PE2,3,4,5,6
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOE, GPIO_PinSource2, GPIO_AF_TRACE);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource3, GPIO_AF_TRACE);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_TRACE);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TRACE);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TRACE);
#endif
#endif
}

pio_type platform_pio_op( unsigned port, pio_type pinmask, int op )
{
  pio_type retval = 1;
  GPIO_InitTypeDef GPIO_InitStructure = { 0 };
  GPIO_TypeDef * base = pio_port[ port ];

  switch( op )
  {
    case PLATFORM_IO_PORT_SET_VALUE:
      base->ODR = pinmask;
      break;

    case PLATFORM_IO_PIN_SET:
      HAL_GPIO_WritePin(base, pinmask, GPIO_PIN_SET);
      break;

    case PLATFORM_IO_PIN_CLEAR:
      HAL_GPIO_WritePin(base, pinmask, GPIO_PIN_RESET);
      break;

    case PLATFORM_IO_PORT_DIR_INPUT:
      pinmask = GPIO_PIN_All;
    case PLATFORM_IO_PIN_DIR_INPUT:
      GPIO_InitStructure.Pin  = pinmask;
      GPIO_InitStructure.Mode = GPIO_MODE_INPUT;

      HAL_GPIO_Init(base, &GPIO_InitStructure);
      break;

    case PLATFORM_IO_PORT_DIR_OUTPUT:
      pinmask = GPIO_PIN_All;
    case PLATFORM_IO_PIN_DIR_OUTPUT:
      GPIO_InitStructure.Pin   = pinmask;
      GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

      HAL_GPIO_Init(base, &GPIO_InitStructure);
      break;

    case PLATFORM_IO_PORT_GET_VALUE:
      retval = pinmask == PLATFORM_IO_READ_IN_MASK ? base->IDR : base->ODR;
      break;

    case PLATFORM_IO_PIN_GET:
      retval = HAL_GPIO_ReadPin(base, pinmask);
      break;

    case PLATFORM_IO_PIN_PULLUP:
      GPIO_InitStructure.Pin   = pinmask;
      GPIO_InitStructure.Pull = GPIO_PULLUP;

      HAL_GPIO_Init(base, &GPIO_InitStructure);
      break;

    case PLATFORM_IO_PIN_PULLDOWN:
      GPIO_InitStructure.Pin   = pinmask;
      GPIO_InitStructure.Pull  = GPIO_PULLDOWN;

      HAL_GPIO_Init(base, &GPIO_InitStructure);
      break;

    case PLATFORM_IO_PIN_NOPULL:
      GPIO_InitStructure.Pin   = pinmask;
      GPIO_InitStructure.Pull  = GPIO_NOPULL;

      HAL_GPIO_Init(base, &GPIO_InitStructure);
      break;

    default:
      retval = 0;
      break;
  }
  return retval;
}

// ****************************************************************************
// UART
// TODO: Support timeouts.

// All possible STM32 uarts defs
static UART_HandleTypeDef stm32_uart_hnd[2];
USART_TypeDef *const stm32_usart[] = { USART1, USART6 };
const u8 stm32_usart_AF[] = { GPIO_AF7_USART1, GPIO_AF8_USART6 };
static GPIO_TypeDef *const usart_gpio_rx_port[] = { GPIOA, GPIOC };
static GPIO_TypeDef *const usart_gpio_tx_port[] = { GPIOB, GPIOC };
static const u16 usart_gpio_rx_pin[] = { GPIO_PIN_10, GPIO_PIN_7 };
static const u16 usart_gpio_tx_pin[] = { GPIO_PIN_7, GPIO_PIN_6 };

static GPIO_TypeDef *const usart_gpio_hwflow_port[] = { GPIOA, GPIOA };
static const u16 usart_gpio_cts_pin[] = { GPIO_PIN_11, GPIO_PIN_0 };
static const u16 usart_gpio_rts_pin[] = { GPIO_PIN_12, GPIO_PIN_1 };

static void uart_init(u32 id, UART_InitTypeDef * initVals)
{
  UART_HandleTypeDef *UartHandle = &stm32_uart_hnd[ id ];
  /* Configure USART IO */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_TypeDef* prxport = usart_gpio_rx_port[ id ];
  GPIO_TypeDef* ptxport = usart_gpio_tx_port[ id ];
  u16 gpio_rx_pin = usart_gpio_rx_pin[ id ];
  u16 gpio_tx_pin = usart_gpio_tx_pin[ id ];

  // Overwrite console UART configuration with the parameters from the configuration file
  if( id == CON_UART_ID )
  {
    prxport = CON_GPIO_PORT_MACRO( STM32F7_CON_RX_PORT );
    ptxport = CON_GPIO_PORT_MACRO( STM32F7_CON_TX_PORT );
    gpio_rx_pin = CON_GPIO_PIN_MACRO( STM32F7_CON_RX_PIN );
    gpio_tx_pin = CON_GPIO_PIN_MACRO( STM32F7_CON_TX_PIN );
  }

  /* Configure USART Tx Pin as alternate function push-pull */
  GPIO_InitStructure.Pin = gpio_tx_pin;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_PULLUP; //pull-up or pull-down
  GPIO_InitStructure.Alternate = stm32_usart_AF[id];
  HAL_GPIO_Init(ptxport, &GPIO_InitStructure);

  /* Configure USART Rx Pin as input floating */
  GPIO_InitStructure.Pin = gpio_rx_pin;
  HAL_GPIO_Init(prxport, &GPIO_InitStructure);

  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle->Instance = stm32_usart[ id ];
  UartHandle->Init = *initVals;

  HAL_UART_DeInit(UartHandle);

  /* Configure USART */
  HAL_UART_Init(UartHandle);
}

static void uarts_init()
{
  // Enable clocks.
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();
}

u32 platform_uart_setup( unsigned id, u32 baud, int databits, int parity, int stopbits )
{
  UART_InitTypeDef UART_InitStructure;

  if( id == CDC_UART_ID ) // no dynamic configuration yet
    return 0;

  UART_InitStructure.BaudRate = baud;
  UART_InitStructure.HwFlowCtl = UART_HWCONTROL_NONE;
  UART_InitStructure.Mode = UART_MODE_TX_RX;

  switch( databits )
  {
    case 5:
    case 6:
    case 7:
    case 8:
      UART_InitStructure.WordLength = UART_WORDLENGTH_8B;
      break;
    case 9:
      UART_InitStructure.WordLength = UART_WORDLENGTH_9B;
      break;
    default:
      UART_InitStructure.WordLength = UART_WORDLENGTH_8B;
      break;
  }

  switch (stopbits)
  {
    case PLATFORM_UART_STOPBITS_1:
      UART_InitStructure.StopBits = UART_STOPBITS_1;
      break;
    case PLATFORM_UART_STOPBITS_2:
      UART_InitStructure.StopBits = UART_STOPBITS_2;
      break;
    default:
      UART_InitStructure.StopBits = UART_STOPBITS_1;
      break;
  }

  switch (parity)
  {
    case PLATFORM_UART_PARITY_EVEN:
      UART_InitStructure.Parity = USART_PARITY_EVEN;
      break;
    case PLATFORM_UART_PARITY_ODD:
      UART_InitStructure.Parity = USART_PARITY_ODD;
      break;
    default:
      UART_InitStructure.Parity = USART_PARITY_NONE;
      break;
  }

  uart_init(id, &UART_InitStructure);

  return baud;
}

extern uint16_t VCP_DataTx(uint8_t* Buf, uint32_t Len);
void platform_s_uart_send( unsigned id, u8 data )
{
#ifdef BUILD_USB_CDC
  if( id == CDC_UART_ID )
    VCP_DataTx( &data, 1 );
  else
#endif
  {
    HAL_UART_Transmit( &stm32_uart_hnd[ id ], &data, 1, 100 );
  }
}

int platform_s_uart_recv( unsigned id, timer_data_type timeout )
{
  HAL_StatusTypeDef status;
  u8 data;
  if( id == CDC_UART_ID ) // this shouldn't happen
    return -1;
  status = HAL_UART_Receive( &stm32_uart_hnd[ id ], &data, 1, timeout );
  if (status != HAL_OK)
    return -1;
  return data;
}

int platform_s_uart_set_flow_control( unsigned id, int type )
{
  USART_TypeDef *usart = stm32_usart[ id ];
  int temp = 0;
  GPIO_InitTypeDef GPIO_InitStructure;

  if( id >= 3 ) // on STM32 only USART1 through USART3 have hardware flow control ([TODO] but only on high density devices?)
    return PLATFORM_ERR;

  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;

  if( type == PLATFORM_UART_FLOW_NONE )
  {
    usart->CR3 &= ~UART_HWCONTROL_RTS_CTS;
    GPIO_InitStructure.Pin = usart_gpio_rts_pin[ id ] | usart_gpio_cts_pin[ id ];
    HAL_GPIO_Init( usart_gpio_hwflow_port[ id ], &GPIO_InitStructure );
    return PLATFORM_OK;
  }
  if( type & PLATFORM_UART_FLOW_CTS )
  {
    temp |= UART_HWCONTROL_CTS;
    GPIO_InitStructure.Pin = usart_gpio_cts_pin[ id ];
    HAL_GPIO_Init( usart_gpio_hwflow_port[ id ], &GPIO_InitStructure );
  }
  if( type & PLATFORM_UART_FLOW_RTS )
  {
    temp |= UART_HWCONTROL_RTS;
    GPIO_InitStructure.Pin = usart_gpio_rts_pin[ id ];
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init( usart_gpio_hwflow_port[ id ], &GPIO_InitStructure );
  }
  usart->CR3 |= temp;
  return PLATFORM_OK;
}

// ****************************************************************************
// Timers

u8 stm32_timer_int_periodic_flag[ NUM_PHYS_TIMER ];

static TIM_HandleTypeDef timer_hnd[2];

// We leave out TIM6/TIM for now, as they are dedicated
const TIM_TypeDef * const timer[] = {
  TIM1,   // ID: 0
  TIM2    // ID: 1
};

const u8 timer_width[] = {
  16,  // ID: 0
  32   // ID: 1
};

#define TIM_GET_PRESCALE( id ) ( (((id) == 0) || ((id) == 5)|| ((id) == 6)|| ((id) == 7)|| ((id) == 8)) ? ( PCLK2_DIV ) : ( PCLK1_DIV ) )
#define TIM_GET_BASE_CLK( id ) ( HCLK / ( TIM_GET_PRESCALE( id ) / 2 ) )

#define TIM_STARTUP_CLOCK       50000

static u32 platform_timer_set_clock( unsigned id, u32 clock );

void SysTick_Handler( void )
{
  HAL_IncTick();
  // Handle virtual timers
  cmn_virtual_timer_cb();

  // Handle system timer call
  cmn_systimer_periodic();

#if defined( WATCHDOG_ENABLE )
  // Refresh watchdog if enabled
  WWDG_SetCounter( WATCH_COUNTER_RESET );
#endif

}

static void timers_init()
{
  unsigned i;

  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();

  // Configure timers
  for( i = 0; i < NUM_TIMER; i ++ )
    timer_hnd[i].Instance = (TIM_TypeDef*)timer[ i ];
    platform_timer_set_clock( i, TIM_STARTUP_CLOCK );
}

static u32 platform_timer_get_clock( unsigned id )
{
  TIM_TypeDef* ptimer = (TIM_TypeDef*)timer[ id ];
  return TIM_GET_BASE_CLK( id ) / ( ptimer->PSC + 1 );
}

static u32 platform_timer_set_clock( unsigned id, u32 clock )
{
  TIM_Base_InitTypeDef timer_base_struct;
  u32 pre = ( TIM_GET_BASE_CLK( id ) / clock ) - 1;

  if( pre > 65535 ) // Limit prescaler to 16-bits
    pre = 65535;

  timer_base_struct.Period = ( timer_width[id] == 32 ? 0xFFFFFFFF : 0xFFFF );
  timer_base_struct.Prescaler = ( u16 )pre;
  timer_base_struct.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer_base_struct.CounterMode = TIM_COUNTERMODE_UP;
  timer_base_struct.RepetitionCounter = 0x0000;
  timer_hnd[ id ].Init = timer_base_struct;
  HAL_TIM_Base_Init( &timer_hnd[ id ] );
  HAL_TIM_Base_Start( &timer_hnd[ id ] );

  return  platform_timer_get_clock( id );
}

void platform_s_timer_delay( unsigned id, timer_data_type delay_us )
{
  TIM_TypeDef *ptimer = (TIM_TypeDef*)timer[ id ];
  volatile unsigned dummy;
  timer_data_type final;

  final = ( ( u64 )delay_us * platform_timer_get_clock( id ) ) / 1000000;
  __HAL_TIM_SET_COUNTER( &timer_hnd[ id ], 0 );
  // clear update flag so we can detect when it wraps
  ptimer->SR &= ~TIM_SR_UIF;
  for( dummy = 0; dummy < 200; dummy ++ );
  u64 timer_period = (u64)ptimer->ARR + 1;
  while( __HAL_TIM_GET_COUNTER( &timer_hnd[ id ] ) < final )
  {
    if ( ptimer->SR & TIM_SR_UIF )
    {
      // timer has wrapped
      ptimer->SR &= ~TIM_SR_UIF;
      if ( final >= timer_period )
        final -= timer_period;
    }
  }
}

timer_data_type platform_s_timer_op( unsigned id, int op, timer_data_type data )
{
  u32 res = 0;
  volatile unsigned dummy;

  data = data;
  switch( op )
  {
    case PLATFORM_TIMER_OP_START:
      __HAL_TIM_SET_COUNTER( &timer_hnd[ id ], 0 );
      for( dummy = 0; dummy < 200; dummy ++ );
      break;

    case PLATFORM_TIMER_OP_READ:
      res = __HAL_TIM_GET_COUNTER( &timer_hnd[ id ] );
      break;

    case PLATFORM_TIMER_OP_SET_CLOCK:
      res = platform_timer_set_clock( id, data );
      break;

    case PLATFORM_TIMER_OP_GET_CLOCK:
      res = platform_timer_get_clock( id );
      break;

    case PLATFORM_TIMER_OP_GET_MAX_CNT:
      res = ( timer_width[id] == 32 ? 0xFFFFFFFF : 0xFFFF );
      break;

  }
  return res;
}

int platform_s_timer_set_match_int( unsigned id, timer_data_type period_us, int type )
{
  TIM_TypeDef* base = ( TIM_TypeDef* )timer[ id ];
  u64 period;
  u32 prescaler, freq;
  timer_data_type final;
  TIM_OC_InitTypeDef  TIM_OCInitStructure = { 0 };

  if( period_us == 0 )
  {
    __HAL_TIM_DISABLE_IT( &timer_hnd[ id ], TIM_IT_CC1 );
    return PLATFORM_TIMER_INT_OK;
  }

  period = ( ( u64 )TIM_GET_BASE_CLK( id ) * period_us ) / 1000000;

  prescaler = (u32)( period / ((u64)1 << timer_width[id]) ) + 1;

  if( prescaler > 0xFFFF )
    return PLATFORM_TIMER_INT_TOO_LONG;

  platform_timer_set_clock( id, TIM_GET_BASE_CLK( id  ) / prescaler );
  freq = platform_timer_get_clock( id );
  final = ( ( u64 )period_us * freq ) / 1000000;

  if( final == 0 )
    return PLATFORM_TIMER_INT_TOO_SHORT;

  __HAL_TIM_DISABLE( &timer_hnd[ id ] );

  TIM_OCInitStructure.OCMode = TIM_OCMODE_TIMING;
  //TIM_OCInitStructure.OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.Pulse = final;
  TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
  HAL_TIM_OC_ConfigChannel( &timer_hnd[ id ] , &TIM_OCInitStructure, TIM_CHANNEL_1 );

  // Patch timer configuration to reload when period is reached
  __HAL_TIM_SET_AUTORELOAD( &timer_hnd[ id ], final );

  /* Set the Preload enable bit for channel1 */
  base->CCMR1 |= TIM_CCMR1_OC1PE;

  stm32_timer_int_periodic_flag[ id ] = type;

  __HAL_TIM_SET_COUNTER( &timer_hnd[ id ], 0 );
  __HAL_TIM_ENABLE( &timer_hnd[ id ] );
  //TIM_ITConfig( base, TIM_IT_CC1, ENABLE );

  return PLATFORM_TIMER_INT_OK;
}

u64 platform_timer_sys_raw_read()
{
  return SysTick->LOAD - SysTick->VAL;
}

void platform_timer_sys_disable_int()
{
  SysTick->CTRL &= ~( 1 << SysTick_CTRL_TICKINT_Pos );
}

void platform_timer_sys_enable_int()
{
  SysTick->CTRL |= 1 << SysTick_CTRL_TICKINT_Pos;
}

timer_data_type platform_timer_read_sys()
{
  return cmn_systimer_get();
}

#ifdef ENABLE_ENC
// ****************************************************************************
// Quadrature Encoder Support (uses timers)
// No pin configuration, many of the timers should work with default config if
// pins aren't reconfigured for another peripheral

void stm32_enc_init( unsigned id )
{
  TIM_TypeDef *ptimer = (TIM_TypeDef *)timer[ id ];

  TIM_Cmd( ptimer, DISABLE );
  TIM_DeInit( ptimer );
  TIM_SetCounter( ptimer, 0 );
  TIM_EncoderInterfaceConfig( ptimer, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_Cmd( ptimer, ENABLE );
}

void stm32_enc_set_counter( unsigned id, unsigned count )
{
  TIM_TypeDef *ptimer = (TIM_TypeDef *)timer[ id ];

  TIM_SetCounter( ptimer, ( u16 )count );
}
#endif

// *****************************************************************************
// CPU specific functions

extern u32 SystemCoreClock;
u32 platform_s_cpu_get_frequency()
{
  SystemCoreClockUpdate();
  return SystemCoreClock;
}

void stm32_cpu_reset()
{
  NVIC_SystemReset();
}

// ****************************************************************************
// Flash access functions

#ifdef BUILD_WOFS
u32 platform_s_flash_write( const void *from, u32 toaddr, u32 size )
{
  u32 ssize = 0;
  const u16 *psrc = ( const u16* )from;
  FLASH_Status flstat;

  while( ssize < size )
  {
    if( ( flstat = FLASH_ProgramHalfWord( toaddr, *psrc ++ ) ) != FLASH_COMPLETE )
    {
      printf( "ERROR in platform_s_flash_write: stat=%d at %08X\n", ( int )flstat, ( unsigned )toaddr );
      break;
    }
    toaddr += 2;
    ssize += 2;
  }
  return ssize;
}

static const u16 flash_sectors[] = { FLASH_Sector_0, FLASH_Sector_1, FLASH_Sector_2, FLASH_Sector_3,
                                     FLASH_Sector_4, FLASH_Sector_5, FLASH_Sector_6, FLASH_Sector_7,
                                     FLASH_Sector_8, FLASH_Sector_9, FLASH_Sector_10, FLASH_Sector_11 };

int platform_flash_erase_sector( u32 sector_id )
{
  return FLASH_EraseSector( flash_sectors[ sector_id ], VoltageRange_3 ) == FLASH_COMPLETE ? PLATFORM_OK : PLATFORM_ERR;
}

#endif // #ifdef BUILD_WOFS

