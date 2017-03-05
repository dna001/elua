// STM32 interrupt support

// Generic headers
#include "platform.h"
#include "platform_conf.h"
#include "elua_int.h"
#include "common.h"

// Platform-specific headers
#include "stm32f7xx.h"

#ifndef VTMR_TIMER_ID
#define VTMR_TIMER_ID         ( -1 )
#endif

// ****************************************************************************
// Interrupt handlers

extern USART_TypeDef *const stm32_usart[];

static void all_usart_irqhandler( int resnum )
{
  //int temp;

  //temp = USART_GetFlagStatus( stm32_usart[ resnum ], USART_FLAG_ORE );
  cmn_int_handler( INT_UART_RX, resnum );
  // if( temp == SET )
  //   for( temp = 0; temp < 10; temp ++ )
  //     platform_s_uart_send( resnum, '@' );
}

void USART1_IRQHandler()
{
  all_usart_irqhandler( 0 );
}

void USART2_IRQHandler()
{
  all_usart_irqhandler( 1 );
}

void USART3_IRQHandler()
{
  all_usart_irqhandler( 2 );
}

void UART4_IRQHandler()
{
  all_usart_irqhandler( 3 );
}

void UART5_IRQHandler()
{
  all_usart_irqhandler( 4 );
}

// ****************************************************************************
// External interrupt handlers

static u16 exti_line_to_gpio( u32 line )
{
  return PLATFORM_IO_ENCODE( ( SYSCFG->EXTICR[line >> 0x02] >> (0x04 * ( line & 0x03 ) ) ) & 0x07, line, PLATFORM_IO_ENC_PIN );
}

// Convert a GPIO ID to a EXINT number
static int exint_gpio_to_src( pio_type piodata )
{
  u16 pin = PLATFORM_IO_GET_PIN( piodata );
  return pin;
}

static void all_exti_irqhandler( int line )
{
  u16 v, port, pin;
  
  v = exti_line_to_gpio( line );
  port = PLATFORM_IO_GET_PORT( v );
  pin = PLATFORM_IO_GET_PIN( v );

  if( EXTI->RTSR & (1 << line ) && platform_pio_op( port, 1 << pin, PLATFORM_IO_PIN_GET ) )
    cmn_int_handler( INT_GPIO_POSEDGE, v );
  if( EXTI->FTSR & (1 << line ) && ( platform_pio_op( port, 1 << pin, PLATFORM_IO_PIN_GET ) == 0 ) )
    cmn_int_handler( INT_GPIO_NEGEDGE, v );

  __HAL_GPIO_EXTI_CLEAR_IT( 1u << line );
}

void EXTI0_IRQHandler()
{
  all_exti_irqhandler( 0 );
}

void EXTI1_IRQHandler()
{
  all_exti_irqhandler( 1 );
}

void EXTI2_IRQHandler()
{
  all_exti_irqhandler( 2 );
}

void EXTI3_IRQHandler()
{
  all_exti_irqhandler( 3 );
}

void EXTI4_IRQHandler()
{
  all_exti_irqhandler( 4 );
}

void EXTI9_5_IRQHandler()
{
  int i;
  for( i = 5; i < 10; i++ )
  {
    if (EXTI->PR & (1u << i))
      all_exti_irqhandler( i );
  }
}

void EXTI15_10_IRQHandler()
{
  int i;
  for( i = 10; i < 16; i++ )
  {
    if (EXTI->PR & (1u << i))
      all_exti_irqhandler( i );
  }
}

// ----------------------------------------------------------------------------
// Timer interrupt handlers

extern const TIM_TypeDef * const timer[];

extern u8 stm32_timer_int_periodic_flag[ NUM_PHYS_TIMER ];

static void tmr_int_handler( unsigned id )
{
  TIM_TypeDef *base = ( TIM_TypeDef* )timer[ id ];

  if (base->SR & TIM_IT_CC1)
  {
    /* Clear the IT pending Bit */
    base->SR = (uint16_t)~TIM_IT_CC1;

    if( id == VTMR_TIMER_ID )
      cmn_virtual_timer_cb();
    else
      cmn_int_handler( INT_TMR_MATCH, id );

    if( stm32_timer_int_periodic_flag[ id ] != PLATFORM_TIMER_INT_CYCLIC )
      base->DIER &= (uint16_t)~TIM_IT_CC1;
  }
}


void TIM1_CC_IRQHandler(void)
{
  tmr_int_handler( 0 );
}

void TIM2_IRQHandler(void)
{
  tmr_int_handler( 1 );
}

void TIM3_IRQHandler(void)
{
  tmr_int_handler( 2 );
}

void TIM4_IRQHandler(void)
{
  tmr_int_handler( 3 );
}

void TIM5_IRQHandler(void)
{
  tmr_int_handler( 4 );
}

void TIM8_CC_IRQHandler(void)
{
  tmr_int_handler( 7 );
}

// ****************************************************************************
// GPIO helper functions

static int gpioh_get_int_status( elua_int_id id, elua_int_resnum resnum )
{
  if ( EXTI->IMR & ( 1 << exint_gpio_to_src( resnum ) ) )
    return 1;
  else
    return 0;
}

static int gpioh_set_int_status( elua_int_id id, elua_int_resnum resnum, int status )
{
  int prev = gpioh_get_int_status( id, resnum );
  u32 exti_line = 1 << exint_gpio_to_src( resnum );
  
  if( status == PLATFORM_CPU_ENABLE )
  {
    // Configure port for interrupt line
    //GPIO_EXTILineConfig( PLATFORM_IO_GET_PORT( resnum ), PLATFORM_IO_GET_PIN( resnum ) );

    /* Clear EXTI line configuration */
    EXTI->IMR &= ~exti_line;
    EXTI->EMR &= ~exti_line;

    /* Clear Rising Falling edge configuration */
    EXTI->RTSR &= ~exti_line;
    EXTI->FTSR &= ~exti_line;

    if( id == INT_GPIO_NEGEDGE )
      EXTI->RTSR |= exti_line;
    else
      EXTI->FTSR |= exti_line;

    EXTI->IMR |= exti_line;

    __HAL_GPIO_EXTI_CLEAR_IT( exti_line );
  }
  else
  {
    //Disable edge
    if( id == INT_GPIO_POSEDGE )
      EXTI->RTSR &= ~exti_line;
    else
      EXTI->FTSR &= ~exti_line;
    
    //If no edges enabled, disable line interrupt
    if( ( ( EXTI->RTSR | EXTI->FTSR ) & exti_line ) == 0 )
      EXTI->IMR &= ~exti_line;
  }
  return prev;
}

static int gpioh_get_int_flag( elua_int_id id, elua_int_resnum resnum, int clear )
{
  int flag = 0;
  u32 mask =  1 << exint_gpio_to_src( resnum );

  if( __HAL_GPIO_EXTI_GET_FLAG( mask ) )
  {
    if( id == INT_GPIO_POSEDGE )
      flag = ( EXTI->RTSR & mask ) != 0;
    else
      flag = ( EXTI->FTSR & mask ) != 0;
  }
  if( flag && clear )
    __HAL_GPIO_EXTI_CLEAR_FLAG( mask );
  return flag;
}

// ****************************************************************************
// Interrupt: INT_GPIO_POSEDGE

static int int_gpio_posedge_set_status( elua_int_resnum resnum, int status )
{
  return gpioh_set_int_status( INT_GPIO_POSEDGE, resnum, status );
}

static int int_gpio_posedge_get_status( elua_int_resnum resnum )
{
  return gpioh_get_int_status( INT_GPIO_POSEDGE, resnum );
}

static int int_gpio_posedge_get_flag( elua_int_resnum resnum, int clear )
{
  return gpioh_get_int_flag( INT_GPIO_POSEDGE, resnum, clear );
}

// ****************************************************************************
// Interrupt: INT_GPIO_NEGEDGE

static int int_gpio_negedge_set_status( elua_int_resnum resnum, int status )
{
  return gpioh_set_int_status( INT_GPIO_NEGEDGE, resnum, status );
}

static int int_gpio_negedge_get_status( elua_int_resnum resnum )
{
  return gpioh_get_int_status( INT_GPIO_NEGEDGE, resnum );
}

static int int_gpio_negedge_get_flag( elua_int_resnum resnum, int clear )
{
  return gpioh_get_int_flag( INT_GPIO_NEGEDGE, resnum, clear );
}

// ****************************************************************************
// Interrupt: INT_TMR_MATCH

static int int_tmr_match_get_status( elua_int_resnum resnum )
{
  TIM_TypeDef *base = ( TIM_TypeDef* )timer[ resnum ];

  return ( base->DIER & TIM_IT_CC1 ) != 0;
}

static int int_tmr_match_set_status( elua_int_resnum resnum, int status )
{
  int previous = int_tmr_match_get_status( resnum );
  TIM_TypeDef *base = ( TIM_TypeDef* )timer[ resnum ];
  
  if ( status == PLATFORM_CPU_ENABLE )
    base->DIER |= TIM_IT_CC1;
  else
    base->DIER &= ~TIM_IT_CC1;
  return previous;
}

static int int_tmr_match_get_flag( elua_int_resnum resnum, int clear )
{
  TIM_TypeDef *base = ( TIM_TypeDef* )timer[ resnum ];
  int status = ( base->SR & TIM_FLAG_CC1 ) ? 1 : 0;

  if( clear )
    base->SR = (u16)~TIM_FLAG_CC1;
  return status;
}

// ****************************************************************************
// Interrupt: INT_UART_RX

static int int_uart_rx_get_status( elua_int_resnum resnum )
{
  USART_TypeDef *base = ( USART_TypeDef* )stm32_usart[ resnum ];
  return ( ( base->CR1 & (1u << 5) ) != 0 ) && ( ( base->ISR & USART_FLAG_RXNE ) != 0 ) ? 1 : 0;
}

static int int_uart_rx_set_status( elua_int_resnum resnum, int status )
{
  USART_TypeDef *base = ( USART_TypeDef* )stm32_usart[ resnum ];
  int prev = int_uart_rx_get_status( resnum );
  if ( status == PLATFORM_CPU_ENABLE ) {
    base->CR1 |= (1u << 5); // RXNEIE
  } else {
    base->CR1 &= ~(1u << 5); // RXNEIE
  }
  return prev;
}

static int int_uart_rx_get_flag( elua_int_resnum resnum, int clear )
{
  USART_TypeDef *base = ( USART_TypeDef* )stm32_usart[ resnum ];
  int status = ( ( base->ISR & USART_FLAG_RXNE ) != 0 ) ? 1 : 0;
  if( clear )
    base->ISR = (u16)~USART_FLAG_RXNE;
  return status;
}

// ****************************************************************************
// Initialize interrupt subsystem

// UART IRQ table
static const u8 uart_irq_table[] = { USART1_IRQn, USART2_IRQn, USART3_IRQn, UART4_IRQn, UART5_IRQn };

// EXTI IRQ table
static const u8 exti_irq_table[] = { EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn, EXTI4_IRQn, EXTI9_5_IRQn, EXTI15_10_IRQn };

// EXTI IRQ table
static const u8 timer_irq_table[] = { TIM1_CC_IRQn, TIM2_IRQn, TIM3_IRQn, TIM4_IRQn, TIM5_IRQn };

void platform_int_init()
{
  unsigned i;
  
  // Enable all USART interrupts in the NVIC
  for( i = 0; i < sizeof( uart_irq_table ) / sizeof( u8 ); i ++ )
  {
    HAL_NVIC_SetPriority( uart_irq_table[ i ], 0, 0 );
    HAL_NVIC_EnableIRQ( uart_irq_table[ i ] );
  }

  // Enable all EXTI interrupts in the NVIC
  for( i = 0; i < sizeof( exti_irq_table ) / sizeof( u8 ); i ++ )
  {
    HAL_NVIC_SetPriority( exti_irq_table[ i ], 0, 0 );
    HAL_NVIC_EnableIRQ( exti_irq_table[ i ] );
  }

#ifdef INT_TMR_MATCH
  for( i = 0; i < sizeof( timer_irq_table ) / sizeof( u8 ); i ++ )
  {
    HAL_NVIC_SetPriority( timer_irq_table[ i ], 0, 1 );
    HAL_NVIC_EnableIRQ( timer_irq_table[ i ] );
  }
#endif  

}

// ****************************************************************************
// Interrupt table
// Must have a 1-to-1 correspondence with the interrupt enum in platform_ints.h!

const elua_int_descriptor elua_int_table[ INT_ELUA_LAST ] = 
{
  { int_gpio_posedge_set_status, int_gpio_posedge_get_status, int_gpio_posedge_get_flag },
  { int_gpio_negedge_set_status, int_gpio_negedge_get_status, int_gpio_negedge_get_flag },
  { int_tmr_match_set_status, int_tmr_match_get_status, int_tmr_match_get_flag },
  { int_uart_rx_set_status, int_uart_rx_get_status, int_uart_rx_get_flag }  
};
