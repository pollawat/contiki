#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__


#define PLATFORM_HAS_LEDS   1
#define PLATFORM_HAS_BUTTON 0

/* **************************************************************************** */
/* ------------------------------ CC1120 Related ------------------------------ */
/* **************************************************************************** */

//#define CC1120DEBUG		1
//#define CC1120TXDEBUG		1
//#define CC1120RXDEBUG		1
//#define CC1120INTDEBUG		1
//#define C1120PROCESSDEBUG	1
//#define CC1120ARCHDEBUG		1
//#define CC1120STATEDEBUG	1

#define RF_CHANNEL				42

#define CC1120LEDS				1

#define CC1120_FHSS_ETSI_50		1
#define CC1120_FHSS_FCC_50		0

#define CC1120GPIOTXCHK			1

#define CC1120_OFF_STATE CC1120_STATE_IDLE

#define CC1120_CCA_PIN_PRESENT 0


#define CC1120_GPIO0_FUNC	(CC1120_GPIO_PKT_SYNC_RXTX | CC1120_GPIO_INV_MASK)	
//#define CC1120_GPIO2_FUNC
#define CC1120_GPIO3_FUNC	CC1120_GPIO_CLEAR_CHANEL_ASSESSMENT


/* --------------------------- CC1120 Pin Mappings. --------------------------- */
#define CC1120_RESET_PORT(type)		GPIOB_##type
#define CC1120_RESET_PIN		8
#define CC1120_RESET_PCR		PORTB_PCR8

#define CC1120_SPI_CSN_PORT(type)	GPIOD_##type
#define CC1120_SPI_CSN_PIN		0
#define CC1120_SPI_CSN_PCR		PORTD_PCR0

#define CC1120_GDO0_PORT(type) 		GPIOA_##type
#define CC1120_GDO0_PIN			5
#define CC1120_GDO0_PCR			PORTA_PCR5

#ifdef CC1120_GPIO2_FUNC
#define CC1120_GDO1_PORT(type) 		GPIOA_##type
#define CC1120_GDO1_PIN			12
#define CC1120_GDO1_PCR			PORTA_PCR12
#endif

#ifdef CC1120_GPIO3_FUNC
#define CC1120_GDO3_PORT(type)		GPIOA_##type
#define CC1120_GDO3_PIN			13
#define CC1120_GDO3_PCR			PORTA_PCR13
#endif

#define CC1120_SPI_MISO_PORT(type)	GPIOD_##type
#define CC1120_SPI_MISO_PIN		3		//17
#define CC1120_SPI_MISO_PCR	PORTD_PCR3

#define CC1120_SPI_MOSI_PORT(type)	GPIOD_##type
#define CC1120_SPI_MOSI_PIN		2
#define CC1120_SPI_MOSI_PCR	PORTD_PCR2

#define CC1120_SPI_CLK_PORT(type)	GPIOC_##type
#define CC1120_SPI_CLK_PIN		5
#define CC1120_SPI_CLK_PCR		PORTC_PCR5


/* ---------------------------- LED Pin Mappings. ----------------------------- */
#define LED_RED_PORT_MASK			PORTB_EN_MASK
#define LED_RED_PORT(type)			GPIOB_##type
#define LED_RED_PIN				18
#define LED_RED_PCR				PORTB_PCR18

#define LED_GREEN_PORT_MASK			PORTB_EN_MASK
#define LED_GREEN_PORT(type)		GPIOB_##type
#define LED_GREEN_PIN			19
#define LED_GREEN_PCR			PORTB_PCR19

#define LED_BLUE_PORT_MASK			PORTD_EN_MASK
#define LED_BLUE_PORT(type)			GPIOD_##type
#define LED_BLUE_PIN			1
#define LED_BLUE_PCR			PORTD_PCR1

#endif /* __PLATFORM_CONF_H__ */
