#include "stm32f405xx.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_rtc.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_tim.h"

#define F_CPU       168000000UL
#define AHB1        F_CPU
#define APB1        F_CPU/4
#define APB1_TIM    APB1*2
#define APB2        F_CPU/2
#define APB2_TIM    APB2*2
#define SysTicksClk 10000
#define SysTicks    F_CPU/SysTicksClk

enum PINs
{
    PIN0=0, PIN1,   PIN2,   PIN3,
    PIN4,   PIN5,   PIN6,   PIN7,
    PIN8,   PIN9,   PIN10,  PIN11,
    PIN12,  PIN13,  PIN14,  PIN15
};

enum STATEs
{
    LOAD=0,
		AUDIO_OFF,
    MAIN,
    SET_TIME
};

enum SET_TIME_STATEs
{
    SET_TIME_HOUR=0,
    SET_TIME_MIN,
    SET_DATE_YEAR,
    SET_DATE_MON,
    SET_DATE_DAY,
    SET_SET
};

enum INPUTs
{
    FM=0,
    BT,
    USB,
    AUX
};

/*--------------------------------------------------------------------------------*/
//TDA Defs
#define TDA7419_ADRESS		0x88
#define TDA_MAIN_SOURCE		0x00
#define TDA_SOURCE_MUTE		0x07

const uint8_t TDA_inputs[4] = {0x82, 0x84, 0x81, 0x83};


/*--------------------------------------------------------------------------------*/
//TFT commands

uint8_t loading_txt[13] = {'l','o','d','.','v','a','l','=','0','0',255,255,255};

uint8_t TFT_TIME[49] = {'t','i','m','e','.','t','x','t','=','"',0,0,':',0,0,'"',255,255,255,\
                        'd','a','y','.','t','x','t','=','"',0,0,'"',255,255,255,\
                        'm','o','n','.','t','x','t','=','"',0,0,'"',255,255,255}; 

uint8_t pages[4][12] = {{'p','a','g','e',' ','l','o','a','d',255,255,255},
												{'p','a','g','e',' ','a','o','f','f',255,255,255},
                        {'p','a','g','e',' ','m','a','i','n',255,255,255},
                        {'p','a','g','e',' ','s','e','t','t',255,255,255}};

uint8_t input_tft[4][24] = {{'A','U','X','.','v','a','l','=','0',255,255,255,'F','M','I','.','v','a','l','=','1',255,255,255},
                            {'F','M','I','.','v','a','l','=','0',255,255,255,'B','T','I','.','v','a','l','=','1',255,255,255},
                            {'B','T','I','.','v','a','l','=','0',255,255,255,'U','S','B','.','v','a','l','=','1',255,255,255},
                            {'U','S','B','.','v','a','l','=','0',255,255,255,'A','U','X','.','v','a','l','=','1',255,255,255}};
                                
uint8_t set_time_tft[6][24] = {{'s','e','t','.','v','a','l','=','0',255,255,255,'t','_','h','.','v','a','l','=','1',255,255,255},
                               {'t','_','h','.','v','a','l','=','0',255,255,255,'t','_','m','.','v','a','l','=','1',255,255,255},
                               {'t','_','m','.','v','a','l','=','0',255,255,255,'d','_','y','.','v','a','l','=','1',255,255,255},
                               {'d','_','y','.','v','a','l','=','0',255,255,255,'d','_','m','.','v','a','l','=','1',255,255,255},
                               {'d','_','m','.','v','a','l','=','0',255,255,255,'d','_','d','.','v','a','l','=','1',255,255,255},
                               {'d','_','d','.','v','a','l','=','0',255,255,255,'s','e','t','.','v','a','l','=','1',255,255,255}};

uint8_t set_time_txt[75] = {'t','_','h','.','t','x','t','=','"','0','0','"',255,255,255,\
                            't','_','m','.','t','x','t','=','"','0','0','"',255,255,255,\
                            'd','_','m','.','t','x','t','=','"','0','0','"',255,255,255,\
                            'd','_','d','.','t','x','t','=','"','0','0','"',255,255,255,\
                            'd','_','y','.','t','x','t','=','"','0','0','"',255,255,255};

/*--------------------------------------------------------------------------------*/

void TIM8_TRG_COM_TIM14_IRQHandler(void);
                                
void Init_RCC(void);
void SysTick_Handler(void);
void Init_GPIO(void);
void Init_RTC(void);
void Init_TFT(void);
void TFT_send(uint8_t *buff, uint8_t size);
void Init_KEYs_TIM(void);
                            
void Init_I2C1(void);
uint8_t I2C1_Send(uint8_t addres,uint8_t *buff, uint16_t size);
uint8_t RDA_set_freq(uint16_t freq);
                            
uint8_t Init_TDA(void);
