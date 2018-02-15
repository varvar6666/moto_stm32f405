#include "stm32f405xx.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_rtc.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_tim.h"
#include "string.h"

#define F_CPU       168000000UL
#define AHB1        F_CPU
#define APB1        F_CPU/4
#define APB1_TIM    APB1*2
#define APB2        F_CPU/2
#define APB2_TIM    APB2*2
#define SysTicksClk 10000
#define SysTicks    F_CPU/SysTicksClk

#define	MEM_ADDRESS 		0x0800C000
#define RADIO_FREQ_ADR	0x0800C004

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
    SET_DATE_WEEK_DAY,
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
uint8_t TFT_reset[7] = {'r','e','s','t',255,255,255};

uint8_t loading_txt[13] = {'l','o','d','.','v','a','l','=','0','0',255,255,255};

uint8_t TFT_TIME[56] = {'t','i','m','e','.','t','x','t','=','"',0,0,':',0,0,'"',255,255,255,\
                        'd','a','t','e','.','t','x','t','=','"',' ',' ',0,0,'.',0,0,'.',0,0,' ',\
												' ',' ',' ','d','a','y','o','f','w','e','e','k','"',255,255,255}; 

uint8_t pages[4][12] = {{'p','a','g','e',' ','l','o','a','d',255,255,255},
                        {'p','a','g','e',' ','a','o','f','f',255,255,255},
                        {'p','a','g','e',' ','m','a','i','n',255,255,255},
                        {'p','a','g','e',' ','s','e','t','t',255,255,255}};

uint8_t input_tft[4][24] = {{'A','U','X','.','v','a','l','=','0',255,255,255,'F','M','I','.','v','a','l','=','1',255,255,255},
                            {'F','M','I','.','v','a','l','=','0',255,255,255,'B','T','I','.','v','a','l','=','1',255,255,255},
                            {'B','T','I','.','v','a','l','=','0',255,255,255,'U','S','B','.','v','a','l','=','1',255,255,255},
                            {'U','S','B','.','v','a','l','=','0',255,255,255,'A','U','X','.','v','a','l','=','1',255,255,255}};
                                
uint8_t set_time_tft[7][24] = {{'s','e','t','.','v','a','l','=','0',255,255,255,'t','_','h','.','v','a','l','=','1',255,255,255},
                               {'t','_','h','.','v','a','l','=','0',255,255,255,'t','_','m','.','v','a','l','=','1',255,255,255},
                               {'t','_','m','.','v','a','l','=','0',255,255,255,'d','_','y','.','v','a','l','=','1',255,255,255},
                               {'d','_','y','.','v','a','l','=','0',255,255,255,'d','_','m','.','v','a','l','=','1',255,255,255},
                               {'d','_','m','.','v','a','l','=','0',255,255,255,'d','_','d','.','v','a','l','=','1',255,255,255},
                               {'d','_','d','.','v','a','l','=','0',255,255,255,'d','_','w','.','v','a','l','=','1',255,255,255},
                               {'d','_','w','.','v','a','l','=','0',255,255,255,'s','e','t','.','v','a','l','=','1',255,255,255}};

uint8_t set_time_txt[97] = {'t','_','h','.','t','x','t','=','"','0','0','"',255,255,255,\
                            't','_','m','.','t','x','t','=','"','0','0','"',255,255,255,\
                            'd','_','m','.','t','x','t','=','"','0','0','"',255,255,255,\
                            'd','_','d','.','t','x','t','=','"','0','0','"',255,255,255,\
                            'd','_','y','.','t','x','t','=','"','0','0','"',255,255,255,\
                            'd','_','w','.','t','x','t','=','"','0','0','0','0','0','0','0','0','0','"',255,255,255,};

uint8_t day_of_week[7][9] = {"Monday   ",
                             "Tuesday  ",
                             "Wednesday",
                             "Thursday ",
                             "Friday   ",
                             "Saturday ",
                             "Sunday   "};

uint8_t main_FM_text[22] = 		 {'t','e','x','t','.','t','x','t','=','"','0','0','0','.','0',' ','F','M','"',255,255,255};
uint8_t main_BT_text[4][22] = {{'t','e','x','t','.','t','x','t','=','"','N','O',' ','C','O','N','N',' ','"',255,255,255},
															 {'t','e','x','t','.','t','x','t','=','"','C','O','N','N','E','C','T',' ','"',255,255,255},
															 {'t','e','x','t','.','t','x','t','=','"',' ',' ','P','L','A','Y',' ',' ','"',255,255,255},
															 {'t','e','x','t','.','t','x','t','=','"',' ',' ','P','A','U','S','E',' ','"',255,255,255}};
uint8_t main_DF_text[2][32] = {{'t','e','x','t','.','t','x','t','=','"','s','o','n','g',' ','0','0','0','/','0','0','0',' ','P','A','U','S','E','"',255,255,255},
															 {'t','e','x','t','.','t','x','t','=','"','s','o','n','g',' ','0','0','0','/','0','0','0',' ','P','L','A','Y',' ','"',255,255,255}};
																 
/*--------------------------------------------------------------------------------*/
// BT
#define BT_RX_BUFF_SIZE 20

enum BT_querys
{
    BT_STATUS=0,
    BT_PLAY_PAUSE,
    BT_FORWARD,
    BT_BACKWARD,
		BT_DISC,
		BT_RESET
};
uint8_t bt_tx_query[6][7] = {{'A','T','#','M','V',13,10},
														 {'A','T','#','M','A',13,10},
														 {'A','T','#','M','D',13,10},
														 {'A','T','#','M','E',13,10},
														 {'A','T','#','M','J',13,10},
														 {'A','T','#','C','Z',13,10}};											 
/*--------------------------------------------------------------------------------*/
//DF Serial Control CMD
#define DF_NEXT    0x01
#define DF_PREW    0x02
#define DF_TRACK   0x03
#define DF_INC_VOL 0x04
#define DF_DEC_VOL 0x05
#define DF_SET_VOL 0x06
#define DF_EQ      0x07
#define DF_PB_MODE 0x08
#define DF_PB_SORC 0x09
#define DF_STBY    0x0A
#define DF_NORM    0x0B
#define DF_RES     0x0C
#define DF_PLAY    0x0D
#define DF_PAUSE   0x0E
#define DF_FOLDER  0x0F
#define DF_VOL_ADJ 0x10
#define DF_RP_PL   0x11

//DF Serial Query Cmd
#define DF_Q_NUM_FILES	0x47
#define DF_Q_CUR_FIL		0x4C
#define DF_Q_CUR_STAT		0x42

//Play Mode
#define DF_REP_ALL 0
#define DF_REP_FLD 0
#define DF_REP_SIN 0
#define DF_RANDOM  0

//Play Source
#define DF_USB     1

uint8_t DF_data[10] = {0x7E, 0xFF, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEF};

/*--------------------------------------------------------------------------------*/
void TIM8_TRG_COM_TIM14_IRQHandler(void);
void DMA1_Stream1_IRQHandler(void);
void DMA1_Stream0_IRQHandler(void);
void USART3_IRQHandler(void);	
                                
void Init_RCC(void);
void SysTick_Handler(void);

uint32_t flash_read(uint32_t address);
uint8_t flash_ready(void);
void flash_erase_sector(uint8_t sector);
void flash_write(uint32_t address, uint32_t data);
void flash_write_newdata(void);
	
void Init_GPIO(void);
void Init_RTC(void);
void Init_TFT(void);
void TFT_send(uint8_t *buff, uint8_t size);
void Init_KEYs_TIM(void);

void Init_BT(void);
void BT_send(uint8_t query);

void Init_DF(void);
void DF_send(uint8_t CMD, uint8_t PAR);

void Init_I2C1(void);
uint8_t I2C1_Send(uint8_t addres,uint8_t *buff, uint16_t size);
uint8_t RDA_set_freq(uint16_t freq);
                            
uint8_t Init_TDA(void);
	
