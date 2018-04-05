#include "main.h"

uint8_t STATE = AUDIO_OFF;
uint8_t SET_TIME_STATE = SET_TIME_HOUR;
uint8_t INPUT_SEL = FM;
uint8_t prev_state;

uint8_t TDA_SET_STATE = TDA_SET_LOUDNESS;
int8_t TDA_SET_LOUD_STATE = TDA_SET_LOUD_CENT_FREQ;
int8_t TDA_SET_TREB_STATE = TDA_SET_TREB_CENT_FREQ;
int8_t TDA_SET_MIDD_STATE = TDA_SET_MIDD_CENT_FREQ;
int8_t TDA_SET_BASS_STATE = TDA_SET_BASS_CENT_FREQ;
int8_t TDA_SET_SATT_STATE = TDA_SET_SATT_L_F;

uint8_t OFF_counter = 0;

uint8_t I2C_res, USB_res;

uint16_t RADIO_FREQ = 1040;

uint8_t BT_Status = BT_NO_DEV;
uint8_t bt_rx_buff[BT_RX_BUFF_SIZE];

uint8_t usb_rx_buff[11];
uint8_t usb_status = USB_PAUSE;
uint8_t usb_play_mode  = 0;

struct TRACK_INFO
{
    uint16_t count;
    uint16_t num;
    uint16_t time;
    uint16_t tlong;
    uint8_t  name[11];
} usb_track_info;

int8_t VOLUME = 15;

uint8_t MUTED = 0;

uint16_t ADC_Buff[ADC_BUF_NUM];

struct TDA_LOUDNESS
{
    uint8_t atteniation;
    uint8_t center_freq;
    uint8_t high_boost;
} TDA_loudness;

struct TDA_TREBLE
{
    int8_t atteniation;
    uint8_t center_freq;
} TDA_treble;

struct TDA_MIDDLE
{
    int8_t atteniation;
    uint8_t center_freq;
    uint8_t Q_factot;
} TDA_middle;

struct TDA_BASS
{
    int8_t atteniation;
    uint8_t center_freq;
    uint8_t Q_factot;
} TDA_bass;

struct TDA_SP_ATT
{
    int8_t left_front;
    int8_t right_front;
    int8_t left_rear;
    int8_t right_rear;
} TDA_sp_att;


int main(void)
{
        
    /*-- Init main clock --*/
    Init_RCC();
    /*-- Init main clock --*/
    Init_GPIO();
 
    AMP_OFF;
    BT_OFF;
    BULB_CH_ON;
    
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;

    Init_TFT();
    TFT_send(TFT_reset,sizeof(TFT_reset)); //RESET TFT
   
    //-- Delay for TFT start --
    for(uint32_t delay = 0;delay < 10000000;delay++){};

    loading_txt[8] = '1';
    loading_txt[9] = '4';
    TFT_send(loading_txt, sizeof(loading_txt));

    Init_ADC();         //1
        
    loading_txt[8] = '2';
    loading_txt[9] = '8';
    TFT_send(loading_txt, sizeof(loading_txt));
        
    Init_RTC();         //2
        
    loading_txt[8] = '4';
    loading_txt[9] = '2';
    TFT_send(loading_txt, sizeof(loading_txt));
    
    Init_KEYs_TIM();    //3
        
    loading_txt[8] = '5';
    loading_txt[9] = '6';
    TFT_send(loading_txt, sizeof(loading_txt));
    
    Init_I2C1();        //4
    
    loading_txt[8] = '7';
    loading_txt[9] = '0';
    TFT_send(loading_txt, sizeof(loading_txt));
        
	Init_BT();          //5
    
    loading_txt[8] = '8';
    loading_txt[9] = '4';
    TFT_send(loading_txt, sizeof(loading_txt));

    Init_USB();         //6->

    USB_res = USB_send_par(USB_CMD_SOURCE, 0x00);
    if(USB_res == 0)
    {
        USB_res = USB_send_par(USB_CMD_VOL, 0x1e);
        USB_res = USB_send_par(USB_CMD_PLAY_MODE, 0x00);
    }                   //<-6
    
    loading_txt[8] = '9';
    loading_txt[9] = '8';
    TFT_send(loading_txt, sizeof(loading_txt));

    uint32_t tmp_mem;
    //STATE = AUDIO_OFF;
    tmp_mem = flash_read(MEM_ADDRESS);
    if(tmp_mem != 0xFFFFFFFF)
    {
        STATE = (0xFF000000 & tmp_mem) >> 24;
        INPUT_SEL = (0xFF0000 & flash_read(MEM_ADDRESS)) >> 16;
        VOLUME = (0xFF00 & flash_read(MEM_ADDRESS)) >> 8;
        
        tmp_mem = flash_read(RADIO_FREQ_ADR);
        RADIO_FREQ = (0xFFFF0000 & tmp_mem) >> 16;
        
        tmp_mem = flash_read(TDA_MAIN_LOUD_ADR);
        TDA_loudness.high_boost  = (0xFF000000 & tmp_mem) >> 24;
        TDA_loudness.center_freq = (0xFF0000   & tmp_mem) >> 16;
        TDA_loudness.atteniation = (0xFF00     & tmp_mem) >> 8;
        
        tmp_mem = flash_read(TDA_TREB_ADR);
        TDA_treble.center_freq = (0xFF000000 & tmp_mem) >> 24;
        TDA_treble.atteniation = (0xFF0000   & tmp_mem) >> 16;
        
        tmp_mem = flash_read(TDA_MIDD_ADR);
        TDA_middle.Q_factot    = (0xFF000000 & tmp_mem) >> 24;
        TDA_middle.center_freq = (0xFF0000   & tmp_mem) >> 16;
        TDA_middle.atteniation = (0xFF00     & tmp_mem) >> 8;
        
        tmp_mem = flash_read(TDA_BASS_ADR);
        TDA_bass.Q_factot    = (0xFF000000 & tmp_mem) >> 24;
        TDA_bass.center_freq = (0xFF0000   & tmp_mem) >> 16;
        TDA_bass.atteniation = (0xFF00     & tmp_mem) >> 8;
        
        tmp_mem = flash_read(TDA_SATT_ADR);
        TDA_sp_att.left_front  = (0xFF000000 & tmp_mem) >> 24;
        TDA_sp_att.right_front = (0xFF0000   & tmp_mem) >> 16;
        TDA_sp_att.left_rear   = (0xFF00     & tmp_mem) >> 8;
        TDA_sp_att.right_rear  = (0xFF       & tmp_mem);
    }

    TFT_send(pages[STATE], sizeof(pages[STATE]));
    TFT_send(tft_apage, sizeof(tft_apage));
    
    I2C_res = Init_TDA();
    
    I2C_res = TEA_set_freq(RADIO_FREQ); // Init FM
    
    if(STATE == MAIN)
    {
            TFT_send(input_tft[INPUT_SEL], sizeof(input_tft[INPUT_SEL]));
        
            main_VOL_text[9] =   VOLUME > 0 ? '+' : '-';
            main_VOL_text[10] = (VOLUME > 0 ? VOLUME/10 : -VOLUME/10) + 0x30;
            main_VOL_text[11] = (VOLUME > 0 ? (VOLUME-(VOLUME/10)*10) : (-VOLUME-(-VOLUME/10)*10)) + 0x30;
            TFT_send(main_VOL_text, sizeof(main_VOL_text));
      
            switch (INPUT_SEL)
            {
                case FM:{
                        main_FM_text[10] = RADIO_FREQ/1000 + 0x30;
                        main_FM_text[11] = (RADIO_FREQ -(RADIO_FREQ/1000)*1000)/100 + 0x30;
                        main_FM_text[12] = (RADIO_FREQ -(RADIO_FREQ/100)*100)/10 + 0x30;
                        main_FM_text[14] = (RADIO_FREQ -(RADIO_FREQ/10)*10) + 0x30;
                        
                        TFT_send(main_text_font_7, sizeof(main_text_font_7));
                        
                        TFT_send(main_FM_text, sizeof(main_FM_text));
                    break;}
                case BT:{
                        TFT_send(main_text_font_7, sizeof(main_text_font_7));
                        
                        BT_send(BT_STATUS);
                    break;}
                case USB:{
                        if(USB_res == 1)
                            TFT_send(main_NO_USB_text, sizeof(main_NO_USB_text));
                        else
                        {
                            USB_send(USB_Q_TRACK_COUNT);
                            USB_send(USB_Q_TRACK_NUMBER);
                            USB_send(USB_Q_TRACK_NAME);
                        
                            main_USB_text[14] =  usb_track_info.count/100 + 0x30;
                            main_USB_text[15] = (usb_track_info.count -(usb_track_info.count/100)*100)/10 + 0x30;
                            main_USB_text[16] = (usb_track_info.count -(usb_track_info.count/10)*10) + 0x30;
                        
                            main_USB_text[10] =  usb_track_info.num/100 + 0x30;
                            main_USB_text[11] = (usb_track_info.num -(usb_track_info.num/100)*100)/10 + 0x30;
                            main_USB_text[12] = (usb_track_info.num -(usb_track_info.num/10)*10) + 0x30;
    /*                  
                            main_USB_text[35] =  (usb_track_info.tlong/60)/10 + 0x30;
                            main_USB_text[36] = ((usb_track_info.tlong/60) -((usb_track_info.tlong/60)/10)*10) + 0x30;
                            main_USB_text[38] =  (usb_track_info.tlong-((usb_track_info.tlong/60)*60))/10 + 0x30;
                            main_USB_text[39] = ((usb_track_info.tlong-((usb_track_info.tlong/60)*60)) -((usb_track_info.tlong-((usb_track_info.tlong/60)*60))/10)*10) + 0x30;
                        
                            main_USB_text[25] =  (usb_track_info.time/60)/10 + 0x30;
                            main_USB_text[26] = ((usb_track_info.time/60) -((usb_track_info.time/60)/10)*10) + 0x30;
                            main_USB_text[28] =  (usb_track_info.time-((usb_track_info.time/60)*60))/10 + 0x30;
                            main_USB_text[29] = ((usb_track_info.time-((usb_track_info.time/60)*60)) -((usb_track_info.time-((usb_track_info.time/60)*60))/10)*10) + 0x30;
    */                        
                            main_USB_text[18] = usb_track_info.name[0];
                            main_USB_text[19] = usb_track_info.name[1];
                            main_USB_text[20] = usb_track_info.name[2];
                            main_USB_text[21] = usb_track_info.name[3];
                            main_USB_text[22] = usb_track_info.name[4];
                            main_USB_text[23] = usb_track_info.name[5];
                            
                            TFT_send(main_text_font_5, sizeof(main_text_font_5));
                            TFT_send(main_USB_text, sizeof(main_USB_text));
                        }
                        break;}
                case AUX:{
                        TFT_send(main_text_font_7, sizeof(main_text_font_7));
                    
                        TFT_send(main_AUX_text, sizeof(main_AUX_text));
                    break;}				
            };
            AMP_ON;
    }

    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
    
    GPIOB->BSRR |= GPIO_BSRR_BS2;
    Init_IWDG();
    BULB_CH_OFF;
    
    while(1)
    {
    
    }
}

void Init_RCC(void)
{
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
    
    RCC->CR |= RCC_CR_HSEON;

    do
    {
        HSEStatus = RCC->CR & RCC_CR_HSERDY;
        StartUpCounter++;
    }    
    while((HSEStatus == 0) && (StartUpCounter != ((uint32_t)1000U)));
    
    if( (RCC->CR & RCC_CR_HSIRDY) != RESET)
    {
        /* Включаем буфер предвыборки FLASH */
        FLASH->ACR |= FLASH_ACR_PRFTEN;

        /* Конфигурируем Flash на 2 цикла ожидания */
    	/* Это нужно потому, что Flash не может работать на высокой частоте */        
        FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
        FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_5WS;   
        
        /* HCLK = SYSCLK || AHB prescaler*/
        RCC->CFGR |= RCC_CFGR_HPRE_DIV1; //AHB clk = 100MHz
        
    	/* PCLK1 = HCLK || APB Low speed prescaler (APB1)*/
		RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV4;

        /* PCLK2 = HCLK || APB high-speed prescaler (APB2)*/
		RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV2;
        
        /* Set PLL input sourse*/
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
        
        /*Set PLL M prescaler */
        RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_Msk;
        RCC->PLLCFGR |= (8 << RCC_PLLCFGR_PLLM_Pos); //8
        
        /*Set PLL N prescaler */
        RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_Msk;
        RCC->PLLCFGR |= (168 << RCC_PLLCFGR_PLLN_Pos);
        
        /*Set PLL P prescaler */
        //RCC->PLLCFGR |= RCC_PLLCFGR_PLLP;
        
        RCC->CR |= RCC_CR_PLLON;
        
        while ((RCC->CR & RCC_CR_PLLRDY) == 0)
        {}
            
        /*Set SYSCLK as PLL */
        RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
        RCC->CFGR |= RCC_CFGR_SW_PLL;
            
        while ((RCC->CFGR & RCC_CFGR_SWS) !=  RCC_CFGR_SWS_PLL)
        {}
    }
    SysTick_Config(SysTicks);
}

void SysTick_Handler(void)
{
    static uint32_t del = 0;
    
    del++;

    if (del == (SysTicksClk/2)) // Per = 1s
    {
        del = 0;
//        if (GPIOB->ODR & GPIO_ODR_OD2)
//        {
//            GPIOB->BSRR |= GPIO_BSRR_BR2;
//        }
//        else
//        {
//            GPIOB->BSRR |= GPIO_BSRR_BS2;
//        }  			
    }
}

void TIM8_TRG_COM_TIM14_IRQHandler(void) // parce buttons
{
    TIM14->SR = 0;
	
    static uint8_t hour;
    static uint8_t min;
    static uint8_t mon;
    static uint8_t day;
    static uint8_t week_day;
    static uint8_t year;
	
    static uint8_t delay_send = 98;
    
    uint8_t i2c_buff[2];// = {TDA_MAIN_SOURCE, TDA_SOURCE_MUTE};
    
    IWDG_res();

    delay_send++;

    GPIOB->BSRR |= GPIO_BSRR_BR12;
    GPIOB->BSRR |= GPIO_BSRR_BS15;
    
    if(BT_SOUCE) // source select & OFF audio
    {
        //GPIOB->BSRR |= GPIO_BSRR_BS12;
        OFF_counter++;
        if((OFF_counter >= 20)&&(STATE == MAIN))
        {
            OFF_counter = 20;
            i2c_buff[0] = TDA_MAIN_SOURCE;
            i2c_buff[1] = TDA_SOURCE_MUTE;
            I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
            
            STATE = AUDIO_OFF;
						
			flash_write_newdata();
            
            TFT_send(pages[STATE], sizeof(pages[STATE]));
			delay_send = 98;
					
			BT_send(BT_DISC);
            USB_send(USB_CMD_PAUSE);
            AMP_OFF;
        }
    }
    else
    {
        if((OFF_counter < 20)&&(OFF_counter != 0))
        {
            if(STATE == MAIN)
            {
                INPUT_SEL++;

                if(INPUT_SEL>AUX) INPUT_SEL = FM;
							
                flash_write_newdata();
                
                i2c_buff[0] = TDA_MAIN_SOURCE;
                i2c_buff[1] = TDA_inputs[INPUT_SEL];
                I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                
                MUTED = 0;
                TFT_send(main_VOL_text, sizeof(main_VOL_text));                

                TFT_send(input_tft[INPUT_SEL], sizeof(input_tft[INPUT_SEL]));
                            
                switch (INPUT_SEL)
                {
                    case FM:{
                            TFT_send(main_text_font_7, sizeof(main_text_font_7));
                            
                            main_FM_text[10] = RADIO_FREQ/1000 + 0x30;
                            main_FM_text[11] = (RADIO_FREQ -(RADIO_FREQ/1000)*1000)/100 + 0x30;
                            main_FM_text[12] = (RADIO_FREQ -(RADIO_FREQ/100)*100)/10 + 0x30;
                            main_FM_text[14] = (RADIO_FREQ -(RADIO_FREQ/10)*10) + 0x30;
                            
                            TFT_send(main_FM_text, sizeof(main_FM_text));
                        break;}
                    case BT:{
                            TFT_send(main_text_font_7, sizeof(main_text_font_7));
                            
                            BT_send(BT_STATUS);
                            
                            TFT_send(main_BT_text[BT_Status], sizeof(main_BT_text[BT_Status]));
                        break;}
                    case USB:{
                        USB_res = USB_send_par(USB_CMD_SOURCE, 0x00);
                        if(USB_res == 1)
                        {
                            TFT_send(main_NO_USB_text, sizeof(main_NO_USB_text));
                        }
                        else
                        {
                            TFT_send(main_text_font_5, sizeof(main_text_font_5));

                            USB_send_par(USB_CMD_VOL, 0x1e);
                            USB_send_par(USB_CMD_PLAY_MODE, 0x00);                      
                            USB_send(USB_Q_TRACK_COUNT);
                            USB_send(USB_Q_TRACK_NUMBER);
                            USB_send(USB_Q_TRACK_NAME);
                        
                            main_USB_text[14] =  usb_track_info.count/100 + 0x30;
                            main_USB_text[15] = (usb_track_info.count -(usb_track_info.count/100)*100)/10 + 0x30;
                            main_USB_text[16] = (usb_track_info.count -(usb_track_info.count/10)*10) + 0x30;
                        
                            main_USB_text[10] =  usb_track_info.num/100 + 0x30;
                            main_USB_text[11] = (usb_track_info.num -(usb_track_info.num/100)*100)/10 + 0x30;
                            main_USB_text[12] = (usb_track_info.num -(usb_track_info.num/10)*10) + 0x30;
    /*                    
                            main_USB_text[35] =  (usb_track_info.tlong/60)/10 + 0x30;
                            main_USB_text[36] = ((usb_track_info.tlong/60) -((usb_track_info.tlong/60)/10)*10) + 0x30;
                            main_USB_text[38] =  (usb_track_info.tlong-((usb_track_info.tlong/60)*60))/10 + 0x30;
                            main_USB_text[39] = ((usb_track_info.tlong-((usb_track_info.tlong/60)*60)) -((usb_track_info.tlong-((usb_track_info.tlong/60)*60))/10)*10) + 0x30;
                        
                            main_USB_text[25] =  (usb_track_info.time/60)/10 + 0x30;
                            main_USB_text[26] = ((usb_track_info.time/60) -((usb_track_info.time/60)/10)*10) + 0x30;
                            main_USB_text[28] =  (usb_track_info.time-((usb_track_info.time/60)*60))/10 + 0x30;
                            main_USB_text[29] = ((usb_track_info.time-((usb_track_info.time/60)*60)) -((usb_track_info.time-((usb_track_info.time/60)*60))/10)*10) + 0x30;
    */                        
                            main_USB_text[18] = usb_track_info.name[0];
                            main_USB_text[19] = usb_track_info.name[1];
                            main_USB_text[20] = usb_track_info.name[2];
                            main_USB_text[21] = usb_track_info.name[3];
                            main_USB_text[22] = usb_track_info.name[4];
                            main_USB_text[23] = usb_track_info.name[5];
                            
                            main_USB_text[30] = 'P';
                            main_USB_text[31] = 'A';
                            main_USB_text[32] = 'U';
                            main_USB_text[33] = 'S';
                            main_USB_text[34] = 'E';
                        
                            TFT_send(main_USB_text, sizeof(main_USB_text));
                        }
                        break;}
                    case AUX:{
                            TFT_send(main_text_font_7, sizeof(main_text_font_7));
                        
                            TFT_send(main_AUX_text, sizeof(main_AUX_text));
                        break;}
                };
            }
            if(STATE == AUDIO_OFF)
            {
                STATE = MAIN;
            
				flash_write_newdata();
            
                TFT_send(pages[STATE], sizeof(pages[STATE]));

                i2c_buff[0] = TDA_MAIN_SOURCE;
                i2c_buff[1] = TDA_inputs[INPUT_SEL];
                
                I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                
                MUTED = 0;
                TFT_send(main_VOL_text, sizeof(main_VOL_text));                
                
                TFT_send(input_tft[INPUT_SEL], sizeof(input_tft[INPUT_SEL]));
                switch (INPUT_SEL)
                {
                    case FM:{
                            TFT_send(main_text_font_7, sizeof(main_text_font_7));
                        
                            main_FM_text[10] = RADIO_FREQ/1000 + 0x30;
                            main_FM_text[11] = (RADIO_FREQ -(RADIO_FREQ/1000)*1000)/100 + 0x30;
                            main_FM_text[12] = (RADIO_FREQ -(RADIO_FREQ/100)*100)/10 + 0x30;
                            main_FM_text[14] = (RADIO_FREQ -(RADIO_FREQ/10)*10) + 0x30;
                            
                            TFT_send(main_FM_text, sizeof(main_FM_text));
                        break;}
                    case BT:{
                            TFT_send(main_text_font_7, sizeof(main_text_font_7));
                        
                            BT_send(BT_STATUS);
                        
                            TFT_send(main_BT_text[BT_Status], sizeof(main_BT_text[BT_Status]));
                        break;}
                    case USB:{
                        USB_res = USB_send_par(USB_CMD_SOURCE, 0x00);
                        if(USB_res == 1)
                        {
                            TFT_send(main_NO_USB_text, sizeof(main_NO_USB_text));
                        }
                        else
                        {
                            TFT_send(main_text_font_5, sizeof(main_text_font_5));
                            
                            USB_send_par(USB_CMD_VOL, 0x1e);
                            USB_send_par(USB_CMD_PLAY_MODE, 0x00);  
                            USB_send(USB_Q_TRACK_COUNT);
                            USB_send(USB_Q_TRACK_NUMBER);
                            USB_send(USB_Q_TRACK_NAME);
                        
                            main_USB_text[14] =  usb_track_info.count/100 + 0x30;
                            main_USB_text[15] = (usb_track_info.count -(usb_track_info.count/100)*100)/10 + 0x30;
                            main_USB_text[16] = (usb_track_info.count -(usb_track_info.count/10)*10) + 0x30;
                        
                            main_USB_text[10] =  usb_track_info.num/100 + 0x30;
                            main_USB_text[11] = (usb_track_info.num -(usb_track_info.num/100)*100)/10 + 0x30;
                            main_USB_text[12] = (usb_track_info.num -(usb_track_info.num/10)*10) + 0x30;
    /*                    
                            main_USB_text[35] =  (usb_track_info.tlong/60)/10 + 0x30;
                            main_USB_text[36] = ((usb_track_info.tlong/60) -((usb_track_info.tlong/60)/10)*10) + 0x30;
                            main_USB_text[38] =  (usb_track_info.tlong-((usb_track_info.tlong/60)*60))/10 + 0x30;
                            main_USB_text[39] = ((usb_track_info.tlong-((usb_track_info.tlong/60)*60)) -((usb_track_info.tlong-((usb_track_info.tlong/60)*60))/10)*10) + 0x30;
                        
                            main_USB_text[25] =  (usb_track_info.time/60)/10 + 0x30;
                            main_USB_text[26] = ((usb_track_info.time/60) -((usb_track_info.time/60)/10)*10) + 0x30;
                            main_USB_text[28] =  (usb_track_info.time-((usb_track_info.time/60)*60))/10 + 0x30;
                            main_USB_text[29] = ((usb_track_info.time-((usb_track_info.time/60)*60)) -((usb_track_info.time-((usb_track_info.time/60)*60))/10)*10) + 0x30;
    */                        
                            main_USB_text[18] = usb_track_info.name[0];
                            main_USB_text[19] = usb_track_info.name[1];
                            main_USB_text[20] = usb_track_info.name[2];
                            main_USB_text[21] = usb_track_info.name[3];
                            main_USB_text[22] = usb_track_info.name[4];
                            main_USB_text[23] = usb_track_info.name[5];
                            
                            main_USB_text[30] = 'P';
                            main_USB_text[31] = 'A';
                            main_USB_text[32] = 'U';
                            main_USB_text[33] = 'S';
                            main_USB_text[34] = 'E';
                        
                            TFT_send(main_USB_text, sizeof(main_USB_text));
                        }
                        break;}
                    case AUX:{
                            TFT_send(main_text_font_7, sizeof(main_text_font_7));
                        
                            TFT_send(main_AUX_text, sizeof(main_AUX_text));
                        break;}
                };

                main_VOL_text[9] =   VOLUME > 0 ? '+' : '-';
                main_VOL_text[10] = (VOLUME > 0 ? VOLUME/10 : -VOLUME/10) + 0x30;
                main_VOL_text[11] = (VOLUME > 0 ? (VOLUME-(VOLUME/10)*10) : (-VOLUME-(-VOLUME/10)*10)) + 0x30;
                
                TFT_send(main_VOL_text, sizeof(main_VOL_text));
                AMP_ON;
                delay_send = 98;
            }
        }
        OFF_counter = 0;
    }
    
    
    if(BT_NEXT) // increase
    {
        //GPIOB->BSRR |= GPIO_BSRR_BS12;
        if(STATE == MAIN)
        {				
            switch (INPUT_SEL)
            {
                case FM:{
                        RADIO_FREQ++;
                        if(RADIO_FREQ >= 1080)RADIO_FREQ = 850;
                        TEA_set_freq(RADIO_FREQ);
                        
                        flash_write_newdata();
        
                        main_FM_text[10] = RADIO_FREQ/1000 + 0x30;
                        main_FM_text[11] = (RADIO_FREQ -(RADIO_FREQ/1000)*1000)/100 + 0x30;
                        main_FM_text[12] = (RADIO_FREQ -(RADIO_FREQ/100)*100)/10 + 0x30;
                        main_FM_text[14] = (RADIO_FREQ -(RADIO_FREQ/10)*10) + 0x30;
                        
                        TFT_send(main_FM_text, sizeof(main_FM_text));
                        break;}
                case BT:{
                        BT_send(BT_FORWARD);
                    break;}
                case USB:{
                        usb_status = USB_PLAY;
                    
                        USB_send(USB_CMD_NEXT);
                        
                        USB_send(USB_Q_TRACK_NUMBER);
                        USB_send(USB_Q_TRACK_NAME);
                        USB_send(USB_Q_STATUS);
                            
                        main_USB_text[10] =  usb_track_info.num/100 + 0x30;
                        main_USB_text[11] = (usb_track_info.num -(usb_track_info.num/100)*100)/10 + 0x30;
                        main_USB_text[12] = (usb_track_info.num -(usb_track_info.num/10)*10) + 0x30;
/*                    
                        main_USB_text[35] =  (usb_track_info.tlong/60)/10 + 0x30;
                        main_USB_text[36] = ((usb_track_info.tlong/60) -((usb_track_info.tlong/60)/10)*10) + 0x30;
                        main_USB_text[38] =  (usb_track_info.tlong-((usb_track_info.tlong/60)*60))/10 + 0x30;
                        main_USB_text[39] = ((usb_track_info.tlong-((usb_track_info.tlong/60)*60)) -((usb_track_info.tlong-((usb_track_info.tlong/60)*60))/10)*10) + 0x30;
                    
                        main_USB_text[25] =  (usb_track_info.time/60)/10 + 0x30;
                        main_USB_text[26] = ((usb_track_info.time/60) -((usb_track_info.time/60)/10)*10) + 0x30;
                        main_USB_text[28] =  (usb_track_info.time-((usb_track_info.time/60)*60))/10 + 0x30;
                        main_USB_text[29] = ((usb_track_info.time-((usb_track_info.time/60)*60)) -((usb_track_info.time-((usb_track_info.time/60)*60))/10)*10) + 0x30;
 */                       
                        main_USB_text[18] = usb_track_info.name[0];
                        main_USB_text[19] = usb_track_info.name[1];
                        main_USB_text[20] = usb_track_info.name[2];
                        main_USB_text[21] = usb_track_info.name[3];
                        main_USB_text[22] = usb_track_info.name[4];
                        main_USB_text[23] = usb_track_info.name[5];
                        
                        if(usb_status == 1)
                        {
                            main_USB_text[30] = 'P';
                            main_USB_text[31] = 'L';
                            main_USB_text[32] = 'A';
                            main_USB_text[33] = 'Y';
                            main_USB_text[34] = ' ';
                        }else if(usb_status == 2)
                        {
                            main_USB_text[30] = 'P';
                            main_USB_text[31] = 'A';
                            main_USB_text[32] = 'U';
                            main_USB_text[33] = 'S';
                            main_USB_text[34] = 'E';
                        }
                    
                        TFT_send(main_USB_text, sizeof(main_USB_text));
                    break;}
            
            }
        }
        
        if(STATE == TDA_SETT)
        {
            switch(TDA_SET_STATE)
            {
                case TDA_SET_LOUDNESS:
                {
                    TDA_SET_LOUD_STATE++;
                    if(TDA_SET_LOUD_STATE > TDA_SET_LOUD_HIGN_BOOST)
                        TDA_SET_LOUD_STATE = TDA_SET_LOUD_HIGN_BOOST;
                    TFT_send(tda_set_LTMB[TDA_SET_LOUD_STATE], sizeof(tda_set_LTMB[TDA_SET_LOUD_STATE]));
                    break;
                }
                case TDA_SET_TREB:
                {
                    TDA_SET_TREB_STATE = TDA_SET_TREB_ATTENUATION;
                    TFT_send(tda_set_LTMB[TDA_SET_TREB_STATE], sizeof(tda_set_LTMB[TDA_SET_TREB_STATE]));
                    break;
                }
                case TDA_SET_MIDD:
                {
                    TDA_SET_MIDD_STATE++;
                    if(TDA_SET_MIDD_STATE > TDA_SET_MIDD_Q_FACTOR)
                        TDA_SET_MIDD_STATE = TDA_SET_MIDD_Q_FACTOR;
                    TFT_send(tda_set_LTMB[TDA_SET_MIDD_STATE], sizeof(tda_set_LTMB[TDA_SET_MIDD_STATE]));
                    break;
                }
                case TDA_SET_BASS:
                {
                    TDA_SET_BASS_STATE++;
                    if(TDA_SET_BASS_STATE > TDA_SET_BASS_Q_FACTOR)
                        TDA_SET_BASS_STATE = TDA_SET_BASS_Q_FACTOR;
                    TFT_send(tda_set_LTMB[TDA_SET_BASS_STATE], sizeof(tda_set_LTMB[TDA_SET_BASS_STATE]));
                    break;
                }
                case TDA_SET_SPEAKER_ATT:
                {
                    TDA_SET_SATT_STATE++;
                    if(TDA_SET_SATT_STATE > TDA_SET_SATT_R_R)
                        TDA_SET_SATT_STATE = TDA_SET_SATT_L_F;
                    TFT_send(tda_set_satt[TDA_SET_SATT_STATE], sizeof(tda_set_satt[TDA_SET_SATT_STATE]));
                    break;
                }
            }
        }
    }

    if(BT_PP) // play/pause
    {
        //GPIOB->BSRR |= GPIO_BSRR_BS12;
        if(STATE == MAIN)
        {				
            switch (INPUT_SEL)
            {
                case BT:{
                        BT_send(BT_PLAY_PAUSE);
                    break;}
                case USB:{
                    if(usb_status == USB_PLAY)
                    {
                        USB_send(USB_CMD_PAUSE);
                        usb_status = USB_PAUSE;
                        
                        main_USB_text[30] = 'P';
                        main_USB_text[31] = 'A';
                        main_USB_text[32] = 'U';
                        main_USB_text[33] = 'S';
                        main_USB_text[34] = 'E';
                        
                        TFT_send(main_USB_text, sizeof(main_USB_text));
                    }
                    else
                    {
                        USB_send(USB_CMD_PLAY);
                        usb_status = USB_PLAY;
                        
                        main_USB_text[30] = 'P';
                        main_USB_text[31] = 'L';
                        main_USB_text[32] = 'A';
                        main_USB_text[33] = 'Y';
                        main_USB_text[34] = ' ';
                        
                        TFT_send(main_USB_text, sizeof(main_USB_text));
                    }
                    
                    break;}
                case FM:
                case AUX:{
                        if(MUTED)
                        {
                            MUTED = 0;
                            
                            i2c_buff[0] = TDA_MAIN_SOURCE;
                            i2c_buff[1] = TDA_inputs[INPUT_SEL];
                            I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                            
                            TFT_send(main_VOL_text, sizeof(main_VOL_text));
                        }
                        else
                        {
                            MUTED = 1;
                            
                            i2c_buff[0] = TDA_MAIN_SOURCE;
                            i2c_buff[1] = TDA_SOURCE_MUTE;
                            I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                            
                            TFT_send(main_VOL_mute, sizeof(main_VOL_mute));
                        }
                         }

            };
        }
    }
    
    if(BT_PREV) // decrease
    {
        //GPIOB->BSRR |= GPIO_BSRR_BS12;
        if(STATE == MAIN)
        {				
            switch (INPUT_SEL)
            {
                case FM:{
                        RADIO_FREQ--;
                        if(RADIO_FREQ <= 850)RADIO_FREQ = 1080;
                        TEA_set_freq(RADIO_FREQ);
                        
                        flash_write_newdata();
                        
                        main_FM_text[10] = RADIO_FREQ/1000 + 0x30;
                        main_FM_text[11] = (RADIO_FREQ -(RADIO_FREQ/1000)*1000)/100 + 0x30;
                        main_FM_text[12] = (RADIO_FREQ -(RADIO_FREQ/100)*100)/10 + 0x30;
                        main_FM_text[14] = (RADIO_FREQ -(RADIO_FREQ/10)*10) + 0x30;
                        
                        TFT_send(main_FM_text, sizeof(main_FM_text));	
                        break;}
                case BT:{
                        BT_send(BT_BACKWARD);
                    break;}
                case USB:{
                        usb_status = USB_PLAY;
                    
                        USB_send(USB_CMD_PREV);

                        USB_send(USB_Q_TRACK_NUMBER);
                        USB_send(USB_Q_TRACK_NAME);
                        USB_send(USB_Q_STATUS);
                            
                        main_USB_text[10] =  usb_track_info.num/100 + 0x30;
                        main_USB_text[11] = (usb_track_info.num -(usb_track_info.num/100)*100)/10 + 0x30;
                        main_USB_text[12] = (usb_track_info.num -(usb_track_info.num/10)*10) + 0x30;
                    
/*                        main_USB_text[35] =  (usb_track_info.tlong/60)/10 + 0x30;
                        main_USB_text[36] = ((usb_track_info.tlong/60) -((usb_track_info.tlong/60)/10)*10) + 0x30;
                        main_USB_text[38] =  (usb_track_info.tlong-((usb_track_info.tlong/60)*60))/10 + 0x30;
                        main_USB_text[39] = ((usb_track_info.tlong-((usb_track_info.tlong/60)*60)) -((usb_track_info.tlong-((usb_track_info.tlong/60)*60))/10)*10) + 0x30;
                    
                        main_USB_text[25] =  (usb_track_info.time/60)/10 + 0x30;
                        main_USB_text[26] = ((usb_track_info.time/60) -((usb_track_info.time/60)/10)*10) + 0x30;
                        main_USB_text[28] =  (usb_track_info.time-((usb_track_info.time/60)*60))/10 + 0x30;
                        main_USB_text[29] = ((usb_track_info.time-((usb_track_info.time/60)*60)) -((usb_track_info.time-((usb_track_info.time/60)*60))/10)*10) + 0x30;
*/
                        main_USB_text[18] = usb_track_info.name[0];
                        main_USB_text[19] = usb_track_info.name[1];
                        main_USB_text[20] = usb_track_info.name[2];
                        main_USB_text[21] = usb_track_info.name[3];
                        main_USB_text[22] = usb_track_info.name[4];
                        main_USB_text[23] = usb_track_info.name[5];
                        
                        if(usb_status == 1)
                        {
                            main_USB_text[30] = 'P';
                            main_USB_text[31] = 'L';
                            main_USB_text[32] = 'A';
                            main_USB_text[33] = 'Y';
                            main_USB_text[34] = ' ';
                        }else if(usb_status == 2)
                        {
                            main_USB_text[30] = 'P';
                            main_USB_text[31] = 'A';
                            main_USB_text[32] = 'U';
                            main_USB_text[33] = 'S';
                            main_USB_text[34] = 'E';
                        }
                    
                        TFT_send(main_USB_text, sizeof(main_USB_text));
                    break;}
            }
        }
        
        if(STATE == TDA_SETT)
        {
            switch(TDA_SET_STATE)
            {
                case TDA_SET_LOUDNESS:
                {
                    TDA_SET_LOUD_STATE--;
                    if(TDA_SET_LOUD_STATE < TDA_SET_LOUD_CENT_FREQ)
                        TDA_SET_LOUD_STATE = TDA_SET_LOUD_CENT_FREQ;
                    TFT_send(tda_set_LTMB[TDA_SET_LOUD_STATE], sizeof(tda_set_LTMB[TDA_SET_LOUD_STATE]));
                    break;
                }
                case TDA_SET_TREB:
                {
                    TDA_SET_TREB_STATE = TDA_SET_TREB_CENT_FREQ;
                    TFT_send(tda_set_LTMB[TDA_SET_TREB_STATE], sizeof(tda_set_LTMB[TDA_SET_TREB_STATE]));
                    break;
                }
                case TDA_SET_MIDD:
                {
                    TDA_SET_MIDD_STATE--;
                    if(TDA_SET_MIDD_STATE < TDA_SET_MIDD_CENT_FREQ)
                        TDA_SET_MIDD_STATE = TDA_SET_MIDD_CENT_FREQ;
                    TFT_send(tda_set_LTMB[TDA_SET_MIDD_STATE], sizeof(tda_set_LTMB[TDA_SET_MIDD_STATE]));
                    break;
                }
                case TDA_SET_BASS:
                {
                    TDA_SET_BASS_STATE--;
                    if(TDA_SET_BASS_STATE < TDA_SET_BASS_CENT_FREQ)
                        TDA_SET_BASS_STATE = TDA_SET_BASS_CENT_FREQ;
                    TFT_send(tda_set_LTMB[TDA_SET_BASS_STATE], sizeof(tda_set_LTMB[TDA_SET_BASS_STATE]));
                    break;
                }
                case TDA_SET_SPEAKER_ATT:
                {
                    TDA_SET_SATT_STATE--;
                    if(TDA_SET_SATT_STATE < TDA_SET_SATT_L_F)
                        TDA_SET_SATT_STATE = TDA_SET_SATT_R_R;
                    TFT_send(tda_set_satt[TDA_SET_SATT_STATE], sizeof(tda_set_satt[TDA_SET_SATT_STATE]));
                    break;
                }
            }
        }        
    }
    
    if(BT_VOL_UP) // increase volume
    {
        //GPIOB->BSRR |= GPIO_BSRR_BS12;
        if(STATE == MAIN)
        {
            MUTED = 0;
            VOLUME++;
            if(VOLUME > 15) VOLUME = 15;
            
            flash_write_newdata();

            i2c_buff[0] = TDA_MAIN_SOURCE;
            i2c_buff[1] = TDA_inputs[INPUT_SEL];
            I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
            
            i2c_buff[0] = TDA_VOLUME;
            i2c_buff[1] = VOLUME > 0 ? VOLUME : 16-VOLUME;
            I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));

            main_VOL_text[9] =   VOLUME > 0 ? '+' : '-';
            main_VOL_text[10] = (VOLUME > 0 ? VOLUME/10 : -VOLUME/10) + 0x30;
            main_VOL_text[11] = (VOLUME > 0 ? (VOLUME-(VOLUME/10)*10) : (-VOLUME-(-VOLUME/10)*10)) + 0x30;
            
            TFT_send(main_VOL_text, sizeof(main_VOL_text));
        }
        
        if(STATE == TDA_SETT)
        {
            switch(TDA_SET_STATE)
            {
                case TDA_SET_LOUDNESS:
                {
                    switch(TDA_SET_LOUD_STATE)
                    {
                        case TDA_SET_LOUD_CENT_FREQ:
                        {
                            TDA_loudness.center_freq++;
                            if(TDA_loudness.center_freq > 3) TDA_loudness.center_freq = 3;
                            switch (TDA_loudness.center_freq)
                            {
                                case 0:
                                {
                                    tda_set_loud[9] =' ';
                                    tda_set_loud[10]='F';
                                    tda_set_loud[11]='L';
                                    tda_set_loud[12]='A';
                                    tda_set_loud[13]='T';
                                    tda_set_loud[14]=' ';
                                    tda_set_loud[15]=' ';
                                    break;
                                }
                                case 1:
                                {
                                    tda_set_loud[9] =' ';
                                    tda_set_loud[10]='4';
                                    tda_set_loud[11]='0';
                                    tda_set_loud[12]='0';
                                    tda_set_loud[13]=' ';
                                    tda_set_loud[14]='H';
                                    tda_set_loud[15]='z';
                                    break;
                                }
                                case 2:
                                {
                                    tda_set_loud[9] =' ';
                                    tda_set_loud[10]='8';
                                    tda_set_loud[11]='0';
                                    tda_set_loud[12]='0';
                                    tda_set_loud[13]=' ';
                                    tda_set_loud[14]='H';
                                    tda_set_loud[15]='z';
                                    break;
                                }
                                case 3:
                                {
                                    tda_set_loud[9] ='2';
                                    tda_set_loud[10]='4';
                                    tda_set_loud[11]='0';
                                    tda_set_loud[12]='0';
                                    tda_set_loud[13]=' ';
                                    tda_set_loud[14]='H';
                                    tda_set_loud[15]='z';
                                    break;
                                }
                            }                            
                            
                            
                            break;
                        }
                        case TDA_SET_LOUD_ATTENUATION:
                        {
                            if(TDA_loudness.atteniation == 0)
                                TDA_loudness.atteniation = 0;
                            else
                            TDA_loudness.atteniation--;

                            tda_set_loud[30] = TDA_loudness.atteniation/10 + 0x30;
                            tda_set_loud[31] = TDA_loudness.atteniation-((TDA_loudness.atteniation/10)*10) + 0x30;
                            break;
                        }
                        case TDA_SET_LOUD_HIGN_BOOST:
                        {
                            TDA_loudness.high_boost = 0;
                            
                            tda_set_loud[48] = ' ';
                            tda_set_loud[49] = 'O';
                            tda_set_loud[50] = 'N';                            
                            break;
                        }
                    }
                    
                    i2c_buff[0] = TDA_MAIN_LOUDNESS;
                    i2c_buff[1] = (TDA_loudness.atteniation & 0xF)|((TDA_loudness.center_freq << 4) & 0x30)| ((TDA_loudness.high_boost << 6) & 0x40);
                    I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                    
                    TFT_send(tda_set_loud, sizeof(tda_set_loud));
                    break;
                }
                case TDA_SET_TREB:
                {
                    if(TDA_SET_TREB_STATE == TDA_SET_TREB_CENT_FREQ)
                    {
                        TDA_treble.center_freq++;
                        if(TDA_treble.center_freq > 3) TDA_treble.center_freq = 3;
                        
                        tda_set_treb[10] = (TDA_treble.center_freq*25)/10 + 0x30;
                        tda_set_treb[12] = (TDA_treble.center_freq*25)-(((TDA_treble.center_freq*25)/10)*10) + 0x30;
                        
                    }
                    else
                    {
                        TDA_treble.atteniation++;
                        if(TDA_treble.atteniation > 15) TDA_treble.atteniation = 15;
                        
                        tda_set_treb[29] = (TDA_treble.atteniation > 0) ? ('+') : ('-');
                        tda_set_treb[30] = abs(TDA_treble.atteniation/10) + 0x30;
                        tda_set_treb[31] = abs(TDA_treble.atteniation-((TDA_treble.atteniation/10)*10)) + 0x30;
                    }
                    
                    i2c_buff[0] = TDA_TREBLE_FILTER;
                    i2c_buff[1] = 0x80 | ((TDA_treble.center_freq << 5)&0x60) | (TDA_treble.atteniation > 0 ? (TDA_treble.atteniation | 0x10):(abs(TDA_treble.atteniation)));
                    I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                    
                    TFT_send(tda_set_treb, sizeof(tda_set_treb));
                    break;
                }
                case TDA_SET_MIDD:
                {
                    switch(TDA_SET_MIDD_STATE)
                    {
                        case TDA_SET_MIDD_CENT_FREQ:
                        {
                            TDA_middle.center_freq++;
                            if(TDA_middle.center_freq > 3) TDA_middle.center_freq = 3;
                            
                            tda_set_midd[9]  = ((TDA_middle.center_freq+1)*5)/10 + 0x30;
                            tda_set_midd[10] = (TDA_middle.center_freq+1)*5-((((TDA_middle.center_freq+1)*5)/10)*10) + 0x30;
                            if(TDA_middle.center_freq == 3) tda_set_midd[10] = '5';

                            break;
                        }
                        case TDA_SET_MIDD_ATTENUATION:
                        {
                            TDA_middle.atteniation++;
                            if(TDA_middle.atteniation > 15) TDA_middle.atteniation = 15;
                            
                            tda_set_midd[29] = (TDA_middle.atteniation > 0) ? ('+') : ('-');
                            tda_set_midd[30] = abs(TDA_middle.atteniation/10) + 0x30;
                            tda_set_midd[31] = abs(TDA_middle.atteniation-((TDA_middle.atteniation/10)*10)) + 0x30;
                            
                            break;
                        }
                        case TDA_SET_MIDD_Q_FACTOR:
                        {
                            TDA_middle.Q_factot++;
                            if(TDA_middle.Q_factot > 3) TDA_middle.Q_factot = 3;
                            
                            tda_set_midd[48] = (TDA_middle.Q_factot*25+50)/100 + 0x30;
                            tda_set_midd[50] = ((TDA_middle.Q_factot*25+50)-((TDA_middle.Q_factot*25+50)/100)*100)/10 + 0x30;
                            tda_set_midd[51] = (TDA_middle.Q_factot*25+50)-((TDA_middle.Q_factot*25+50)/10)*10 + 0x30;
                            
                            break;
                        }
                    }
                    i2c_buff[0] = TDA_MIDDLE_FILTER;
                    i2c_buff[1] = ((TDA_middle.Q_factot << 5) & 0x60) | ((TDA_middle.atteniation > 0 ? (TDA_middle.atteniation | 0x10):(abs(TDA_middle.atteniation))) & 0x1F);
                    I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                    
                    i2c_buff[0] = TDA_M_B_CENT_FREQ;
                    i2c_buff[1] = 0xC0 | ((TDA_bass.center_freq << 4) & 0x30) | ((TDA_middle.center_freq << 2) & 0xC);
                    I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                    
                    TFT_send(tda_set_midd, sizeof(tda_set_midd));
                    break;
                }
                case TDA_SET_BASS:
                {
                    switch(TDA_SET_BASS_STATE)
                    {
                        case TDA_SET_BASS_CENT_FREQ:
                        {
                            TDA_bass.center_freq++;
                            if(TDA_bass.center_freq > 3) TDA_bass.center_freq = 3;
                            
                            switch(TDA_bass.center_freq)
                            {
                                case 0:
                                {
                                    tda_set_bass[9]  = ' ';
                                    tda_set_bass[10] = '6';
                                    break;
                                }
                                case 1:
                                {
                                    tda_set_bass[9]  = ' ';
                                    tda_set_bass[10] = '8';
                                    break;
                                }
                                case 2:
                                {
                                    tda_set_bass[9]  = '1';
                                    tda_set_bass[10] = '0';
                                    break;
                                }
                                case 3:
                                {
                                    tda_set_bass[9]  = '2';
                                    tda_set_bass[10] = '0';
                                    break;
                                }
                            }

                            break;
                        }
                        case TDA_SET_BASS_ATTENUATION:
                        {
                            TDA_bass.atteniation++;
                            if(TDA_bass.atteniation > 15) TDA_bass.atteniation = 15;
                            
                            tda_set_bass[28] = (TDA_bass.atteniation > 0) ? ('+') : ('-');
                            tda_set_bass[29] = abs(TDA_bass.atteniation/10) + 0x30;
                            tda_set_bass[30] = abs(TDA_bass.atteniation-((TDA_bass.atteniation/10)*10)) + 0x30;
                            
                            break;
                        }
                        case TDA_SET_BASS_Q_FACTOR:
                        {
                            TDA_bass.Q_factot++;
                            if(TDA_bass.Q_factot > 3) TDA_bass.Q_factot = 3;
                            
                            tda_set_bass[47] = '1';
                            tda_set_bass[49] = (TDA_bass.Q_factot*25)/10 + 0x30;
                            tda_set_bass[50] =  TDA_bass.Q_factot*25 - ((TDA_bass.Q_factot*25)/10)*10 + 0x30;
                            if(TDA_bass.Q_factot == 3)
                            {
                                tda_set_bass[47] = '2';
                                tda_set_bass[49] = '0';
                                tda_set_bass[50] = '0';
                            }
                            
                            break;
                        }
                    }
                    
                    i2c_buff[0] = TDA_BASS_FILTER;
                    i2c_buff[1] = ((TDA_bass.Q_factot << 5) & 0x60) | ((TDA_bass.atteniation > 0 ? (TDA_bass.atteniation | 0x10):(abs(TDA_bass.atteniation))) & 0x1F);
                    I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                    
                    i2c_buff[0] = TDA_M_B_CENT_FREQ;
                    i2c_buff[1] = ((TDA_bass.center_freq << 4) & 0x30) | ((TDA_middle.center_freq << 2) & 0xC);
                    I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                    
                    TFT_send(tda_set_bass, sizeof(tda_set_bass));
                    break;
                }
                case TDA_SET_SPEAKER_ATT:
                {
                    switch(TDA_SET_SATT_STATE)
                    {
                        case TDA_SET_SATT_L_F:
                        {
                            TDA_sp_att.left_front++;
                            if(TDA_sp_att.left_front > 15) TDA_sp_att.left_front = 15;
                            
                            i2c_buff[0] = TDA_SPEAKER_ATT_LF;
                            i2c_buff[1] = TDA_sp_att.left_front > 0 ? TDA_sp_att.left_front : 16-TDA_sp_att.left_front;
                            I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                            
                            tda_set_satt_val[9] =   TDA_sp_att.left_front > 0 ? '+' : '-';
                            tda_set_satt_val[10] = (TDA_sp_att.left_front > 0 ?  TDA_sp_att.left_front/10 : -TDA_sp_att.left_front/10) + 0x30;
                            tda_set_satt_val[11] = (TDA_sp_att.left_front > 0 ? (TDA_sp_att.left_front - (TDA_sp_att.left_front/10)*10) : (-TDA_sp_att.left_front-(-TDA_sp_att.left_front/10)*10)) + 0x30;
                            break;
                        }
                        case TDA_SET_SATT_R_F:
                        {
                            TDA_sp_att.right_front++;
                            if(TDA_sp_att.right_front > 15) TDA_sp_att.right_front = 15;
                            
                            i2c_buff[0] = TDA_SPEAKER_ATT_RF;
                            i2c_buff[1] = TDA_sp_att.right_front > 0 ? TDA_sp_att.right_front : 16-TDA_sp_att.right_front;
                            
                            tda_set_satt_val[25] =  TDA_sp_att.right_front > 0 ? '+' : '-';
                            tda_set_satt_val[26] = (TDA_sp_att.right_front > 0 ?  TDA_sp_att.right_front/10 : -TDA_sp_att.right_front/10) + 0x30;
                            tda_set_satt_val[27] = (TDA_sp_att.right_front > 0 ? (TDA_sp_att.right_front - (TDA_sp_att.right_front/10)*10) : (-TDA_sp_att.right_front-(-TDA_sp_att.right_front/10)*10)) + 0x30;                            break;
                        }
                        case TDA_SET_SATT_L_R:
                        {
                            TDA_sp_att.left_rear++;
                            if(TDA_sp_att.left_rear > 15) TDA_sp_att.left_rear = 15;
                            
                            i2c_buff[0] = TDA_SPEAKER_ATT_LR;
                            i2c_buff[1] = TDA_sp_att.left_rear > 0 ? TDA_sp_att.left_rear : 16-TDA_sp_att.left_rear;
                            
                            tda_set_satt_val[41] =  TDA_sp_att.left_rear > 0 ? '+' : '-';
                            tda_set_satt_val[42] = (TDA_sp_att.left_rear > 0 ?  TDA_sp_att.left_rear/10 : -TDA_sp_att.left_rear/10) + 0x30;
                            tda_set_satt_val[43] = (TDA_sp_att.left_rear > 0 ? (TDA_sp_att.left_rear - (TDA_sp_att.left_rear/10)*10) : (-TDA_sp_att.left_rear-(-TDA_sp_att.left_rear/10)*10)) + 0x30;
                            break;
                        }
                        case TDA_SET_SATT_R_R:
                        {
                            TDA_sp_att.right_rear++;
                            if(TDA_sp_att.right_rear > 15) TDA_sp_att.right_rear = 15;
                            
                            i2c_buff[0] = TDA_SPEAKER_ATT_RR;
                            i2c_buff[1] = TDA_sp_att.right_rear > 0 ? TDA_sp_att.right_rear : 16-TDA_sp_att.right_rear;
                            
                            tda_set_satt_val[57] =  TDA_sp_att.right_rear > 0 ? '+' : '-';
                            tda_set_satt_val[58] = (TDA_sp_att.right_rear > 0 ?  TDA_sp_att.right_rear/10 : -TDA_sp_att.right_rear/10) + 0x30;
                            tda_set_satt_val[59] = (TDA_sp_att.right_rear > 0 ? (TDA_sp_att.right_rear - (TDA_sp_att.right_rear/10)*10) : (-TDA_sp_att.right_rear-(-TDA_sp_att.right_rear/10)*10)) + 0x30;
                            break;
                        }
                    }
                    I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                    
                    TFT_send(tda_set_satt_val, sizeof(tda_set_satt_val));
                    
                    break;
                }
            }
            flash_write_newdata();
        }
    }
    
    if(BT_VOL_DOWN) // decrease volume
    {
        //GPIOB->BSRR |= GPIO_BSRR_BS12;
        if(STATE == MAIN)
        {
            MUTED = 0;
            VOLUME--;
            if(VOLUME < -79) VOLUME = -79;
            
            flash_write_newdata();
            
            i2c_buff[0] = TDA_MAIN_SOURCE;
            i2c_buff[1] = TDA_inputs[INPUT_SEL];
            I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
            
            i2c_buff[0] = TDA_VOLUME;
            i2c_buff[1] = VOLUME > 0 ? VOLUME : 16-VOLUME;
            I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));

            main_VOL_text[9] =   VOLUME > 0 ? '+' : '-';
            main_VOL_text[10] = (VOLUME > 0 ? VOLUME/10 : -VOLUME/10) + 0x30;
            main_VOL_text[11] = (VOLUME > 0 ? (VOLUME-(VOLUME/10)*10) : (-VOLUME-(-VOLUME/10)*10)) + 0x30;
            
            TFT_send(main_VOL_text, sizeof(main_VOL_text));
        }
        
        if(STATE == TDA_SETT)
        {
            switch(TDA_SET_STATE)
            {
                case TDA_SET_LOUDNESS:
                {
                    switch(TDA_SET_LOUD_STATE)
                    {
                        case TDA_SET_LOUD_CENT_FREQ:
                        {
                            if(TDA_loudness.center_freq == 0) 
                                TDA_loudness.center_freq = 0;
                            else
                                TDA_loudness.center_freq--;
                            
                            switch (TDA_loudness.center_freq)
                            {
                                case 0:
                                {
                                    tda_set_loud[9] =' ';
                                    tda_set_loud[10]='F';
                                    tda_set_loud[11]='L';
                                    tda_set_loud[12]='A';
                                    tda_set_loud[13]='T';
                                    tda_set_loud[14]=' ';
                                    tda_set_loud[15]=' ';
                                    break;
                                }
                                case 1:
                                {
                                    tda_set_loud[9] =' ';
                                    tda_set_loud[10]='4';
                                    tda_set_loud[11]='0';
                                    tda_set_loud[12]='0';
                                    tda_set_loud[13]=' ';
                                    tda_set_loud[14]='H';
                                    tda_set_loud[15]='z';
                                    break;
                                }
                                case 2:
                                {
                                    tda_set_loud[9] =' ';
                                    tda_set_loud[10]='8';
                                    tda_set_loud[11]='0';
                                    tda_set_loud[12]='0';
                                    tda_set_loud[13]=' ';
                                    tda_set_loud[14]='H';
                                    tda_set_loud[15]='z';
                                    break;
                                }
                                case 3:
                                {
                                    tda_set_loud[9] ='2';
                                    tda_set_loud[10]='4';
                                    tda_set_loud[11]='0';
                                    tda_set_loud[12]='0';
                                    tda_set_loud[13]=' ';
                                    tda_set_loud[14]='H';
                                    tda_set_loud[15]='z';
                                    break;
                                }
                            }                            
                            
                            
                            break;
                        }
                        case TDA_SET_LOUD_ATTENUATION:
                        {
                            TDA_loudness.atteniation++;
                            if(TDA_loudness.atteniation > 15)
                                TDA_loudness.atteniation = 15;

                            tda_set_loud[30] = TDA_loudness.atteniation/10 + 0x30;
                            tda_set_loud[31] = TDA_loudness.atteniation-((TDA_loudness.atteniation/10)*10) + 0x30;
                            break;
                        }
                        case TDA_SET_LOUD_HIGN_BOOST:
                        {
                            TDA_loudness.high_boost = 1;
                            
                            tda_set_loud[48] = 'O';
                            tda_set_loud[49] = 'F';
                            tda_set_loud[50] = 'F';                            
                            break;
                        }
                    }
                    
                    i2c_buff[0] = TDA_MAIN_LOUDNESS;
                    i2c_buff[1] = (TDA_loudness.atteniation & 0xF)|((TDA_loudness.center_freq << 4) & 0x30)| ((TDA_loudness.high_boost << 6) & 0x40);
                    I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                    
                    TFT_send(tda_set_loud, sizeof(tda_set_loud));
                    break;
                }
                case TDA_SET_TREB:
                {
                    if(TDA_SET_TREB_STATE == TDA_SET_TREB_CENT_FREQ)
                    {
                        if(TDA_treble.center_freq == 0)
                            TDA_treble.center_freq = 0;
                        else                        
                            TDA_treble.center_freq--;
                        
                        
                        tda_set_treb[10] = (TDA_treble.center_freq*25)/10 + 0x30;
                        tda_set_treb[12] = (TDA_treble.center_freq*25)-(((TDA_treble.center_freq*25)/10)*10) + 0x30;
                        
                    }
                    else
                    {
                        TDA_treble.atteniation--;
                        if(TDA_treble.atteniation < -15) TDA_treble.atteniation = -15;
                        
                        tda_set_treb[29] = (TDA_treble.atteniation > 0) ? ('+') : ('-');
                        tda_set_treb[30] = abs(TDA_treble.atteniation/10) + 0x30;
                        tda_set_treb[31] = abs(TDA_treble.atteniation-((TDA_treble.atteniation/10)*10)) + 0x30;
                    }
                    
                    i2c_buff[0] = TDA_TREBLE_FILTER;
                    i2c_buff[1] = 0x80 | ((TDA_treble.center_freq << 5)&0x60) | (TDA_treble.atteniation > 0 ? (TDA_treble.atteniation | 0x10):(abs(TDA_treble.atteniation)));
                    I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                    
                    TFT_send(tda_set_treb, sizeof(tda_set_treb));
                    break;
                }
                case TDA_SET_MIDD:
                {
                    switch(TDA_SET_MIDD_STATE)
                    {
                        case TDA_SET_MIDD_CENT_FREQ:
                        {
                            if(TDA_middle.center_freq == 0)
                                TDA_middle.center_freq = 0;
                            else
                                TDA_middle.center_freq--;
                            
                            tda_set_midd[9]  = ((TDA_middle.center_freq+1)*5)/10 + 0x30;
                            tda_set_midd[10] = (TDA_middle.center_freq+1)*5-((((TDA_middle.center_freq+1)*5)/10)*10) + 0x30;
                            if(TDA_middle.center_freq == 3) tda_set_midd[10] = '5';

                            break;
                        }
                        case TDA_SET_MIDD_ATTENUATION:
                        {
                            TDA_middle.atteniation--;
                            if(TDA_middle.atteniation < -15) TDA_middle.atteniation = -15;
                            
                            tda_set_midd[29] = (TDA_middle.atteniation > 0) ? ('+') : ('-');
                            tda_set_midd[30] = abs(TDA_middle.atteniation/10) + 0x30;
                            tda_set_midd[31] = abs(TDA_middle.atteniation-((TDA_middle.atteniation/10)*10)) + 0x30;
                            
                            break;
                        }
                        case TDA_SET_MIDD_Q_FACTOR:
                        {
                            if(TDA_middle.Q_factot == 0)
                                TDA_middle.Q_factot = 0;
                            else
                                TDA_middle.Q_factot--;
                            
                            tda_set_midd[48] = (TDA_middle.Q_factot*25+50)/100 + 0x30;
                            tda_set_midd[50] = ((TDA_middle.Q_factot*25+50)-((TDA_middle.Q_factot*25+50)/100)*100)/10 + 0x30;
                            tda_set_midd[51] = (TDA_middle.Q_factot*25+50)-((TDA_middle.Q_factot*25+50)/10)*10 + 0x30;
                            
                            break;
                        }
                    }
                    
                    i2c_buff[0] = TDA_MIDDLE_FILTER;
                    i2c_buff[1] = ((TDA_middle.Q_factot << 5) & 0x60) | ((TDA_middle.atteniation > 0 ? (TDA_middle.atteniation | 0x10):(abs(TDA_middle.atteniation))) & 0x1F);
                    I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                    
                    i2c_buff[0] = TDA_M_B_CENT_FREQ;
                    i2c_buff[1] = 0xC0 | ((TDA_bass.center_freq << 4) & 0x30) | ((TDA_middle.center_freq << 2) & 0xC);
                    I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                    
                    TFT_send(tda_set_midd, sizeof(tda_set_midd));
                    break;
                }
                case TDA_SET_BASS:
                {
                    switch(TDA_SET_BASS_STATE)
                    {
                        case TDA_SET_BASS_CENT_FREQ:
                        {
                            if(TDA_bass.center_freq == 0)
                                TDA_bass.center_freq = 0;
                            else
                                TDA_bass.center_freq--;
                            
                            switch(TDA_bass.center_freq)
                            {
                                case 0:
                                {
                                    tda_set_bass[9]  = ' ';
                                    tda_set_bass[10] = '6';
                                    break;
                                }
                                case 1:
                                {
                                    tda_set_bass[9]  = ' ';
                                    tda_set_bass[10] = '8';
                                    break;
                                }
                                case 2:
                                {
                                    tda_set_bass[9]  = '1';
                                    tda_set_bass[10] = '0';
                                    break;
                                }
                                case 3:
                                {
                                    tda_set_bass[9]  = '2';
                                    tda_set_bass[10] = '0';
                                    break;
                                }
                            }

                            break;
                        }
                        case TDA_SET_BASS_ATTENUATION:
                        {
                            TDA_bass.atteniation--;
                            if(TDA_bass.atteniation < -15) TDA_bass.atteniation = -15;
                            
                            tda_set_bass[28] = (TDA_bass.atteniation > 0) ? ('+') : ('-');
                            tda_set_bass[29] = abs(TDA_bass.atteniation/10) + 0x30;
                            tda_set_bass[30] = abs(TDA_bass.atteniation-((TDA_bass.atteniation/10)*10)) + 0x30;
                            
                            break;
                        }
                        case TDA_SET_BASS_Q_FACTOR:
                        {
                            if(TDA_bass.Q_factot == 0)
                                TDA_bass.Q_factot = 0;
                            else
                                TDA_bass.Q_factot--;
                            
                            tda_set_bass[47] = '1';
                            tda_set_bass[49] = (TDA_bass.Q_factot*25)/10 + 0x30;
                            tda_set_bass[50] =  TDA_bass.Q_factot*25 - ((TDA_bass.Q_factot*25)/10)*10 + 0x30;
                            if(TDA_bass.Q_factot == 3)
                            {
                                tda_set_bass[47] = '2';
                                tda_set_bass[49] = '0';
                                tda_set_bass[50] = '0';
                            }
                            
                            break;
                        }
                    }
                    
                    i2c_buff[0] = TDA_BASS_FILTER;
                    i2c_buff[1] = ((TDA_bass.Q_factot << 5) & 0x60) | ((TDA_bass.atteniation > 0 ? (TDA_bass.atteniation | 0x10):(abs(TDA_bass.atteniation))) & 0x1F);
                    I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                    
                    i2c_buff[0] = TDA_M_B_CENT_FREQ;
                    i2c_buff[1] = ((TDA_bass.center_freq << 4) & 0x30) | ((TDA_middle.center_freq << 2) & 0xC);
                    I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                    
                    TFT_send(tda_set_bass, sizeof(tda_set_bass));
                    break;
                }
                case TDA_SET_SPEAKER_ATT:
                {
                    switch(TDA_SET_SATT_STATE)
                    {
                        case TDA_SET_SATT_L_F:
                        {
                            TDA_sp_att.left_front--;
                            if(TDA_sp_att.left_front < -79) TDA_sp_att.left_front = -79;
                            
                            i2c_buff[0] = TDA_SPEAKER_ATT_LF;
                            i2c_buff[1] = TDA_sp_att.left_front > 0 ? TDA_sp_att.left_front : 16-TDA_sp_att.left_front;
                            
                            tda_set_satt_val[9] =   TDA_sp_att.left_front > 0 ? '+' : '-';
                            tda_set_satt_val[10] = (TDA_sp_att.left_front > 0 ?  TDA_sp_att.left_front/10 : -TDA_sp_att.left_front/10) + 0x30;
                            tda_set_satt_val[11] = (TDA_sp_att.left_front > 0 ? (TDA_sp_att.left_front - (TDA_sp_att.left_front/10)*10) : (-TDA_sp_att.left_front-(-TDA_sp_att.left_front/10)*10)) + 0x30;
                            break;
                        }
                        case TDA_SET_SATT_R_F:
                        {
                            TDA_sp_att.right_front--;
                            if(TDA_sp_att.right_front < -79) TDA_sp_att.right_front = -79;
                            
                            i2c_buff[0] = TDA_SPEAKER_ATT_RF;
                            i2c_buff[1] = TDA_sp_att.right_front > 0 ? TDA_sp_att.right_front : 16-TDA_sp_att.right_front;
                            
                            tda_set_satt_val[25] =  TDA_sp_att.right_front > 0 ? '+' : '-';
                            tda_set_satt_val[26] = (TDA_sp_att.right_front > 0 ?  TDA_sp_att.right_front/10 : -TDA_sp_att.right_front/10) + 0x30;
                            tda_set_satt_val[27] = (TDA_sp_att.right_front > 0 ? (TDA_sp_att.right_front - (TDA_sp_att.right_front/10)*10) : (-TDA_sp_att.right_front-(-TDA_sp_att.right_front/10)*10)) + 0x30;                            break;
                        }
                        case TDA_SET_SATT_L_R:
                        {
                            TDA_sp_att.left_rear--;
                            if(TDA_sp_att.left_rear < -79) TDA_sp_att.left_rear = -79;
                            
                            i2c_buff[0] = TDA_SPEAKER_ATT_LR;
                            i2c_buff[1] = TDA_sp_att.left_rear > 0 ? TDA_sp_att.left_rear : 16-TDA_sp_att.left_rear;
                            
                            tda_set_satt_val[41] =  TDA_sp_att.left_rear > 0 ? '+' : '-';
                            tda_set_satt_val[42] = (TDA_sp_att.left_rear > 0 ?  TDA_sp_att.left_rear/10 : -TDA_sp_att.left_rear/10) + 0x30;
                            tda_set_satt_val[43] = (TDA_sp_att.left_rear > 0 ? (TDA_sp_att.left_rear - (TDA_sp_att.left_rear/10)*10) : (-TDA_sp_att.left_rear-(-TDA_sp_att.left_rear/10)*10)) + 0x30;
                            break;
                        }
                        case TDA_SET_SATT_R_R:
                        {
                            TDA_sp_att.right_rear--;
                            if(TDA_sp_att.right_rear < -79) TDA_sp_att.right_rear = -79;
                            
                            i2c_buff[0] = TDA_SPEAKER_ATT_RR;
                            i2c_buff[1] = TDA_sp_att.right_rear > 0 ? TDA_sp_att.right_rear : 16-TDA_sp_att.right_rear;
                            
                            tda_set_satt_val[57] =  TDA_sp_att.right_rear > 0 ? '+' : '-';
                            tda_set_satt_val[58] = (TDA_sp_att.right_rear > 0 ?  TDA_sp_att.right_rear/10 : -TDA_sp_att.right_rear/10) + 0x30;
                            tda_set_satt_val[59] = (TDA_sp_att.right_rear > 0 ? (TDA_sp_att.right_rear - (TDA_sp_att.right_rear/10)*10) : (-TDA_sp_att.right_rear-(-TDA_sp_att.right_rear/10)*10)) + 0x30;
                            break;
                        }
                    }
                    I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_buff, sizeof(i2c_buff));
                    
                    TFT_send(tda_set_satt_val, sizeof(tda_set_satt_val));
                    
                    break;
                }
            }
            flash_write_newdata();
        }
    }
    
    if(BT_CLK_UP) // Clock button for increase value
    {
        if(STATE == SET_TIME)
        {
            switch (SET_TIME_STATE)
            {
                case SET_TIME_HOUR:{
                                    hour++;
                                    if(hour >= 24) hour = 0;

                                    set_time_txt[9]  = hour/10 + 0x30;
                                    set_time_txt[10] = (hour - ((hour/10)*10)) + 0x30;

                                    TFT_send(&set_time_txt[0], 15);
                                        
                                    break;
                                   }
                case SET_TIME_MIN:{
                                    min++;
                                    if(min >= 60) min = 0;

                                    set_time_txt[24] = min/10 + 0x30;
                                    set_time_txt[25] = (min - ((min/10)*10)) + 0x30;

                                    TFT_send(&set_time_txt[15], 15);
                    
                                    break;
                                   }                    
                case SET_DATE_MON:{
                                    mon++;
                                    if(mon >= 13) mon = 1;

                                    set_time_txt[39] = mon/10 + 0x30;
                                    set_time_txt[40] = (mon - ((mon/10)*10)) + 0x30;

                                    TFT_send(&set_time_txt[30], 15);
                                        
                                    break;
                                   }                    
                case SET_DATE_DAY:{
                                    day++;
                                    if(day >= 32) day = 1;

                                    set_time_txt[54] = day/10 + 0x30;
                                    set_time_txt[55] = (day - ((day/10)*10)) + 0x30;

                                    TFT_send(&set_time_txt[45], 15);
                    
                                    break;
                                   }
                case SET_DATE_YEAR:{
                                    year++;
                                    if(year >= 30) year = 18;

                                    set_time_txt[69] = year/10 + 0x30;
                                    set_time_txt[70] = (year - ((year/10)*10)) + 0x30;

                                    TFT_send(&set_time_txt[60], 15);
                    
                                    break;
                                   }
                case SET_DATE_WEEK_DAY:{
                                    week_day++;
                                    if(week_day >= 8) week_day = 1;

                                    //memcpy(&set_time_txt[84], &day_of_week[((RTC->DR & RTC_DR_WDU_Msk) >> RTC_DR_WDU_Pos) - 1],9);

                                    memcpy(&set_time_txt[84], &day_of_week[week_day-1],9);
                    
                                    TFT_send(&set_time_txt[75], 23);
                
                                    break;
                               }
            case SET_SET:{
                                STATE = prev_state;
                                RCC->APB1ENR |= RCC_APB1ENR_PWREN;
                                PWR->CR |= PWR_CR_DBP;
                                RTC->WPR = 0xCA;
                                RTC->WPR = 0x53;
                                RTC->ISR |= RTC_ISR_INIT;
                                while((RTC->ISR & RTC_ISR_INITF) == 0){};
                                RTC->TR = (min/10 << 12) | 
                                         ((min - ((min/10)*10)) << 8) | 
                                          (hour/10 << 20) | 
                                         ((hour - ((hour/10)*10)) << 16);
                                RTC->DR = (day/10 << 4)  | 
                                          (day - (day/10)*10) | 
                                          (mon/10 << 12) | 
                                         ((mon - ((mon/10)*10)) << 8) | 
                                          (year/10 << 20) |
                                         ((year - ((year/10)*10)) << 16) |
                                          (week_day << 13);
                                RTC->ISR &= ~RTC_ISR_INIT;
                                RTC->WPR = 0xFF;
                                PWR->CR &= ~PWR_CR_DBP;
                                
                                TFT_send(pages[STATE], sizeof(pages[STATE]));
                                delay_send = 98;
                                break;
                               }
            default:break;
            }
        }
    }

    if(BT_CLK_DOWN) //Clock button for set time
    {
        switch(STATE)
        {
            case AUDIO_OFF:
            case MAIN:{
                        prev_state = STATE;
                        STATE = SET_TIME;
                        TFT_send(pages[STATE], sizeof(pages[STATE]));
                        
                        SET_TIME_STATE = SET_TIME_HOUR;

                        TFT_send(set_time_tft[SET_TIME_STATE], sizeof(set_time_tft[SET_TIME_STATE]));
                            
						hour = ((RTC->TR & RTC_TR_HT_Msk) >> RTC_TR_HT_Pos)*10 + ((RTC->TR & RTC_TR_HU_Msk) >> RTC_TR_HU_Pos);
                        min  = ((RTC->TR & RTC_TR_MNT_Msk)>> RTC_TR_MNT_Pos)*10+ ((RTC->TR & RTC_TR_MNU_Msk)>> RTC_TR_MNU_Pos);
                        day  = ((RTC->DR & RTC_DR_DT_Msk) >> RTC_DR_DT_Pos)*10 +  (RTC->DR & RTC_DR_DU_Msk);
                        mon  = ((RTC->DR & RTC_DR_MT_Msk) >> RTC_DR_MT_Pos)*10 + ((RTC->DR & RTC_DR_MU_Msk) >> RTC_DR_MU_Pos);
                        year = ((RTC->DR & RTC_DR_YT_Msk) >> RTC_DR_YT_Pos)*10 + ((RTC->DR & RTC_DR_YU_Msk) >> RTC_DR_YU_Pos);
                        week_day = (RTC->DR & RTC_DR_WDU_Msk) >> RTC_DR_WDU_Pos;
														
                        set_time_txt[9]  = hour/10 + 0x30;
                        set_time_txt[10] = (hour - ((hour/10)*10)) + 0x30;
                        
                        set_time_txt[24] = min/10 + 0x30;
                        set_time_txt[25] = (min - ((min/10)*10)) + 0x30;

                        set_time_txt[39] = mon/10 + 0x30;
                        set_time_txt[40] = (mon - ((mon/10)*10)) + 0x30;

                        set_time_txt[54] = day/10 + 0x30;
                        set_time_txt[55] = (day - ((day/10)*10)) + 0x30;
                            
                        set_time_txt[69] = year/10 + 0x30;
                        set_time_txt[70] = (year - ((year/10)*10)) + 0x30;
                        
						memcpy(&set_time_txt[84], &day_of_week[((RTC->DR & RTC_DR_WDU_Msk) >> RTC_DR_WDU_Pos) - 1],9);
                            
                        TFT_send(set_time_txt, sizeof(set_time_txt));

                        break;}
            case SET_TIME:{
                        SET_TIME_STATE++;
                        if(SET_TIME_STATE>SET_SET) SET_TIME_STATE = SET_TIME_HOUR;

                        TFT_send(set_time_tft[SET_TIME_STATE], sizeof(set_time_tft[SET_TIME_STATE]));

                        break;}

            default: break;
        }
    }

    
    if(BT_ENC_A)
    {
        if(STATE == MAIN)
        {
            STATE = TDA_SETT;
            
            TDA_SET_STATE = TDA_SET_LOUDNESS;
            TFT_send(tda_sett_pages[TDA_SET_STATE], sizeof(tda_sett_pages[TDA_SET_STATE]));
            TFT_send(tft_apage, sizeof(tft_apage));

            
            TDA_SET_LOUD_STATE = TDA_SET_LOUD_CENT_FREQ;
            TFT_send(tda_set_LTMB[TDA_SET_LOUD_STATE], sizeof(tda_set_LTMB[TDA_SET_LOUD_STATE]));
            
            switch (TDA_loudness.center_freq)
            {
                case 0:
                {
                    tda_set_loud[9] =' ';
                    tda_set_loud[10]='F';
                    tda_set_loud[11]='L';
                    tda_set_loud[12]='A';
                    tda_set_loud[13]='T';
                    tda_set_loud[14]=' ';
                    tda_set_loud[15]=' ';
                    break;
                }
                case 1:
                {
                    tda_set_loud[9] =' ';
                    tda_set_loud[10]='4';
                    tda_set_loud[11]='0';
                    tda_set_loud[12]='0';
                    tda_set_loud[13]=' ';
                    tda_set_loud[14]='H';
                    tda_set_loud[15]='z';
                    break;
                }
                case 2:
                {
                    tda_set_loud[9] =' ';
                    tda_set_loud[10]='8';
                    tda_set_loud[11]='0';
                    tda_set_loud[12]='0';
                    tda_set_loud[13]=' ';
                    tda_set_loud[14]='H';
                    tda_set_loud[15]='z';
                    break;
                }
                case 3:
                {
                    tda_set_loud[9] ='2';
                    tda_set_loud[10]='4';
                    tda_set_loud[11]='0';
                    tda_set_loud[12]='0';
                    tda_set_loud[13]=' ';
                    tda_set_loud[14]='H';
                    tda_set_loud[15]='z';
                    break;
                }
            }
            
            

            
            
            tda_set_loud[30] = TDA_loudness.atteniation/10 + 0x30;
            tda_set_loud[31] = TDA_loudness.atteniation-((TDA_loudness.atteniation/10)*10) + 0x30;
            
            if(TDA_loudness.high_boost)
            {
                tda_set_loud[48] = 'O';
                tda_set_loud[49] = 'F';
                tda_set_loud[50] = 'F';
            }
            else
            {
                tda_set_loud[48] = ' ';
                tda_set_loud[49] = 'O';
                tda_set_loud[50] = 'N';
            }
            
            TFT_send(tda_set_loud, sizeof(tda_set_loud));
            
            
            
        }else if(STATE == TDA_SETT)
        {
            if(TDA_SET_STATE == TDA_SET_SPEAKER_ATT)
            {
                STATE = MAIN;
                TFT_send(pages[STATE], sizeof(pages[STATE]));
                TFT_send(tft_apage, sizeof(tft_apage));
                delay_send = 98;
            }
            else
            {
                TDA_SET_STATE++;
                
                TFT_send(tda_sett_pages[TDA_SET_STATE], sizeof(tda_sett_pages[TDA_SET_STATE]));
                TFT_send(tft_apage, sizeof(tft_apage));
                
                switch(TDA_SET_STATE)
                {
                    case TDA_SET_TREB:
                    {
                        TDA_SET_TREB_STATE = TDA_SET_TREB_CENT_FREQ;
                        TFT_send(tda_set_LTMB[TDA_SET_TREB_STATE], sizeof(tda_set_LTMB[TDA_SET_TREB_STATE]));
                        
                        tda_set_treb[10] = (TDA_treble.center_freq*25)/10 + 0x30;
                        tda_set_treb[12] = TDA_treble.center_freq*25-(((TDA_treble.center_freq*25)/10)*10) + 0x30;
                        
                        tda_set_treb[29] = (TDA_treble.atteniation > 0) ? ('+') : ('-');
                        tda_set_treb[30] = abs(TDA_treble.atteniation/10) + 0x30;
                        tda_set_treb[31] = abs(TDA_treble.atteniation-((TDA_treble.atteniation/10)*10)) + 0x30;
                            
                        TFT_send(tda_set_treb, sizeof(tda_set_treb));
                        break;
                    }
                    case TDA_SET_MIDD:
                    {
                        TDA_SET_MIDD_STATE = TDA_SET_MIDD_CENT_FREQ;

                        TFT_send(tda_set_LTMB[TDA_SET_MIDD_STATE], sizeof(tda_set_LTMB[TDA_SET_MIDD_STATE]));

                        tda_set_midd[9]  = ((TDA_middle.center_freq+1)*5)/10 + 0x30;
                        tda_set_midd[10] = (TDA_middle.center_freq+1)*5-((((TDA_middle.center_freq+1)*5)/10)*10) + 0x30;
                        if(TDA_middle.center_freq == 3) tda_set_midd[10] = '5';

                        tda_set_midd[29] = (TDA_middle.atteniation > 0) ? ('+') : ('-');
                        tda_set_midd[30] = abs(TDA_middle.atteniation/10) + 0x30;
                        tda_set_midd[31] = abs(TDA_middle.atteniation-((TDA_middle.atteniation/10)*10)) + 0x30;
                        
                        tda_set_midd[48] = (TDA_middle.Q_factot*25+50)/100 + 0x30;
                        tda_set_midd[50] = ((TDA_middle.Q_factot*25+50)-((TDA_middle.Q_factot*25+50)/100)*100)/10 + 0x30;
                        tda_set_midd[51] = (TDA_middle.Q_factot*25+50)-((TDA_middle.Q_factot*25+50)/10)*10 + 0x30;
                        
                        TFT_send(tda_set_midd, sizeof(tda_set_midd));
                        break;
                    }
                    case TDA_SET_BASS:
                    {
                        TDA_SET_BASS_STATE = TDA_SET_BASS_CENT_FREQ;
                        TFT_send(tda_set_LTMB[TDA_SET_BASS_STATE], sizeof(tda_set_LTMB[TDA_SET_BASS_STATE]));
                        
                        switch(TDA_bass.center_freq)
                        {
                            case 0:
                            {
                                tda_set_bass[9]  = ' ';
                                tda_set_bass[10] = '6';
                                break;
                            }
                            case 1:
                            {
                                tda_set_bass[9]  = ' ';
                                tda_set_bass[10] = '8';
                                break;
                            }
                            case 2:
                            {
                                tda_set_bass[9]  = '1';
                                tda_set_bass[10] = '0';
                                break;
                            }
                            case 3:
                            {
                                tda_set_bass[9]  = '2';
                                tda_set_bass[10] = '0';
                                break;
                            }
                        }
                        
                        tda_set_bass[28] = (TDA_bass.atteniation > 0) ? ('+') : ('-');
                        tda_set_bass[29] = abs(TDA_bass.atteniation/10) + 0x30;
                        tda_set_bass[30] = abs(TDA_bass.atteniation-((TDA_bass.atteniation/10)*10)) + 0x30;
                        
                        tda_set_bass[47] = '1';
                        tda_set_bass[49] = (TDA_bass.Q_factot*25)/10 + 0x30;
                        tda_set_bass[50] =  TDA_bass.Q_factot*25 - ((TDA_bass.Q_factot*25)/10)*10 + 0x30;
                        if(TDA_bass.Q_factot == 3)
                        {
                            tda_set_bass[47] = '2';
                            tda_set_bass[49] = '0';
                            tda_set_bass[50] = '0';
                        }

                        TFT_send(tda_set_bass, sizeof(tda_set_bass));
                        break;
                    }
                    case TDA_SET_SPEAKER_ATT:
                    {
                        TDA_SET_SATT_STATE = TDA_SET_SATT_L_F;
                        TFT_send(tda_set_satt[TDA_SET_SATT_STATE], sizeof(tda_set_satt[TDA_SET_SATT_STATE]));
                        TFT_send(tda_set_satt[TDA_SET_SATT_STATE], sizeof(tda_set_satt[TDA_SET_SATT_STATE]));
                        
                        tda_set_satt_val[9] =   TDA_sp_att.left_front > 0 ? '+' : '-';
                        tda_set_satt_val[10] = (TDA_sp_att.left_front > 0 ?  TDA_sp_att.left_front/10 : -TDA_sp_att.left_front/10) + 0x30;
                        tda_set_satt_val[11] = (TDA_sp_att.left_front > 0 ? (TDA_sp_att.left_front - (TDA_sp_att.left_front/10)*10) : (-TDA_sp_att.left_front-(-TDA_sp_att.left_front/10)*10)) + 0x30;
                        
                        tda_set_satt_val[25] =  TDA_sp_att.right_front > 0 ? '+' : '-';
                        tda_set_satt_val[26] = (TDA_sp_att.right_front > 0 ?  TDA_sp_att.right_front/10 : -TDA_sp_att.right_front/10) + 0x30;
                        tda_set_satt_val[27] = (TDA_sp_att.right_front > 0 ? (TDA_sp_att.right_front - (TDA_sp_att.right_front/10)*10) : (-TDA_sp_att.right_front-(-TDA_sp_att.right_front/10)*10)) + 0x30;

                        tda_set_satt_val[41] =  TDA_sp_att.left_rear > 0 ? '+' : '-';
                        tda_set_satt_val[42] = (TDA_sp_att.left_rear > 0 ?  TDA_sp_att.left_rear/10 : -TDA_sp_att.left_rear/10) + 0x30;
                        tda_set_satt_val[43] = (TDA_sp_att.left_rear > 0 ? (TDA_sp_att.left_rear - (TDA_sp_att.left_rear/10)*10) : (-TDA_sp_att.left_rear-(-TDA_sp_att.left_rear/10)*10)) + 0x30;
                        
                        tda_set_satt_val[57] =  TDA_sp_att.right_rear > 0 ? '+' : '-';
                        tda_set_satt_val[58] = (TDA_sp_att.right_rear > 0 ?  TDA_sp_att.right_rear/10 : -TDA_sp_att.right_rear/10) + 0x30;
                        tda_set_satt_val[59] = (TDA_sp_att.right_rear > 0 ? (TDA_sp_att.right_rear - (TDA_sp_att.right_rear/10)*10) : (-TDA_sp_att.right_rear-(-TDA_sp_att.right_rear/10)*10)) + 0x30;
                        
                        TFT_send(tda_set_satt_val, sizeof(tda_set_satt_val));
                        break;
                    }
                }
            }
        }
    }
    
    if(((STATE == MAIN)||(STATE == AUDIO_OFF))&&(delay_send >= 100))
    {
		delay_send = 0;
        
        BT_send(BT_STATUS);
        
        TFT_TIME[14] = ((RTC->TR & RTC_TR_MNU_Msk)>> RTC_TR_MNU_Pos)+ 0x30; // minute
        TFT_TIME[13] = ((RTC->TR & RTC_TR_MNT_Msk)>> RTC_TR_MNT_Pos)+ 0x30;
        
        TFT_TIME[11] = ((RTC->TR & RTC_TR_HU_Msk) >> RTC_TR_HU_Pos) + 0x30; // hour
        TFT_TIME[10] = ((RTC->TR & RTC_TR_HT_Msk) >> RTC_TR_HT_Pos) + 0x30;
        
        TFT_TIME[32] =  (RTC->DR & RTC_DR_DU_Msk)        						+ 0x30; // day
        TFT_TIME[31] = ((RTC->DR & RTC_DR_DT_Msk) >> RTC_DR_DT_Pos) + 0x30;
        
        TFT_TIME[35] = ((RTC->DR & RTC_DR_MU_Msk) >> RTC_DR_MU_Pos) + 0x30; // month
        TFT_TIME[34] = ((RTC->DR & RTC_DR_MT_Msk) >> RTC_DR_MT_Pos) + 0x30;
        
        TFT_TIME[38] = ((RTC->DR & RTC_DR_YU_Msk) >> RTC_DR_YU_Pos) + 0x30; // year
        TFT_TIME[37] = ((RTC->DR & RTC_DR_YT_Msk) >> RTC_DR_YT_Pos) + 0x30;
        
		memcpy(&TFT_TIME[43], &day_of_week[((RTC->DR & RTC_DR_WDU_Msk) >> RTC_DR_WDU_Pos) - 1],9);

        TFT_send(TFT_TIME, sizeof(TFT_TIME));
        if(INPUT_SEL == USB)
        {
            USB_send(USB_Q_STATUS);
            USB_send(USB_Q_TRACK_NUMBER);
            USB_send(USB_Q_TRACK_NAME);
            
            main_USB_text[10] =  usb_track_info.num/100 + 0x30;
            main_USB_text[11] = (usb_track_info.num -(usb_track_info.num/100)*100)/10 + 0x30;
            main_USB_text[12] = (usb_track_info.num -(usb_track_info.num/10)*10) + 0x30;
        
            main_USB_text[18] = usb_track_info.name[0];
            main_USB_text[19] = usb_track_info.name[1];
            main_USB_text[20] = usb_track_info.name[2];
            main_USB_text[21] = usb_track_info.name[3];
            main_USB_text[22] = usb_track_info.name[4];
            main_USB_text[23] = usb_track_info.name[5];
            
            if(usb_status == 1)
            {
                main_USB_text[30] = 'P';
                main_USB_text[31] = 'L';
                main_USB_text[32] = 'A';
                main_USB_text[33] = 'Y';
                main_USB_text[34] = ' ';
            }else if(usb_status == 2)
            {
                main_USB_text[30] = 'P';
                main_USB_text[31] = 'A';
                main_USB_text[32] = 'U';
                main_USB_text[33] = 'S';
                main_USB_text[34] = 'E';
            }
            
            TFT_send(main_USB_text, sizeof(main_USB_text));            
        }
    }
    
    GPIOB->BSRR |= GPIO_BSRR_BR15;
}

void DMA1_Stream0_IRQHandler(void)
{
	DMA1->LIFCR = DMA_LIFCR_CTCIF0 |
                  DMA_LIFCR_CHTIF0 | 
                  DMA_LIFCR_CFEIF0 |
                  DMA_LIFCR_CTEIF0 |
				  DMA_LIFCR_CDMEIF0;
	
	DMA1_Stream0->M0AR = (uint32_t) bt_rx_buff;
	DMA1_Stream0->NDTR = BT_RX_BUFF_SIZE;
	DMA1_Stream0->CR |= DMA_SxCR_EN;
    
    if((bt_rx_buff[0] == 13)&&(bt_rx_buff[1] == 10))
	{
        switch(bt_rx_buff[3])
        {
            case 'I':{
                        BT_Status = BT_NO_DEV;
                    break;}
            case 'U':{
                        if(bt_rx_buff[4] == '1')
                            BT_Status = BT_NO_DEV;
                        else if(((bt_rx_buff[4] == '3')||(bt_rx_buff[4] == '5'))&&(BT_Status == BT_NO_DEV))
                            BT_Status = BT_CONN;
                    break;}
            case 'P':{
                        if (BT_Status == BT_PLAY)
                            BT_Status = BT_PAUSE;
                    break;}
            case 'R':{
                        BT_Status = BT_PLAY;
                    break;}
        
        };
        if ((STATE == MAIN) && (INPUT_SEL == BT))
            TFT_send(main_BT_text[BT_Status],sizeof(main_BT_text[BT_Status]));
    }
}

void DMA2_Stream0_IRQHandler(void)
{
    DMA2->LIFCR |= DMA_LIFCR_CFEIF0 |
                   DMA_LIFCR_CDMEIF0 |
                   DMA_LIFCR_CTEIF0 |
                   DMA_LIFCR_CHTIF0 |
                   DMA_LIFCR_CTCIF0;
	uint8_t V_IN = (ADC_Buff[0]*174)/0xFFF; //174 = 3.468(Vref) * 5(devider on pcb) * 10(for calculations)
	uint8_t P_IN = (ADC_Buff[1]*100)/0xFFF; //165 = 3.3(Vref) * 5(devider on pcb) * 10(for calculations)
    
    ADC_text[9]  =  V_IN/100 + 0x30;
    ADC_text[10] = (V_IN -(V_IN/100)*100)/10 + 0x30;
    ADC_text[12] = (V_IN -(V_IN/10)*10) + 0x30;
    
    ADC_text[17] =  P_IN/100 + 0x30;
    ADC_text[18] = (P_IN -(P_IN/100)*100)/10 + 0x30;
    ADC_text[19] = (P_IN -(P_IN/10)*10) + 0x30;
    
    TFT_send(ADC_text, sizeof(ADC_text));
}

void UART5_IRQHandler(void)
{
	if(UART5->SR & USART_SR_IDLE)
	{
		volatile uint32_t tmp;
		tmp = UART5->SR;
		tmp = UART5->DR;
		(void)tmp;
		DMA1_Stream0->CR &= ~DMA_SxCR_EN;
	}
}

void Init_GPIO(void)
{
    RCC-> AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                     RCC_AHB1ENR_GPIOBEN |
                     RCC_AHB1ENR_GPIOCEN |
					 RCC_AHB1ENR_GPIODEN;
  
    //LEDs
    GPIOB->MODER |= GPIO_MODE_OUTPUT_PP << PIN2*2  | // LED_WORK
                    GPIO_MODE_OUTPUT_PP << PIN12*2 | // Green 1
                    GPIO_MODE_OUTPUT_PP << PIN13*2 | // Red   1
                    GPIO_MODE_OUTPUT_PP << PIN14*2 | // Green 2
                    GPIO_MODE_OUTPUT_PP << PIN15*2;  // Red   2
	
    //Amplifier ON/OFF
    GPIOB->MODER &= ~((uint32_t)0x3     << PIN3*2);
    GPIOB->MODER |= GPIO_MODE_OUTPUT_PP << PIN3*2;
    
    //BT ON/OFF
    GPIOC->MODER |= GPIO_MODE_OUTPUT_PP << PIN13*2;
    
    //Bulb Check ON/OFF
    GPIOB->MODER |= GPIO_MODE_OUTPUT_PP << PIN6*2;
    
    //Set GPIOA PIN0 as usart4 TX <===> DF
    GPIOA->AFR[0] |= GPIO_AF8_UART4 << PIN0*4 |
                     GPIO_AF8_UART4 << PIN1*4;
    GPIOA->PUPDR |= GPIO_PULLUP << PIN0*2 |
                    GPIO_PULLUP << PIN1*2;
    GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN0*2 |
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN1*2;
    GPIOA->MODER |= GPIO_MODE_AF_PP << PIN0*2 |
                    GPIO_MODE_AF_PP << PIN1*2;
    
	//Set GPIOB PIN10 as usart3 TX & PIN11 as usart3 RX <===> TFT
    GPIOB->AFR[1] |= GPIO_AF7_USART3 << (PIN10*4-32) |
                     GPIO_AF7_USART3 << (PIN11*4-32);
    GPIOB->PUPDR |= GPIO_PULLUP << PIN10*2 |
					GPIO_PULLUP << PIN11*2;
    GPIOB->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN10*2 |
					  GPIO_SPEED_FREQ_VERY_HIGH << PIN1*2;
    GPIOB->MODER |= GPIO_MODE_AF_PP << PIN10*2 |
					GPIO_MODE_AF_PP << PIN11*2;
                    
    //Set GPIOC PIN12 as usart5 TX & GPIOD PIN2 as usart5 RX <===> BT
    GPIOC->AFR[1] |= GPIO_AF8_UART5 << (PIN12*4-32);
    GPIOD->AFR[0] |= GPIO_AF8_UART5 <<  PIN2*4;
    GPIOC->PUPDR |= GPIO_PULLUP << PIN12*2;
    GPIOD->PUPDR |= GPIO_PULLUP << PIN2*2;
    GPIOC->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN12*2;
    GPIOD->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN2*2;
    GPIOC->MODER |= GPIO_MODE_AF_PP << PIN12*2;
    GPIOD->MODER |= GPIO_MODE_AF_PP << PIN2*2;
		
    //GPIOB PIN8,9 I2C1 
    GPIOB->AFR[1] |= GPIO_AF4_I2C1 << (PIN8*4-32) |
                     GPIO_AF4_I2C1 << (PIN9*4-32);
    GPIOB->MODER |= GPIO_MODE_AF_PP << PIN8*2 |
                    GPIO_MODE_AF_PP << PIN9*2;
    GPIOB->PUPDR |= GPIO_PULLUP << PIN8*2 |
                    GPIO_PULLUP << PIN9*2;
    GPIOB->OTYPER |= GPIO_OTYPER_OT8 |
                     GPIO_OTYPER_OT9;
    GPIOB->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN8*2 |
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN9*2;    
    
    GPIOA->MODER &= ~((uint32_t) 0x3 << PIN15*2);
    GPIOA->PUPDR &= ~((uint32_t)0x3 << PIN15*2);
    GPIOB->MODER &= ~((uint32_t)0x3 << PIN4*2);
    GPIOB->PUPDR &= ~((uint32_t)0x3 << PIN4*2);

	GPIOA->MODER |= GPIO_MODE_INPUT << PIN8 *2 | //BT_CLK_DOWN
                    GPIO_MODE_INPUT << PIN9 *2 | //BT_CLK_UP
                    GPIO_MODE_INPUT << PIN10*2 | //BT1
                    GPIO_MODE_INPUT << PIN11*2 | //BT2
                    GPIO_MODE_INPUT << PIN12*2 | //BT3
                    GPIO_MODE_INPUT << PIN15*2;  //BT4
    GPIOC->MODER |= GPIO_MODE_INPUT << PIN10*2 | //BT5
                    GPIO_MODE_INPUT << PIN11*2;  //BT6
    GPIOB->MODER |= GPIO_MODE_INPUT << PIN4*2 |  //ENC_B
                    GPIO_MODE_INPUT << PIN5*2;   //ENC_C
                    
		
	//Set GPIOA PIN3,4 GPIOC PIN10 as analog input
    GPIOA->MODER |=	GPIO_MODE_ANALOG << PIN3*2 | //ADC3 - IN
                    GPIO_MODE_ANALOG << PIN4*2;  //ADC4 - prevmo
    GPIOA->OSPEEDR |=	GPIO_SPEED_FREQ_VERY_HIGH << PIN3*2 |
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN4*2;
    GPIOA->PUPDR |= GPIO_NOPULL << PIN3*2 |
                    GPIO_NOPULL << PIN4*2;
    
}

void Init_RTC(void)
{
    if((RCC->BDCR & RCC_BDCR_RTCEN)!=RCC_BDCR_RTCEN)//Проверка работы часов, если не включены, то инициализировать
    {
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;//Включить тактирование PWR и Backup
        PWR->CR |= PWR_CR_DBP; //Разрешить доступ к Backup области
        
        RCC->BDCR |= RCC_BDCR_BDRST;//Сбросить Backup область
        RCC->BDCR &= ~RCC_BDCR_BDRST;
       
        RCC->BDCR |= RCC_BDCR_RTCEN | 
                     RCC_BDCR_RTCSEL_0;        //Выбрать LSE источник (кварц 32768) и подать тактирование
        
        RCC->BDCR |= RCC_BDCR_LSEON;//Включить LSE
        while ((RCC->BDCR & RCC_BDCR_LSEON) != RCC_BDCR_LSEON){} //Дождаться включения
            
        PWR->CR &= ~PWR_CR_DBP;//запретить доступ к Backup области
    }
}

void Init_TFT(void)
{
    //UART3 to TFT
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    USART3->BRR = APB1/115200;
    USART3->CR1 = USART_CR1_UE |
                  USART_CR1_TE;
    USART3->CR3 = USART_CR3_DMAT;
    
    //DMA1_Stream4 for UART4
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    DMA1_Stream3->CR = DMA_SxCR_DIR_0 |
                       DMA_SxCR_MINC |
                       DMA_SxCR_CHSEL_2;
    DMA1_Stream3->PAR = (uint32_t) &USART3->DR;
}

void TFT_send(uint8_t *buff, uint8_t size)
{
    //while(DMA1_Stream4->NDTR != 0){};
			
    DMA1_Stream3->M0AR = (uint32_t) buff;
    DMA1_Stream3->NDTR = size;
    DMA1->LIFCR = DMA_LIFCR_CTCIF3 |
                  DMA_LIFCR_CHTIF3 | 
                  DMA_LIFCR_CFEIF3 |
                  DMA_LIFCR_CTEIF3;
    DMA1_Stream3->CR |= DMA_SxCR_EN;
	
	while(DMA1_Stream3->NDTR != 0){};
}

void Init_BT(void)
{
    //MX_USART3_UART_Init();
    RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
    UART5->BRR = APB1/921600;
    UART5->CR1 = USART_CR1_UE | 
                 USART_CR1_TE | 
                 USART_CR1_RE |
				 USART_CR1_IDLEIE;
    UART5->CR3 = USART_CR3_DMAR |
                 USART_CR3_DMAT;
    
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    DMA1_Stream0->CR = DMA_SxCR_MINC |
					   DMA_SxCR_TCIE |
                       DMA_SxCR_CHSEL_2;
    DMA1_Stream0->PAR = (uint32_t) &UART5->DR;
    DMA1_Stream0->M0AR = (uint32_t) bt_rx_buff;
    DMA1_Stream0->NDTR = BT_RX_BUFF_SIZE;
    DMA1_Stream0->CR |= DMA_SxCR_EN;
    
    DMA1_Stream7->CR &= ~DMA_SxCR_EN;
    DMA1_Stream7->CR = DMA_SxCR_DIR_0 |
                       DMA_SxCR_MINC |
                       DMA_SxCR_CHSEL_2;
    DMA1_Stream7->PAR = (uint32_t) &UART5->DR;
        
    BT_ON;
    
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	NVIC_EnableIRQ(UART5_IRQn);
}

void BT_send(uint8_t query)
{
    DMA1->HIFCR = DMA_HIFCR_CTCIF7 |
                  DMA_HIFCR_CHTIF7 | 
                  DMA_HIFCR_CFEIF7 |
                  DMA_HIFCR_CTEIF7;	
        
    DMA1_Stream7->M0AR = (uint32_t) bt_tx_query[query];
    DMA1_Stream7->NDTR = 7;
    DMA1_Stream7->CR |= DMA_SxCR_EN;
}

void Init_USB(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
    
    UART4->BRR = APB1/9600;
    UART4->CR1 = USART_CR1_UE |
                 USART_CR1_TE |
				 USART_CR1_RE;
}

uint8_t USB_send(uint8_t CMD)
{
    uint8_t tmp;
    uint32_t delay = 0;
    USB_command[2] = CMD;
    
    if(UART4->SR & USART_SR_RXNE)
        tmp = UART4->DR;
    
    for(uint8_t i = 0;i<4;i++)
    {
        while(!(UART4->SR & USART_SR_TC));
        UART4->DR = USB_command[i];
    }
    
    switch(CMD)
    {
        case USB_Q_STATUS:
        {
            for(uint8_t i = 4;i>0;i--)
            {
                delay = 0;
                while(!(UART4->SR & USART_SR_RXNE))
                {
                delay++;
                if(delay == 2000000)
                    return 1;
                };
                tmp = UART4->DR;
                usb_status |= ((tmp - ((tmp >= 0x61)? 0x57:0x30)) << ((i-1)*4));
            }
            break;
        }
        case USB_Q_TRACK_COUNT:
        {
            for(uint8_t i = 4;i>0;i--)
            {
                delay = 0;
                while(!(UART4->SR & USART_SR_RXNE))
                {
                delay++;
                if(delay == 2000000)
                    return 1;
                };
                tmp = UART4->DR;
                usb_track_info.count |= ((tmp - ((tmp >= 0x61)? 0x57:0x30)) << ((i-1)*4));
            }
            break;
        }
        case USB_Q_TRACK_NUMBER:
        {
            for(uint8_t i = 4;i>0;i--)
            {
                delay = 0;
                while(!(UART4->SR & USART_SR_RXNE))
                {
                delay++;
                if(delay == 2000000)
                    return 1;
                };
                tmp = UART4->DR;
                usb_track_info.num |= ((tmp - ((tmp >= 0x61)? 0x57:0x30)) << ((i-1)*4));
            }
            break;
        }
        case USB_Q_TRACK_LONG:
        {
            for(uint8_t i = 4;i>0;i--)
            {
                delay = 0;
                while(!(UART4->SR & USART_SR_RXNE))
                {
                delay++;
                if(delay == 2000000)
                    return 1;
                };
                tmp = UART4->DR;
                usb_track_info.tlong |= ((tmp - ((tmp >= 0x61)? 0x57:0x30)) << ((i-1)*4));
            }
            break;
        }
        case USB_Q_TRACK_TIME:
        {
            for(uint8_t i = 4;i>0;i--)
            {
                delay = 0;
                while(!(UART4->SR & USART_SR_RXNE))
                {
                delay++;
                if(delay == 2000000)
                    return 1;
                };
                tmp = UART4->DR;
                usb_track_info.time |= ((tmp - ((tmp >= 0x61)? 0x57:0x30)) << ((i-1)*4));
            }
            break;
        }
        case USB_Q_TRACK_NAME:
        {
            for(uint8_t i = 0;i<11;i++)
            {
                delay = 0;
                while(!(UART4->SR & USART_SR_RXNE))
                {
                delay++;
                if(delay == 2000000)
                    return 1;
                };
                tmp = UART4->DR;
                usb_track_info.name[i] = tmp;
            }
            break;
        }
        case USB_CMD_PLAY:
        case USB_CMD_PAUSE:
        case USB_CMD_NEXT:
        case USB_CMD_PREV:
        {
            for(uint8_t i = 0;i<2;i++)
            {
                delay = 0;
                while(!(UART4->SR & USART_SR_RXNE))
                {
                delay++;
                if(delay == 1000000)
                    return 1;
                };
                uint8_t tmp = UART4->DR;
            }           
            break;
        }
    }
        
    for(uint32_t i = 0;i<5500000;i++){};
    return 0;
}

uint8_t USB_send_par(uint8_t CMD, uint8_t PAR)
{
    uint32_t delay = 0;
    uint8_t tmp;
    USB_command5[2] = CMD;
    USB_command5[3] = PAR;
    
    if(UART4->SR & USART_SR_RXNE)
        tmp = UART4->DR;
    
    for(uint8_t i = 0;i<5;i++)
    {
        while(!(UART4->SR & USART_SR_TXE));
        UART4->DR = USB_command5[i];
    }
    
    if(CMD == USB_CMD_SOURCE)
        for(uint8_t i = 0;i<5;i++)
        {
            delay = 0;
            while(!(UART4->SR & USART_SR_RXNE))
            {
            delay++;
            if(delay == 2000000)
                return 1;
            };
            tmp = UART4->DR;
            if(tmp == 'S')
                return 1;
        }
    else
        for(uint8_t i = 0;i<2;i++)
        {
            delay = 0;
            while(!(UART4->SR & USART_SR_RXNE))
            {
            delay++;
            if(delay == 2000000)
                return 1;
            };
            tmp = UART4->DR;
        }
    for(uint32_t i = 0;i<4500000;i++){};
    return 0;
}

void Init_KEYs_TIM(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->PSC = APB1_TIM/10000-1;
    TIM14->ARR = 2000-1;
    TIM14->DIER = TIM_DIER_UIE;
    TIM14->CR1 = TIM_CR1_CEN; 
}

void Init_I2C1(void)
{
    //I2C1 Init
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    I2C1->CR2 = 45 << I2C_CR2_FREQ_Pos;// |
    I2C1->CCR = APB1/200000;
    I2C1->TRISE = APB1/1000000+1;
    I2C1->CR1 = I2C_CR1_PE;
}

uint8_t I2C1_Send(uint8_t addres,uint8_t *buff, uint16_t size)
{
    uint16_t i;
    
    I2C1->CR1 |= I2C_CR1_START;             // формирование сигнала старт
        
    while (!(I2C1->SR1 & I2C_SR1_SB))       // ждем окончания формирования сигнала "Старт"
    {
        if(I2C1->SR1 & I2C_SR1_BERR)
        return 1;
    }
    (void) I2C1->SR1;(void) I2C1->SR2;

    I2C1->DR = addres;                        // Передаем адресс
    
    while (!(I2C1->SR1 & I2C_SR1_ADDR))     // ожидаем окончания передачи адреса
    {
        if((I2C1->SR1 & I2C_SR1_BERR) || (I2C1->SR1 & I2C_SR1_AF))
        {
            I2C1->CR1 |= I2C_CR1_STOP;
            return 2;
        }
    }
    (void) I2C1->SR1;(void) I2C1->SR2;

    for(i=0;i<size;i++)
    {
        I2C1->DR = buff[i];
        while (!(I2C1->SR1 & I2C_SR1_BTF))  // ожидаем окончания передачи
        {
            if((I2C1->SR1 & I2C_SR1_BERR) || (I2C1->SR1 & I2C_SR1_AF))
            {   
                I2C1->CR1 |= I2C_CR1_STOP;
                return 3;
            }
        }
        (void) I2C1->SR1;(void) I2C1->SR2;
    }
    
    I2C1->CR1 |= I2C_CR1_STOP;              // формирование сигнала "Стоп"
    return 0;
}

uint8_t TEA_set_freq(uint16_t freq)
{
    uint8_t tea[5];
    
	freq = (4*(freq*100000UL+225000UL))/32768;
    
	tea[0] = freq >> 8;
    tea[1] = freq & 0xff;
    
    tea[2] = 0x10;
    tea[3] = 0x12;
    tea[4] = 0;//high ingection
    return I2C1_Send(0xC0, tea,sizeof(tea));
}

uint8_t Init_TDA(void)
{
    uint8_t init_buff[19];
    
    init_buff[0] = 0x20; // AI 1 + Subaddres 00000 - Main source sel
	init_buff[1] = (STATE == MAIN) ? TDA_inputs[INPUT_SEL] : TDA_SOURCE_MUTE; // Main source = SE2(FM), gain = 0;
    init_buff[2] = (TDA_loudness.atteniation & 0xF)|((TDA_loudness.center_freq << 4) & 0x30)| ((TDA_loudness.high_boost << 6) & 0x40); // Loudless
    init_buff[3] = 0xC7; // CLK FM off, SM step 2.56, 0.96, I2C, off
    init_buff[4] = VOLUME > 0 ? VOLUME : 16-VOLUME; // VOLUME
    init_buff[5] = 0x80 | ((TDA_treble.center_freq << 5)&0x60) | (TDA_treble.atteniation > 0 ? (TDA_treble.atteniation | 0x10):(abs(TDA_treble.atteniation))); // Ref out ext + treble off;
    init_buff[6] = ((TDA_middle.Q_factot << 5) & 0x60) | ((TDA_middle.atteniation > 0 ? (TDA_middle.atteniation | 0x10):(abs(TDA_middle.atteniation))) & 0x1F); // mid off
    init_buff[7] = ((TDA_bass.Q_factot << 5) & 0x60) | ((TDA_bass.atteniation > 0 ? (TDA_bass.atteniation | 0x10):(abs(TDA_bass.atteniation))) & 0x1F); // bass off
    init_buff[8] = 0x05; // sec source - mute, rear - main
    init_buff[9] = ((TDA_bass.center_freq << 4) & 0x30) | ((TDA_middle.center_freq << 2) & 0xC); // sub off all
    init_buff[10]= 0xFF; // mix off;
    init_buff[11]= TDA_sp_att.left_front  > 0 ? TDA_sp_att.left_front  : 16-TDA_sp_att.left_front; //Lefr  Front
    init_buff[12]= TDA_sp_att.right_front > 0 ? TDA_sp_att.right_front : 16-TDA_sp_att.right_front;//Right Front
    init_buff[13]= TDA_sp_att.left_rear   > 0 ? TDA_sp_att.left_rear   : 16-TDA_sp_att.left_rear;  //Lefr  Rear
    init_buff[14]= TDA_sp_att.right_rear  > 0 ? TDA_sp_att.right_rear  : 16-TDA_sp_att.right_rear; //Right Rear
    init_buff[15]= 0x00; //Mix level
    init_buff[16]= 0x00; //Sub att
    init_buff[17]= 0x39; //SA on
    init_buff[18]= 0x00; //test mode off
    
    return I2C1_Send(TDA7419_ADDRESS, init_buff, sizeof(init_buff));
}

uint32_t flash_read(uint32_t address)
{
	return (*(__IO uint32_t*) address);
}

//Функция возврщает true когда можно стирать или писать память.
uint8_t flash_ready(void)
{
	return !(FLASH->SR & FLASH_SR_BSY);
}

//Функция стирает одну страницу. В качестве адреса можно использовать любой
//принадлежащий диапазону адресов той странице которую нужно очистить.
void flash_erase_sector(uint8_t sector) 
{
		FLASH->CR |= FLASH_CR_SER; //Устанавливаем бит стирания одной страницы
		FLASH->CR |= sector << FLASH_CR_SNB_Pos; // Задаем её адрес
		FLASH->CR |= FLASH_CR_STRT; // Запускаем стирание
		
		while(!flash_ready())//Ждем пока страница сотрется.
		
		FLASH->CR&= ~(FLASH_CR_SER); //Сбрасываем бит обратно
}

void flash_write(uint32_t address, uint32_t data)
{
		FLASH->CR |= FLASH_CR_PG; //Разрешаем программирование флеша
		
		FLASH->CR |= FLASH_CR_PSIZE_1;
		
		while(!flash_ready()); //Ожидаем готовности флеша к записи
		
		*(__IO uint32_t*)address = (uint32_t)data; //Пишем младшие 2 бата
		
		while(!flash_ready());
		
		FLASH->CR &= ~(FLASH_CR_PG); //Запрещаем программирование флеша
}

void flash_write_newdata(void)
{
    uint32_t tmp_mem;
    
	flash_erase_sector(3); //start from 0x0800C000
	if(STATE == TDA_SETT)
    tmp_mem = (MAIN << 24 )   | (INPUT_SEL << 16) | (0xFF00 & (VOLUME << 8 ));
    else
    tmp_mem = (STATE << 24 )  | (INPUT_SEL << 16) | (0xFF00 & (VOLUME << 8 ));
    
    flash_write(MEM_ADDRESS, tmp_mem);
    
    tmp_mem = RADIO_FREQ << 16;
	flash_write(RADIO_FREQ_ADR, tmp_mem); //Radio Freq
    
    tmp_mem = (TDA_loudness.high_boost << 24) | (TDA_loudness.center_freq << 16) | (0xFF00 & (TDA_loudness.atteniation << 8));
    flash_write(TDA_MAIN_LOUD_ADR, tmp_mem); //Main loud
    
    tmp_mem = (TDA_treble.center_freq  << 24) | (0xFF0000 & (TDA_treble.atteniation   << 16));
    flash_write(TDA_TREB_ADR, tmp_mem); //Treble
    
    tmp_mem = (TDA_middle.Q_factot     << 24) | (TDA_middle.center_freq   << 16) | (0xFF00 & (TDA_middle.atteniation   << 8));
    flash_write(TDA_MIDD_ADR, tmp_mem); //Middle
    
    tmp_mem = (TDA_bass.Q_factot       << 24) | (TDA_bass.center_freq     << 16) | (0xFF00 & (TDA_bass.atteniation     << 8));
    flash_write(TDA_BASS_ADR, tmp_mem); //Bass
    
    tmp_mem = (0xFF000000 & (TDA_sp_att.left_front   << 24)) | (0xFF0000 & (TDA_sp_att.right_front   << 16)) | (0xFF00 & (TDA_sp_att.left_rear     << 8)) | (TDA_sp_att.right_rear & 0xFF);
    flash_write(TDA_SATT_ADR, tmp_mem); //Speaker attenuation
}

void Init_ADC(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->CR2 = ADC_CR2_ADON | 
                ADC_CR2_DMA |
                ADC_CR2_DDS |
                ADC_CR2_EXTSEL_3 | // Timer 3 TRGO event
                ADC_CR2_EXTEN_0;
    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->SMPR2 = ADC_SMPR2_SMP3_1 | ADC_SMPR2_SMP3_2 | // clock num for ADC3
                  ADC_SMPR2_SMP4_1 | ADC_SMPR2_SMP4_2;  // clock num for ADC4
    ADC1->SQR1 = (ADC_BUF_NUM-1) << ADC_SQR1_L_Pos; // 2 conversation
    ADC1->SQR3 = 3 << ADC_SQR3_SQ1_Pos |
                 4 << ADC_SQR3_SQ2_Pos;	
    //DMA Init
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    DMA2_Stream0->CR |= DMA_SxCR_CIRC |
                        DMA_SxCR_MINC | 
                        DMA_SxCR_MSIZE_0 | 
                        DMA_SxCR_PSIZE_0 |
						DMA_SxCR_TCIE;
    DMA2_Stream0->PAR = (uint32_t) &ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t) ADC_Buff;
    DMA2_Stream0->NDTR = ADC_BUF_NUM;
    
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    DMA2->LIFCR |= DMA_LIFCR_CFEIF0 |
                   DMA_LIFCR_CDMEIF0 |
                   DMA_LIFCR_CTEIF0 |
                   DMA_LIFCR_CHTIF0 |
                   DMA_LIFCR_CTCIF0;
    //Timer 3s Init
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;//Timer for ADC
    TIM3->PSC = APB1_TIM/10000-1;
    TIM3->ARR = 1000;
    TIM3->CR2 |= TIM_CR2_MMS_1;
    TIM3->CR1 |= TIM_CR1_CEN;
}

void Init_IWDG(void)
{
    IWDG->KR = 0x5555;
    IWDG->PR = IWDG_PR_PR_0 | IWDG_PR_PR_1; // /32
    IWDG->RLR = 0xFFF;
    IWDG->KR = 0xAAAA;
    IWDG->KR = 0xCCCC;
}

void IWDG_res(void)
{
    IWDG->KR = 0xAAAA;
}
