#include "main.h"

uint8_t STATE = LOAD;
uint8_t SET_TIME_STATE = SET_TIME_HOUR;
uint8_t INPUT_SEL = FM;
uint8_t prev_state;

uint8_t OFF_counter = 0;

uint8_t I2C_res;

uint16_t RADIO_FREQ = 0;

uint8_t BT_connected = 0;
uint8_t bt_rx_buff[BT_RX_BUFF_SIZE];

uint8_t DF_play = DF_ST_NO_USB;
uint8_t df_rx_buff[10];

int8_t VOLUME = 0;

uint16_t ADC_Buff[ADC_BUF_NUM];

int main(void)
{
        
    /*-- Init main clock --*/
    Init_RCC();
    /*-- Init main clock --*/
    Init_GPIO();
 
/*		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;	
	
		Init_ADC();
	
    Init_TFT();
		TFT_send(TFT_reset,sizeof(TFT_reset)); //RESET TFT
   
  	//-- Delay for TFT start --
    for(uint32_t delay = 0;delay < 10000000;delay++) 
    {};
			
    loading_txt[8] = '2';
    loading_txt[9] = '5';
    TFT_send(loading_txt, sizeof(loading_txt));
    
    Init_RTC();

    loading_txt[8] = '5';
    loading_txt[9] = '0';
    TFT_send(loading_txt, sizeof(loading_txt));
    
    Init_KEYs_TIM();
    
    loading_txt[8] = '7';
    loading_txt[9] = '5';
    TFT_send(loading_txt, sizeof(loading_txt));
    
    Init_I2C1();
    
		Init_BT();
		
		Init_DF();
//		DF_send(DF_RES, 0);
		DF_send(DF_PB_SORC, DF_USB);
		DF_send(DF_SET_VOL,30);
		DF_send(DF_Q_NUM_FILES, 0);
		
    //STATE = AUDIO_OFF;
		STATE = (0xFF000000 & flash_read(MEM_ADDRESS)) >> 24;
		INPUT_SEL = (0xFF0000 & flash_read(MEM_ADDRESS)) >> 16;
		VOLUME = (0xFF00 & flash_read(MEM_ADDRESS)) >> 8;
		RADIO_FREQ = (0xFFFF0000 & flash_read(RADIO_FREQ_ADR)) >> 16;

		TFT_send(pages[STATE], sizeof(pages[STATE]));

		I2C_res = Init_TDA();
		
		I2C_res = RDA_set_freq(RADIO_FREQ); // Init FM
		
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
										
										TFT_send(main_FM_text, sizeof(main_FM_text));
									break;}
					case BT:{
										BT_send(BT_STATUS);
									break;}
					case USB:{
										DF_send(DF_Q_CUR_STAT, 0);
									break;}
					case AUX:{
										TFT_send(main_AUX_text, sizeof(main_AUX_text));
									break;}				
				};
		}


		
	
    NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
		NVIC_EnableIRQ(DMA1_Stream1_IRQn);
		NVIC_EnableIRQ(DMA1_Stream0_IRQn);
		NVIC_EnableIRQ(USART3_IRQn);
			
		*/
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
        if (GPIOB->ODR & GPIO_ODR_OD2)
        {
            GPIOB->BSRR |= GPIO_BSRR_BR2;
        }
        else
        {
            GPIOB->BSRR |= GPIO_BSRR_BS2;
        }  			
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
	
		static uint8_t delay_send = 100;
	
		delay_send++;
	
    uint8_t i2c_sel_buff[2] = {TDA_MAIN_SOURCE, TDA_SOURCE_MUTE};
		uint8_t i2c_vol_buff[2] = {TDA_VOLUME, VOLUME};
		
		uint8_t V_IN = (ADC_Buff[0]*164)/0xFFF; //165 = 3.3(Vref) * 5(devider on pcb) * 10(for calculations)
		uint8_t P_IN = (ADC_Buff[1]*164)/0xFFF; //165 = 3.3(Vref) * 5(devider on pcb) * 10(for calculations)
		
    if(GPIOC->IDR & GPIO_PIN_5) // source select & OFF audio
    {
        OFF_counter++;
        if((OFF_counter >= 20)&&(STATE == MAIN))
        {
            OFF_counter = 20;
            i2c_sel_buff[0] = TDA_MAIN_SOURCE;
            i2c_sel_buff[1] = TDA_SOURCE_MUTE;
            
            I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_sel_buff, sizeof(i2c_sel_buff));
            
            STATE = AUDIO_OFF;
						
						flash_write_newdata();
            
            TFT_send(pages[STATE], sizeof(pages[STATE]));
						delay_send = 100;
					
						BT_send(BT_DISC);
            /* ADD stop players  */
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
                
                i2c_sel_buff[0] = TDA_MAIN_SOURCE;
                i2c_sel_buff[1] = TDA_inputs[INPUT_SEL];
                I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_sel_buff, sizeof(i2c_sel_buff));
                
                TFT_send(input_tft[INPUT_SEL], sizeof(input_tft[INPUT_SEL]));
							
							  switch (INPUT_SEL)
								{
									case FM:{
														main_FM_text[10] = RADIO_FREQ/1000 + 0x30;
														main_FM_text[11] = (RADIO_FREQ -(RADIO_FREQ/1000)*1000)/100 + 0x30;
														main_FM_text[12] = (RADIO_FREQ -(RADIO_FREQ/100)*100)/10 + 0x30;
														main_FM_text[14] = (RADIO_FREQ -(RADIO_FREQ/10)*10) + 0x30;
														
														TFT_send(main_FM_text, sizeof(main_FM_text));
													break;}
									case BT:{
														BT_send(BT_STATUS);
														TFT_send(main_BT_text[BT_connected], sizeof(main_BT_text[BT_connected]));
													break;}
									case USB:{
														DF_send(DF_Q_NUM_FILES, 0);
														DF_send(DF_Q_CUR_FIL, 0);
													break;}
									case AUX:{
														TFT_send(main_AUX_text, sizeof(main_AUX_text));
													break;}
								};
            }
            if(STATE == AUDIO_OFF)
            {
                STATE = MAIN;
            
								flash_write_newdata();
            
                TFT_send(pages[STATE], sizeof(pages[STATE]));

                i2c_sel_buff[0] = TDA_MAIN_SOURCE;
                i2c_sel_buff[1] = TDA_inputs[INPUT_SEL];
                
                I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_sel_buff, sizeof(i2c_sel_buff));
                
                TFT_send(input_tft[INPUT_SEL], sizeof(input_tft[INPUT_SEL]));
								switch (INPUT_SEL)
								{
									case FM:{
														main_FM_text[10] = RADIO_FREQ/1000 + 0x30;
														main_FM_text[11] = (RADIO_FREQ -(RADIO_FREQ/1000)*1000)/100 + 0x30;
														main_FM_text[12] = (RADIO_FREQ -(RADIO_FREQ/100)*100)/10 + 0x30;
														main_FM_text[14] = (RADIO_FREQ -(RADIO_FREQ/10)*10) + 0x30;
														
														TFT_send(main_FM_text, sizeof(main_FM_text));
													break;}
									case BT:{
														BT_send(BT_STATUS);
														TFT_send(main_BT_text[BT_connected], sizeof(main_BT_text[BT_connected]));
													break;}
									case USB:{
														DF_send(DF_Q_CUR_FIL, 0);
													break;}								
								};

								main_VOL_text[9] =   VOLUME > 0 ? '+' : '-';
								main_VOL_text[10] = (VOLUME > 0 ? VOLUME/10 : -VOLUME/10) + 0x30;
								main_VOL_text[11] = (VOLUME > 0 ? (VOLUME-(VOLUME/10)*10) : (-VOLUME-(-VOLUME/10)*10)) + 0x30;
								
								TFT_send(main_VOL_text, sizeof(main_VOL_text));
            }
        }
        OFF_counter = 0;
    }
    
    
		if(GPIOC->IDR & GPIO_PIN_4) // increase
		{
			if(STATE == MAIN)
			{				
				switch (INPUT_SEL)
				{
					case FM:{
										RADIO_FREQ++;
										if(RADIO_FREQ >= 1080)RADIO_FREQ = 850;
										RDA_set_freq(RADIO_FREQ);
										
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
										DF_send(DF_NEXT,0);
										DF_send(DF_Q_CUR_FIL,0);
										DF_send(DF_Q_CUR_STAT,0);
									break;}
				
				}
			}
		}

		if(GPIOC->IDR & GPIO_PIN_3) // play/pause
		{
			if(STATE == MAIN)
			{				
				switch (INPUT_SEL)
				{
					case BT:{
										BT_send(BT_PLAY_PAUSE);
									break;}
					case USB:{
										if (DF_play != DF_ST_NO_USB)
										{
											if(DF_play == DF_ST_PLAY)
												DF_send(DF_PAUSE, 0);
											if(DF_play == DF_ST_STOP)
												DF_send(DF_RND_PLAY, 1);
											if(DF_play == DF_ST_PAUSE)
												DF_send(DF_PLAY, 0);

											DF_send(DF_Q_CUR_FIL,0);
										}
										DF_send(DF_Q_CUR_STAT,0);
									break;}

				};
			}
		}
		
		if(GPIOC->IDR & GPIO_PIN_2) // decrease
		{
			if(STATE == MAIN)
			{				
				switch (INPUT_SEL)
				{
					case FM:{
										RADIO_FREQ--;
										if(RADIO_FREQ <= 850)RADIO_FREQ = 1080;
										RDA_set_freq(RADIO_FREQ);
										
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
										DF_send(DF_PREW,0);
										DF_send(DF_Q_CUR_FIL,0);
										DF_send(DF_Q_CUR_STAT,0);
									break;}
				}
			}
		}
		
		if(GPIOB->IDR & GPIO_PIN_5) // increase volume
		{
			VOLUME++;
			if(VOLUME > 15) VOLUME = 15;
			
			flash_write_newdata();

			i2c_vol_buff[1] = VOLUME > 0 ? VOLUME : 16-VOLUME;
			I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_vol_buff, sizeof(i2c_vol_buff));

			main_VOL_text[9] =   VOLUME > 0 ? '+' : '-';
			main_VOL_text[10] = (VOLUME > 0 ? VOLUME/10 : -VOLUME/10) + 0x30;
			main_VOL_text[11] = (VOLUME > 0 ? (VOLUME-(VOLUME/10)*10) : (-VOLUME-(-VOLUME/10)*10)) + 0x30;
			
			TFT_send(main_VOL_text, sizeof(main_VOL_text));

		}
		
		if(GPIOB->IDR & GPIO_PIN_4) // decrease volume
		{
			VOLUME--;
			if(VOLUME < -79) VOLUME = -79;
			
			flash_write_newdata();

			i2c_vol_buff[1] = VOLUME > 0 ? VOLUME : 16-VOLUME;
			I2C_res = I2C1_Send(TDA7419_ADDRESS, i2c_vol_buff, sizeof(i2c_vol_buff));

			main_VOL_text[9] =   VOLUME > 0 ? '+' : '-';
			main_VOL_text[10] = (VOLUME > 0 ? VOLUME/10 : -VOLUME/10) + 0x30;
			main_VOL_text[11] = (VOLUME > 0 ? (VOLUME-(VOLUME/10)*10) : (-VOLUME-(-VOLUME/10)*10)) + 0x30;
			
			TFT_send(main_VOL_text, sizeof(main_VOL_text));
		}
		
		if(GPIOC->IDR & GPIO_PIN_1)
		{
			DF_send(DF_Q_CUR_STAT,0);
		}
		
		if(GPIOC->IDR & GPIO_PIN_8) // Clock button for increase value
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

                                    memcpy(&set_time_txt[84], &day_of_week[((RTC->DR & RTC_DR_WDU_Msk) >> RTC_DR_WDU_Pos) - 1],9);

                                    TFT_send(&set_time_txt[75], 16);
                    
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
																		delay_send = 100;
                                    break;
                                   }
                default:break;
            }
        }
    }

    if((GPIOC->IDR & GPIO_PIN_9)) //Clock button for set time
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

    if(((STATE == MAIN)||(STATE == AUDIO_OFF))&&(delay_send >= 100))
    {
				delay_send = 0;
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
			
				ADC_text[9]  =  V_IN/100 + 0x30;
				ADC_text[10] = (V_IN -(V_IN/100)*100)/10 + 0x30;
				ADC_text[12] = (V_IN -(V_IN/10)*10) + 0x30;
				
				ADC_text[16] =  P_IN/100 + 0x30;
				ADC_text[17] = (P_IN -(P_IN/100)*100)/10 + 0x30;
				ADC_text[19] = (P_IN -(P_IN/10)*10) + 0x30;
				
				TFT_send(ADC_text, sizeof(ADC_text));

				if((INPUT_SEL == BT)&&(BT_connected == 0))
					BT_send(BT_STATUS);
				if((INPUT_SEL == USB)&&(DF_play == DF_ST_NO_USB))
					DF_send(DF_Q_CUR_STAT, 0);
    }

}

void DMA1_Stream1_IRQHandler(void)
{
	static uint8_t bt_inc = 0;
	DMA1->LIFCR = DMA_LIFCR_CTCIF1 |
                DMA_LIFCR_CHTIF1 | 
                DMA_LIFCR_CFEIF1 |
                DMA_LIFCR_CTEIF1 |
								DMA_LIFCR_CDMEIF1;
	
	DMA1_Stream1->M0AR = (uint32_t) bt_rx_buff;
	DMA1_Stream1->NDTR = BT_RX_BUFF_SIZE;
	DMA1_Stream1->CR |= DMA_SxCR_EN;
	
	if ((STATE == MAIN) && (INPUT_SEL == BT))
	{
		if((bt_rx_buff[0] == 13)&&(bt_rx_buff[1] == 10))
		{
			if(bt_rx_buff[3] == 'U')
			{
				if(bt_rx_buff[4] == '1')
				{
					BT_connected = 0;
					TFT_send(main_BT_text[BT_connected],sizeof(main_BT_text[BT_connected]));
				}
				if((bt_rx_buff[4] == '3')||(bt_rx_buff[4] == '5'))
				{
					BT_connected = 1;
					TFT_send(main_BT_text[BT_connected],sizeof(main_BT_text[BT_connected]));
				}
			}
			else if (bt_rx_buff[3] == 'P')
			{
				if(bt_inc)
					BT_connected = 1;
				else
					bt_inc = 1;
				if(BT_connected)
					TFT_send(main_BT_text[BT_connected+2],sizeof(main_BT_text[BT_connected+2]));
				else
				{
					TFT_send(main_BT_text[BT_connected],sizeof(main_BT_text[BT_connected]));
				}
			}
			else if	(bt_rx_buff[3] == 'R')
			{
				if(BT_connected)
					TFT_send(main_BT_text[BT_connected+1],sizeof(main_BT_text[BT_connected+1]));
			}else if((bt_rx_buff[2] == 'I')||(bt_rx_buff[3] == 'I'))
			{
					BT_connected = 0;
					bt_inc = 0;
					TFT_send(main_BT_text[BT_connected],sizeof(main_BT_text[BT_connected]));
			}
		}
		if(bt_rx_buff[0] == 'M')
		{
			if (bt_rx_buff[1] == 'P')
			{
				if(bt_inc)
					BT_connected = 1;
				else
					bt_inc = 1;

				if(BT_connected)
					TFT_send(main_BT_text[BT_connected+2],sizeof(main_BT_text[BT_connected+2]));
				else
				{
					DMA1_Stream3->CR |= DMA_SxCR_EN;
					TFT_send(main_BT_text[BT_connected],sizeof(main_BT_text[BT_connected]));
				}
			}
			else if	(bt_rx_buff[1] == 'R')
			{
				if(BT_connected)
					TFT_send(main_BT_text[BT_connected+1],sizeof(main_BT_text[BT_connected+1]));
				
			}
		}
	
	}
		
}

void DMA1_Stream0_IRQHandler(void)
{
	DMA1->LIFCR = DMA_LIFCR_CTCIF0 |
                DMA_LIFCR_CHTIF0 | 
                DMA_LIFCR_CFEIF0 |
                DMA_LIFCR_CTEIF0 |
								DMA_LIFCR_CDMEIF0;
	
	DMA1_Stream0->CR |= DMA_SxCR_EN;
	
	if ((df_rx_buff[0] == 0x7E)&&(df_rx_buff[1] == 0xFF))
	{
		if(df_rx_buff[3] == DF_Q_NUM_FILES)
		{
				main_DF_text[0][19] =  df_rx_buff[6]/100 + 0x30;
				main_DF_text[0][20] = (df_rx_buff[6] -(df_rx_buff[6]/100)*100)/10 + 0x30;
				main_DF_text[0][21] = (df_rx_buff[6] -(df_rx_buff[6]/10)*10) + 0x30;
				main_DF_text[1][19] =  df_rx_buff[6]/100 + 0x30;
				main_DF_text[1][20] = (df_rx_buff[6] -(df_rx_buff[6]/100)*100)/10 + 0x30;
				main_DF_text[1][21] = (df_rx_buff[6] -(df_rx_buff[6]/10)*10) + 0x30;
				main_DF_text[2][19] =  df_rx_buff[6]/100 + 0x30;
				main_DF_text[2][20] = (df_rx_buff[6] -(df_rx_buff[6]/100)*100)/10 + 0x30;
				main_DF_text[2][21] = (df_rx_buff[6] -(df_rx_buff[6]/10)*10) + 0x30;;
		}

		if(df_rx_buff[3] == DF_Q_CUR_FIL)
		{
				main_DF_text[0][15] =  df_rx_buff[6]/100 + 0x30;
				main_DF_text[0][16] = (df_rx_buff[6] -(df_rx_buff[6]/100)*100)/10 + 0x30;
				main_DF_text[0][17] = (df_rx_buff[6] -(df_rx_buff[6]/10)*10) + 0x30;
				main_DF_text[1][15] =  df_rx_buff[6]/100 + 0x30;
				main_DF_text[1][16] = (df_rx_buff[6] -(df_rx_buff[6]/100)*100)/10 + 0x30;
				main_DF_text[1][17] = (df_rx_buff[6] -(df_rx_buff[6]/10)*10) + 0x30;
				main_DF_text[2][15] =  df_rx_buff[6]/100 + 0x30;
				main_DF_text[2][16] = (df_rx_buff[6] -(df_rx_buff[6]/100)*100)/10 + 0x30;
				main_DF_text[2][17] = (df_rx_buff[6] -(df_rx_buff[6]/10)*10) + 0x30;			
		}
		
		if(df_rx_buff[3] == DF_Q_CUR_STAT)
		{
			DF_play = df_rx_buff[6];
		}
		
		if((df_rx_buff[3] == 0x40)||(df_rx_buff[3] == 0x3B))
			DF_play = DF_ST_NO_USB;

		if(df_rx_buff[3] == 0x3A)
		{
			DF_play = DF_ST_STOP;
		}
		if ((STATE == MAIN) && (INPUT_SEL == USB))
		{
			if(DF_play == DF_ST_NO_USB)
			{
				TFT_send(main_DF_text_no, sizeof(main_DF_text_no));
			}
			else
			{
				TFT_send(main_DF_text[DF_play], sizeof(main_DF_text[DF_play]));
			}
		}
	}
}

void USART3_IRQHandler(void)
{
	if(USART3->SR & USART_SR_IDLE)
	{
		volatile uint32_t tmp;
		tmp = USART3->SR;
		tmp = USART3->DR;
		(void)tmp;
		DMA1_Stream1->CR &= ~DMA_SxCR_EN;
	}
}



void Init_GPIO(void)
{
    RCC-> AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                     RCC_AHB1ENR_GPIOBEN |
                     RCC_AHB1ENR_GPIOCEN |
										 RCC_AHB1ENR_GPIODEN;
  
		GPIOB->MODER |= GPIO_MODE_OUTPUT_PP << PIN2*2  |
										GPIO_MODE_OUTPUT_PP << PIN12*2 |
										GPIO_MODE_OUTPUT_PP << PIN13*2 |
										GPIO_MODE_OUTPUT_PP << PIN14*2 |
										GPIO_MODE_OUTPUT_PP << PIN15*2;
	
    //Set GPIOA PIN0 as usart4 TX
    GPIOA->AFR[0] |= GPIO_AF8_UART4 << PIN0*4;
    GPIOA->PUPDR |= GPIO_PULLUP << PIN0*2;
    GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN0*2;
    GPIOA->MODER |= GPIO_MODE_AF_PP << PIN0*2;
    
	  //Set GPIOB PIN10 as usart3 TX & PIN11 as usart3 RX
    GPIOB->AFR[1] |= GPIO_AF7_USART3 << (PIN10*4-32) |
										 GPIO_AF7_USART3 << (PIN11*4-32);
    GPIOB->PUPDR |= GPIO_PULLUP << PIN10*2 |
										GPIO_PULLUP << PIN11*2;
    GPIOB->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN10*2 |
											GPIO_SPEED_FREQ_VERY_HIGH << PIN1*2;
    GPIOB->MODER |= GPIO_MODE_AF_PP << PIN10*2 |
										GPIO_MODE_AF_PP << PIN11*2;
		//Set GPIOC PIN12 as usart5 TX & GPIOD PIN2 as usart5 RX
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
    
		GPIOC->MODER |= GPIO_MODE_INPUT << PIN0*2 |
										GPIO_MODE_INPUT << PIN1*2 |
										GPIO_MODE_INPUT << PIN2*2 |
										GPIO_MODE_INPUT << PIN3*2 |
										GPIO_MODE_INPUT << PIN4*2 |
										GPIO_MODE_INPUT << PIN5*2 |
										GPIO_MODE_INPUT << PIN6*2;
		
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
    //UART4 to TFT
    RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
    UART4->BRR = APB1/115200;
    UART4->CR1 = USART_CR1_UE |
                 USART_CR1_TE;
    UART4->CR3 = USART_CR3_DMAT;
    
    //DMA1_Stream4 for UART4
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    DMA1_Stream4->CR = DMA_SxCR_DIR_0 |
                       DMA_SxCR_MINC |
                       DMA_SxCR_CHSEL_2;
    DMA1_Stream4->PAR = (uint32_t) &UART4->DR;
}

void TFT_send(uint8_t *buff, uint8_t size)
{
    //while(DMA1_Stream4->NDTR != 0){};
			
    DMA1_Stream4->M0AR = (uint32_t) buff;
    DMA1_Stream4->NDTR = size;
    DMA1->HIFCR = DMA_HIFCR_CTCIF4 |
                  DMA_HIFCR_CHTIF4 | 
                  DMA_HIFCR_CFEIF4 |
                  DMA_HIFCR_CTEIF4;
    DMA1_Stream4->CR |= DMA_SxCR_EN;
	
	  while(DMA1_Stream4->NDTR != 0){};
}

void Init_BT(void)
{
    //MX_USART3_UART_Init();
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    USART3->BRR = APB1/115200;
    USART3->CR1 = USART_CR1_UE | 
                  USART_CR1_TE | 
                  USART_CR1_RE |
									USART_CR1_IDLEIE;
    USART3->CR3 = USART_CR3_DMAR |
                  USART_CR3_DMAT;
    
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    DMA1_Stream1->CR = DMA_SxCR_MINC |
											 DMA_SxCR_TCIE |
                       DMA_SxCR_CHSEL_2;
    DMA1_Stream1->PAR = (uint32_t) &USART3->DR;
    DMA1_Stream1->M0AR = (uint32_t) bt_rx_buff;
    DMA1_Stream1->NDTR = BT_RX_BUFF_SIZE;
    DMA1_Stream1->CR |= DMA_SxCR_EN;
    
    DMA1_Stream3->CR = DMA_SxCR_DIR_0 |
                       DMA_SxCR_MINC |
                       DMA_SxCR_CHSEL_2;
    DMA1_Stream3->PAR = (uint32_t) &USART3->DR;
/*		DMA1_Stream3->M0AR = (uint32_t) bt_tx_query[BT_RESET];
		DMA1_Stream3->NDTR = 7;
		DMA1_Stream3->CR |= DMA_SxCR_EN;*/
}

void BT_send(uint8_t query)
{
		DMA1->LIFCR = DMA_LIFCR_CTCIF3 |
									DMA_LIFCR_CHTIF3 | 
									DMA_LIFCR_CFEIF3 |
									DMA_LIFCR_CTEIF3;	
	
		for(uint32_t delay = 0;delay < 100000;delay++){};
			
		DMA1_Stream3->M0AR = (uint32_t) bt_tx_query[query];
		DMA1_Stream3->NDTR = 7;
		DMA1_Stream3->CR |= DMA_SxCR_EN;
}

void Init_DF(void)
{
		RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
    UART5->BRR = APB1/9600;
    UART5->CR1 = USART_CR1_UE |
                 USART_CR1_TE |
								 USART_CR1_RE;
    UART5->CR3 = USART_CR3_DMAT|
                 USART_CR3_DMAR;
	
    DMA1_Stream7->CR = DMA_SxCR_DIR_0 |
                       DMA_SxCR_MINC |
                       DMA_SxCR_CHSEL_2;
    DMA1_Stream7->PAR = (uint32_t) &UART5->DR;
    DMA1_Stream7->M0AR = (uint32_t) DF_data;
    DMA1_Stream7->NDTR = 10;
	
		DMA1_Stream0->CR = DMA_SxCR_CIRC |	
											 DMA_SxCR_MINC |
											 DMA_SxCR_TCIE |
											 DMA_SxCR_CHSEL_2;
		DMA1_Stream0->PAR = (uint32_t) &UART5->DR;
		DMA1_Stream0->M0AR = (uint32_t) df_rx_buff;
		DMA1_Stream0->NDTR = 10;
		DMA1_Stream0->CR |= DMA_SxCR_EN;
}

void DF_send(uint8_t CMD, uint8_t PAR)
{
		DMA1->HIFCR = DMA_HIFCR_CTCIF7 |
									DMA_HIFCR_CHTIF7 | 
									DMA_HIFCR_CFEIF7 |
									DMA_HIFCR_CTEIF7;

		for(uint32_t delay = 0;delay < 1000000;delay++){};
			
		DF_data[3] = CMD;
    DF_data[6] = PAR;
		
		uint16_t sum = 0 - (DF_data[1] + DF_data[2] + DF_data[3] + DF_data[4] + DF_data[5] + DF_data[6]);
    
    DF_data[7] = ((sum & 0xFF00) >> 8);
    DF_data[8] = sum & 0xFF;	

		DMA1_Stream7->NDTR = 10;
    DMA1_Stream7->CR |= DMA_SxCR_EN;
		
		//while(DMA1_Stream7->NDTR != 0){};
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

uint8_t RDA_set_freq(uint16_t freq)
{
/*    uint8_t RDA_buff[12];
    
    RDA_buff[0] = 0xD0; RDA_buff[1] = 0x05; //02H:DHIZ - normal, MONO - stereo, BASS - boost en |02L: NEW_METHOD, EN
    freq-=760;
    RDA_buff[2] = freq >> 2;
    RDA_buff[3] = (freq << 6) | 0x18;       //03H:CHAN = 1040-760 >>2 |03L: CHAN << 6, BAND (01) - 76-108, SPACE (00) - 100kHz
    RDA_buff[4] = 0x02; RDA_buff[5] = 0x00; //04HL
    RDA_buff[6] = 0x88; RDA_buff[7] = 0x87; //05H:??? |05L:ANT- ON, VOL (1111) - 
    RDA_buff[8] = 0x00; RDA_buff[9] = 0x00; //06HL
    RDA_buff[10]= 0x42; RDA_buff[11]= 0x02; //07HL:???
    
    return I2C1_Send(0xC0, RDA_buff,sizeof(RDA_buff));*/
    
	  uint8_t tea[5];
    freq = (4*(freq*100000UL+225000UL))/32768;
    tea[0] = freq >> 8;
    tea[1] = freq & 0xff;
    
    tea[2] = 0x10;
    tea[3] = 0x10;
    tea[4] = 0;//high ingection
    return I2C1_Send(0xC0, tea,sizeof(tea));
}

uint8_t Init_TDA(void)
{
    uint8_t init_buff[19];
    
    init_buff[0] = 0x20; // AI 1 + Subaddres 00000 - Main source sel
		init_buff[1] = (STATE == MAIN) ? TDA_inputs[INPUT_SEL] : TDA_SOURCE_MUTE;// Main source = SE2(FM), gain = 0;
    init_buff[2] = 0x00; // Loudless off
    init_buff[3] = 0xC7; // CLK FM off, SM step 2.56, 0.96, I2C, off
    init_buff[4] = VOLUME; // VOL 0;
    init_buff[5] = 0x90; // Ref out ext + treble off;
    init_buff[6] = 0x50; // mid off
    init_buff[7] = 0x50; // bass off
    init_buff[8] = 0x05; // sec source - mute, rear - main
    init_buff[9] = 0x00; // sub off all
    init_buff[10]= 0xFF; // mix off;
    init_buff[11]= 0x00;
    init_buff[12]= 0x00;
    init_buff[13]= 0x00;
    init_buff[14]= 0x00;
    init_buff[15]= 0x00;
    init_buff[16]= 0x00;
    init_buff[17]= 0x39; // SA on
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
	flash_erase_sector(3); //start from 0x0800C000
	flash_write(MEM_ADDRESS, ((STATE << 24 ) | (INPUT_SEL << 16) | (0xFF00 & (VOLUME << 8 ))));
	flash_write(RADIO_FREQ_ADR, RADIO_FREQ << 16);

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
    TIM3->ARR = 100;
    TIM3->CR2 |= TIM_CR2_MMS_1;
    TIM3->CR1 |= TIM_CR1_CEN;
}
