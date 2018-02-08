#include "main.h"

uint8_t STATE = LOAD;
uint8_t SET_TIME_STATE = SET_TIME_HOUR;
uint8_t INPUT_SEL = FM;

uint8_t OFF_counter = 0;

uint8_t I2C_res;

int main(void)
{
    /*-- Delay for TFT start --*/
    for(uint32_t delay = 0;delay < 1000000;delay++) 
    {};    
        
    /*-- Init main clock --*/
    Init_RCC();
    /*-- Init main clock --*/
    Init_GPIO();
    Init_TFT();
			
    while(DMA1_Stream4->NDTR != 0){};
    loading_txt[8] = '2';
    loading_txt[9] = '5';
    TFT_send(loading_txt, sizeof(loading_txt));
    
    Init_RTC();

    while(DMA1_Stream4->NDTR != 0){};
    loading_txt[8] = '5';
    loading_txt[9] = '0';
    TFT_send(loading_txt, sizeof(loading_txt));
    
    Init_KEYs_TIM();
    
		while(DMA1_Stream4->NDTR != 0){};
    loading_txt[8] = '7';
    loading_txt[9] = '5';
    TFT_send(loading_txt, sizeof(loading_txt));
    
    Init_I2C1();
    
    I2C_res = RDA_set_freq(1040);
    
    I2C_res = Init_TDA();

//    TFT_TIME[14] = ((RTC->TR & 0xF00)   >> 8)  + 0x30;
//    TFT_TIME[13] = ((RTC->TR & 0x7000)  >> 12) + 0x30;
//    
//    TFT_TIME[11] = ((RTC->TR & 0xF0000) >> 16) + 0x30;
//    TFT_TIME[10] = ((RTC->TR & 0x700000)>> 20) + 0x30;
//    
//    TFT_TIME[29] =  (RTC->DR & 0xF)            + 0x30;
//    TFT_TIME[28] = ((RTC->DR & 0x30)    >> 4)  + 0x30;
//    
//    TFT_TIME[44] = ((RTC->DR & 0xF00)   >> 8)  + 0x30;
//    TFT_TIME[43] = ((RTC->DR & 0x1000)  >> 12) + 0x30;
    
  
    STATE = AUDIO_OFF;
    TFT_send(pages[STATE], sizeof(pages[STATE]));
    TFT_send(input_tft[INPUT_SEL], sizeof(input_tft[INPUT_SEL]));
//    TFT_send(TFT_TIME, sizeof(TFT_TIME));
    
    NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
    while(1)
    {
    
    }
}

void Init_RCC(void)
{
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
    
    RCC->CR |= RCC_CR_HSION;

//    do
//    {
//        HSEStatus = RCC->CR & RCC_CR_HSERDY;
//        StartUpCounter++;
//    }    
//    while((HSEStatus == 0) && (StartUpCounter != ((uint32_t)100U)));
    
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
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;
        
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
    static uint32_t time = 0;
    
    del++;
    time++;

    if (del == (SysTicksClk/2)) // Per = 1s
    {
        del = 0;
    }
    
    if (time == (SysTicksClk*5)) // 10s
    {
        time = 0;
//        if((STATE == MAIN)||(STATE == AUDIO_OFF))
//        {
//            TFT_TIME[14] = ((RTC->TR & 0xF00)   >> 8)  + 0x30;
//            TFT_TIME[13] = ((RTC->TR & 0x7000)  >> 12) + 0x30;
//            
//            TFT_TIME[11] = ((RTC->TR & 0xF0000) >> 16) + 0x30;
//            TFT_TIME[10] = ((RTC->TR & 0x700000)>> 20) + 0x30;
//            
//            TFT_TIME[29] =  (RTC->DR & 0xF)            + 0x30;
//            TFT_TIME[28] = ((RTC->DR & 0x30)    >> 4)  + 0x30;
//            
//            TFT_TIME[44] = ((RTC->DR & 0xF00)   >> 8)  + 0x30;
//            TFT_TIME[43] = ((RTC->DR & 0x1000)  >> 12) + 0x30;
//            
//            TFT_send(TFT_TIME, sizeof(TFT_TIME));
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
	
    uint8_t i2c_sel_buff[2];
		
    if(GPIOC->IDR & GPIO_PIN_5)
    {
        OFF_counter++;
        if((OFF_counter >= 20)&&(STATE == MAIN))
        {
            OFF_counter = 20;
            i2c_sel_buff[0] = TDA_MAIN_SOURCE;
            i2c_sel_buff[1] = TDA_SOURCE_MUTE;
            
            I2C_res = I2C1_Send(TDA7419_ADRESS, i2c_sel_buff, sizeof(i2c_sel_buff));
            
            STATE = AUDIO_OFF;
            
            TFT_send(pages[STATE], sizeof(pages[STATE]));
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
                
                i2c_sel_buff[0] = TDA_MAIN_SOURCE;
                i2c_sel_buff[1] = TDA_inputs[INPUT_SEL];
                I2C_res = I2C1_Send(TDA7419_ADRESS, i2c_sel_buff, sizeof(i2c_sel_buff));
                
                TFT_send(input_tft[INPUT_SEL], sizeof(input_tft[INPUT_SEL]));
            }
            if(STATE == AUDIO_OFF)
            {
                STATE = MAIN;
                
                TFT_send(pages[STATE], sizeof(pages[STATE]));

                i2c_sel_buff[0] = TDA_MAIN_SOURCE;
                i2c_sel_buff[1] = TDA_inputs[INPUT_SEL];
                
                I2C_res = I2C1_Send(TDA7419_ADRESS, i2c_sel_buff, sizeof(i2c_sel_buff));
                
                TFT_send(input_tft[INPUT_SEL], sizeof(input_tft[INPUT_SEL]));
            }
        }
        OFF_counter = 0;
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

                                    set_time_txt[84] = day_of_week[week_day][0];
                                    set_time_txt[85] = day_of_week[week_day][1];
                                    set_time_txt[86] = day_of_week[week_day][2];

                                    TFT_send(&set_time_txt[75], 16);
                    
                                    break;
                                   }
                case SET_SET:{
                                    STATE = MAIN;
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
                                    
//                                    TFT_TIME[14] = ((RTC->TR & 0xF00)   >> 8)  + 0x30;
//                                    TFT_TIME[13] = ((RTC->TR & 0x7000)  >> 12) + 0x30;
//                                    
//                                    TFT_TIME[11] = ((RTC->TR & 0xF0000) >> 16) + 0x30;
//                                    TFT_TIME[10] = ((RTC->TR & 0x700000)>> 20) + 0x30;
//                                    
//                                    TFT_TIME[29] =  (RTC->DR & 0xF)            + 0x30;
//                                    TFT_TIME[28] = ((RTC->DR & 0x30)    >> 4)  + 0x30;
//                                    
//                                    TFT_TIME[44] = ((RTC->DR & 0xF00)   >> 8)  + 0x30;
//                                    TFT_TIME[43] = ((RTC->DR & 0x1000)  >> 12) + 0x30;
//                                    
//                                    TFT_send(TFT_TIME, sizeof(TFT_TIME));
                                    
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
            case MAIN:{
                        STATE = SET_TIME;
                        TFT_send(pages[STATE], sizeof(pages[STATE]));
                        
                        SET_TIME_STATE = SET_TIME_HOUR;

                        while(DMA1_Stream4->NDTR != 0){};

                        TFT_send(set_time_tft[SET_TIME_STATE], sizeof(set_time_tft[SET_TIME_STATE]));
                            
                        hour = ((RTC->TR & 0x700000) >> 20)*10 + ((RTC->TR & 0xF0000) >> 16);
                        min  = ((RTC->TR & 0x7000)   >> 12)*10 + ((RTC->TR & 0xF00)   >> 8);
                        day  = ((RTC->DR & 0x30)     >>  4)*10 +  (RTC->DR & 0xF);
                        mon  = ((RTC->DR & 0x1000)   >> 12)*10 + ((RTC->DR & 0xF00)   >> 8);
                        year = ((RTC->DR & 0xF00000) >> 20)*10 + ((RTC->DR & 0xF0000) >> 16);
                        week_day = RTC->DR & 0xE000 >> 13;
                            
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
                        
                        set_time_txt[84] = day_of_week[week_day][0];
                        set_time_txt[85] = day_of_week[week_day][1];
                        set_time_txt[86] = day_of_week[week_day][2];                        
                            
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

    if((STATE == MAIN)||(STATE == AUDIO_OFF))
    {
        TFT_TIME[14] = ((RTC->TR & 0xF00)   >> 8)  + 0x30; // minute
        TFT_TIME[13] = ((RTC->TR & 0x7000)  >> 12) + 0x30;
        
        TFT_TIME[11] = ((RTC->TR & 0xF0000) >> 16) + 0x30; // time
        TFT_TIME[10] = ((RTC->TR & 0x700000)>> 20) + 0x30;
        
        TFT_TIME[30] =  (RTC->DR & 0xF)            + 0x30; // day
        TFT_TIME[29] = ((RTC->DR & 0x30)    >> 4)  + 0x30;
        
        TFT_TIME[33] = ((RTC->DR & 0xF00)   >> 8)  + 0x30; // month
        TFT_TIME[32] = ((RTC->DR & 0x1000)  >> 12) + 0x30;
        
        TFT_TIME[36] = ((RTC->DR & 0xF0000) >> 16) + 0x30; // year
        TFT_TIME[35] = ((RTC->DR & 0xF00000)>> 20) + 0x30;
        
        TFT_TIME[38] = day_of_week[((RTC->DR & 0xE000)   >> 13) - 1][0]; // day of week
        TFT_TIME[39] = day_of_week[((RTC->DR & 0xE000)   >> 13) - 1][1];
        TFT_TIME[40] = day_of_week[((RTC->DR & 0xE000)   >> 13) - 1][2];
        
        TFT_send(TFT_TIME, sizeof(TFT_TIME));
    }

}

void Init_GPIO(void)
{
    RCC-> AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                     RCC_AHB1ENR_GPIOBEN |
                     RCC_AHB1ENR_GPIOCEN;
    
    //Set GPIOC PIN10 as usart4 TX
    GPIOA->AFR[0] |= GPIO_AF8_UART4 << PIN0*4;
    GPIOA->PUPDR |= GPIO_PULLUP << PIN0*2;
    GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN0*2;
    GPIOA->MODER |= GPIO_MODE_AF_PP << PIN0*2;
    
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
    while(DMA1_Stream4->NDTR != 0){};
		
//		DMA1->HIFCR |= DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CFEIF4;
			
    DMA1_Stream4->M0AR = (uint32_t) buff;
    DMA1_Stream4->NDTR = size;
    DMA1->HIFCR = DMA_HIFCR_CTCIF4 |
                  DMA_HIFCR_CHTIF4 | 
                  DMA_HIFCR_CFEIF4 |
                  DMA_HIFCR_CTEIF4;
    DMA1_Stream4->CR |= DMA_SxCR_EN;
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
    init_buff[1] = 0x87; // Main source = SE2(FM), gain = 0;
    init_buff[2] = 0x00; // Loudless off
    init_buff[3] = 0xC7; // CLK FM off, SM step 2.56, 0.96, I2C, off
    init_buff[4] = 0x10; // VOL 0;
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
    
    return I2C1_Send(TDA7419_ADRESS, init_buff, sizeof(init_buff));
}
