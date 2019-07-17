#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include <stdlib.h>


#define LED_PWM         (1 << 8)                                        /* Port D, pin 8                                    */
#define LEDC            30                                              /* Count of LEDs in strip (should be 1 or greater)  */
#define PORTOUT         GPIOD                                           /* Port for WS2812B                                 */
#define ClearOutBit     (PORTOUT->ODR = 0)                              /* 0 to output                                      */
#define SetOutBit       (PORTOUT->ODR = LED_PWM)                        /* 1  to output                                     */

#define RED             0x0F                                            /* Red initialization                               */
#define GREEN           0xF0                                            /* Green initialization                             */
#define BLUE            0x55                                            /* Blue initialization                              */

#define DWT_CYCCNT      (*(volatile unsigned long *)0xE0001004)         /* Macros for delay function                        */
#define DWT_CONTROL     (*(volatile unsigned long *)0xE0001000)
#define SCB_DEMCR       (*(volatile unsigned long *)0xE000EDFC)         /*                                                  */
    
	
unsigned char ledred[LEDC+1] ;                                          /* Array of red                                     */
unsigned char ledblue[LEDC+1] ;                                         /* Array of blue                                    */
unsigned char ledgreen[LEDC+1];                                         /* Array of green                                   */

/* Setup pin for LED strip */
static inline void setup_pin(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;                                /* Enable clocking of port D (PWM LED)              */
    PORTOUT->MODER |=  GPIO_MODER_MODER8_0;                             /* Enable high level for LED                        */
}

/* Delay initialization */
void DWT_Init(void)
{
    SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;                            /* Allow to use the counter                         */
    DWT_CYCCNT  = 0;                                                    /* Reset the counting register                      */
    DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;                              /* Start the counter                                */
}

/* Delta for delay */
static __inline uint32_t delta(uint32_t t0, uint32_t t1)
{
    return (t1 - t0); 
}
/* delay_us */
void delay_us(uint32_t us)
{
    uint32_t t0 =  DWT->CYCCNT;
    uint32_t us_count_tic =  us * (SystemCoreClock/1000000);
    while (delta(t0, DWT->CYCCNT) < us_count_tic) ;
}

/* delay_ms */
void delay_ms(uint32_t ms)
{
    uint32_t t0 =  DWT->CYCCNT;
    uint32_t us_count_tic =  ms * (SystemCoreClock/1000);
    while (delta(t0, DWT->CYCCNT) < us_count_tic) ;
}

/* Load to strip data array */
void loadWS2812B (void)
{
		
	unsigned	char a,b,temp;
	/* Reset formation  */
	PORTOUT->ODR = 0;
	__nop();

	for(a=0;a<LEDC;a++)
	{
		for(b=0;b<3;b++)
		{
			switch (b)
			{
			case 0 :
				temp=ledgreen[a];
				break;
			case 1 :
				temp=ledred[a];
				break;
			case 2 :
				temp=ledblue[a];
				break;
			default:
				temp = 0;
			} 
			/* Byte loading */
			if(temp&0x80)
			{
				/* Formation of bit 1 */
				SetOutBit;
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				ClearOutBit;
				__nop();
			}
			else
			{
				/* Formation of bit 0 */
				SetOutBit;
				__nop();__nop();__nop();__nop();
				ClearOutBit;
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
			}
			
			if(temp&0x40)
			{
				/* Formation of bit 1 */
				SetOutBit;
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				ClearOutBit;
				__nop();
			}
			else
			{
				/* Formation of bit 0 */
					SetOutBit;
				__nop();__nop();__nop();__nop();
					ClearOutBit;
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
			}
			
			if(temp&0x20)
			{
				/* Formation of bit 1 */
				SetOutBit;
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				ClearOutBit;
				__nop();
			}
			else
			{
				/* Formation of bit 0 */
					SetOutBit;
				__nop();__nop();__nop();__nop();
					ClearOutBit;
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
			}
			if(temp&0x10)
			{
				/* Formation of bit 1 */
				SetOutBit;
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				ClearOutBit;
				__nop();
			}
			else
			{
				/* Formation of bit 0 */
					SetOutBit;
				__nop();__nop();__nop();__nop();
					ClearOutBit;
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
			}
			if(temp&0x8)
			{
				/* Formation of bit 1 */
				SetOutBit;
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				ClearOutBit;
				__nop();
			}
			else
			{
				/* Formation of bit 0 */
					SetOutBit;
				__nop();__nop();__nop();__nop();
					ClearOutBit;
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
			}
			
			if(temp&0x4)
			{
				/* Formation of bit 1 */
				SetOutBit;
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				ClearOutBit;
				__nop();
			}
			else
			{
				/* Formation of bit 0 */
					SetOutBit;
				__nop();__nop();__nop();__nop();
					ClearOutBit;
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
			}
			if(temp&0x2)
			{
				/* Formation of bit 1 */ 
				SetOutBit;
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				ClearOutBit;
				__nop();
			}
			else
			{
				/* Formation of bit 1 */
					SetOutBit;
				__nop();__nop();__nop();__nop();
					ClearOutBit;
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
			}
			
			if(temp&0x1)
			{
				/* Formation of bit 1 */
				SetOutBit;
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				ClearOutBit;
				__nop();
			}
			else
			{
				/* Formation of bit 0 */
					SetOutBit;
				__nop();__nop();__nop();__nop();
					ClearOutBit;
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
			}
		}
    }
}

/* Clear array in LED strip */
void clear_LED()
{
    for (int i = 0; i < LEDC; i++)
	{
		ledred[i] =   0;
		ledblue[i] =  0;
		ledgreen[i] = 0;
		delay_us(500);
  }
	loadWS2812B();
}

/* Move effects */
void CometWhite (void)
{
	/* Forward movement */
	unsigned char i = 5;
	do
	{	
		ledred[i] =    255;
		ledblue[i] =   255;
		ledgreen[i] =  255;

		ledred[i-1] =   255/4;
		ledblue[i-1] =  255/4;
		ledgreen[i-1] = 255/4;
		
		ledred[i-2] =   255/8;
		ledblue[i-2] =  255/8;
		ledgreen[i-2] = 255/8;
		
		ledred[i-3] =   255/16;
		ledblue[i-3] =  255/16;
		ledgreen[i-3] = 255/16;
		
		ledred[i-4] =   255/32;
		ledblue[i-4] =  255/32;
		ledgreen[i-4] = 255/32;
		
		ledred[i-5] =   0;
		ledblue[i-5] =  0;
		ledgreen[i-5] = 0;
		
		delay_ms(80);
		i++;

		loadWS2812B();
} while (i<=LEDC);
	
	/* Disabling last 4 LED */
	ledred[LEDC-1] =   0;
	ledblue[LEDC-1] =  0;
	ledgreen[LEDC-1] = 0;
	
	ledred[LEDC-2] =   0;
	ledblue[LEDC-2] =  0;
	ledgreen[LEDC-2] = 0;
	
	ledred[LEDC-3] =   0;
	ledblue[LEDC-3] =  0;
	ledgreen[LEDC-3] = 0;
	
	ledred[LEDC-4] =   0;
	ledblue[LEDC-4] =  0;
	ledgreen[LEDC-4] = 0;
}

void moveRed (void)		
{
    clear_LED();
	/* Forward movement */
	unsigned char i = 0;
	do
	{	ledred[i] =  250;
		ledblue[i] =   0;
		ledgreen[i] =  0;
		delay_ms(30);
			
		ledred[i-1] =  0;
		ledblue[i-1] = 0;
		ledgreen[i-1]= 0;
		delay_ms(30);
		i++;
			
		loadWS2812B();
	} while (i<=LEDC);
		
	/* Back movement */
	i = LEDC;
	do
	{	ledred[i-1] = 250;
		ledblue[i-1] =   0;
		ledgreen[i-1] =  0;
		delay_ms(15);

		ledred[i] =      0;
		ledblue[i] =     0;
		ledgreen[i] =    0;
		delay_ms(15);
		i--;
		loadWS2812B();
	} while (i>0);

    ledred[0] =      0;
    ledblue[0] =     0;
    ledgreen[0] =    0;
}

void moveWhite (void)
{
    ledred[0] =  0;
    ledblue[0] =  0;
    ledgreen[0] =  0;
	
	/* Forward moving */
	unsigned char i = 2;
	do
	{	ledred[i] =  255;
		ledblue[i] =  255;
		ledgreen[i] =  255;
		delay_ms(30);
		
		ledred[i-1] =   0;
		ledblue[i-1] =  0;
		ledgreen[i-1] = 0;
		delay_ms(30);
		i++;
		
		loadWS2812B();
		
	} while (i<=LEDC);

	/* Back moving */
    i = LEDC;
	do
	{	ledred[i-1] =    255;
		ledblue[i-1] =  255;
		ledgreen[i-1] =  255;
		delay_ms(15);

		ledred[i] =   0;
		ledblue[i] =  0;
		ledgreen[i] = 0;
		delay_ms(15);
		i--;
		loadWS2812B();
		
	} while (i>=1);
	ledred[0] =   0;
	ledblue[0] =  0;
	ledgreen[0] = 0;
}	

void lightningBlueGreen (void)
{
	unsigned char i = 0;
	do
	{	ledred[i] =   0;
		ledblue[i] =  255;
		ledgreen[i] = 255;
		delay_ms(100);

		i++;
		loadWS2812B();
		
	} while (i<=LEDC);
}

void ColorLight (void)
{
    unsigned char n = 0;
	do
	{	unsigned char i = 0;
		unsigned char j = 1;
		unsigned char k = 2;

		do
		{	ledred[k] =   0;
			ledblue[i] =  0;
			ledgreen[j] = 0;
		
			ledred[i] =   255;
			ledblue[j] =  255;
			ledgreen[k] = 255;
			delay_us(100);

			i=i+3;
			j=j+3;
			k=k+3;

			loadWS2812B();
		
		} while (i<=LEDC);

		i = 0;
		j = 1;
		k = 2;
	
		do
		{	ledred[i] =   0;
			ledblue[j] =  0;
			ledgreen[k] = 0;

			ledred[j] =   255;
			ledblue[k] =  255;
			ledgreen[i] = 255;
			delay_us(100);

			i=i+3;
			j=j+3;
			k=k+3;
		
			loadWS2812B();
		} while (i<=LEDC);

		i = 0;
		j = 1;
		k = 2;
		do
		{	ledred[j] =   0;
			ledblue[k] =  0;
			ledgreen[i] = 0;

			ledred[k] =   255;
			ledblue[i] =  255;
			ledgreen[j] = 255;

			delay_us(100);

			i=i+3;
			j=j+3;
			k=k+3;

			loadWS2812B();
		}while (i<=LEDC);
	n++;

	}while (n<=50);
	
	for (int i = 0; i < LEDC; i++)
	{
		ledred[i] =   0;
		ledblue[i] =  0;
		ledgreen[i] = 0;

		loadWS2812B();
	}
}

int main(void)
{
    setup_pin();                    /* LED initialization   */
    DWT_Init();                     /* Delay initialization */
    
    while (1)  
    {		
		CometWhite();
		moveRed();
		moveWhite();
		lightningBlueGreen();
		ColorLight();
    }
}