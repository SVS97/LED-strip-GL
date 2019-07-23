#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include <stdlib.h>


#define led_PWM         (1 << 8)                                        /* Port D, pin 8                                    */
#define LEDC            30                                              /* Count of LEDs in strip (should be 1 or greater)  */
#define PORTOUT         GPIOD                                           /* Port for WS2812B                                 */
#define ClearOutBit     (PORTOUT->ODR = 0)                              /* 0 to output                                      */
#define SetOutBit       (PORTOUT->ODR = led_PWM)                        /* 1  to output                                     */

#define RED             0x0F                                            /* Red initialization                               */
#define GREEN           0xF0                                            /* Green initialization                             */
#define BLUE            0x55                                            /* Blue initialization                              */

#define GREEN_LOAD      0
#define RED_LOAD        1
#define BLUE_LOAD       2

#define COLOURS          3

#define DWT_CYCCNT      (*(volatile unsigned long *)0xE0001004)         /* Macros for delay function                        */
#define DWT_CONTROL     (*(volatile unsigned long *)0xE0001000)
#define SCB_DEMCR       (*(volatile unsigned long *)0xE000EDFC)         /*                                                  */
    
//#define BIT_7        (temp&0x80)                                        /* Data for LED arrays                              */
/*#define BIT_6        (temp&0x40)
#define BIT_5        (temp&0x20)
#define BIT_4        (temp&0x10)
#define BIT_3        (temp&0x8)
#define BIT_2        (temp&0x4)
#define BIT_1        (temp&0x2)
#define BIT_0        (temp&0x1) */                                        /*                                                  */

#define BIT_N_IS_SET(var, num)     ((var) & (1 << num))

unsigned char ledred[LEDC+1] ;                                          /* Array of red                                     */
unsigned char ledblue[LEDC+1] ;                                         /* Array of blue                                    */
unsigned char ledgreen[LEDC+1];                                         /* Array of green                                   */

/* Setup pin for LED strip */
static inline void board_setupPin(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;                                /* Enable clocking of port D (PWM LED)              */
    PORTOUT->MODER |=  GPIO_MODER_MODER8_0;                             /* Enable high level for LED                        */
}

/* Delay initialization */
void board_dwtInit(void)
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
/* board_delay_us */
void board_delay_us(uint32_t us)
{
    uint32_t t0 =  DWT->CYCCNT;
    uint32_t us_count_tic =  us * (SystemCoreClock/1000000);
    while (delta(t0, DWT->CYCCNT) < us_count_tic) ;
}

/* board_delay_ms */
void board_delay_ms(uint32_t ms)
{
    uint32_t t0 =  DWT->CYCCNT;
    uint32_t us_count_tic =  ms * (SystemCoreClock/1000);
    while (delta(t0, DWT->CYCCNT) < us_count_tic) ;
}

/* Load to strip data array */
void led_load (void)
{
    unsigned char a,b,temp;
    /* Reset formation  */
    PORTOUT->ODR = 0;
    __nop();

    for(a = 0; a < LEDC; a++)
    {
        for(b = 0; b < COLOURS; b++)
        {
            switch (b)
            {
            case GREEN_LOAD :
                temp=ledgreen[a];
                break;
            case RED_LOAD :
                temp=ledred[a];
                break;
            case BLUE_LOAD :
                temp=ledblue[a];
                break;
            default:
                temp = 0;
            } 
            /* Byte loading */
            if(BIT_N_IS_SET(temp, 7))
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

            if(BIT_N_IS_SET(temp, 6))
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
            
            if(BIT_N_IS_SET(temp, 5))
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
            if(BIT_N_IS_SET(temp, 4))
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
            if(BIT_N_IS_SET(temp, 3))
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
            
            if(BIT_N_IS_SET(temp, 2))
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
            if(BIT_N_IS_SET(temp, 1))
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
            
            if(BIT_N_IS_SET(temp, 0))
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
void led_clear()
{
    for (int i = 0; i < LEDC; i++)
    {
        ledred[i] =   0;
        ledblue[i] =  0;
        ledgreen[i] = 0;
        board_delay_us(500);
    }
    led_load();
}

/* Move effects */
void led_cometWhite (void)
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
        
        board_delay_ms(80);
        i++;

        led_load();
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

void led_moveRed (void)     
{
    led_clear();
    /* Forward movement */
    unsigned char i = 0;
    do
    {   ledred[i] =  250;
        ledblue[i] =   0;
        ledgreen[i] =  0;
        board_delay_ms(30);
            
        ledred[i-1] =  0;
        ledblue[i-1] = 0;
        ledgreen[i-1]= 0;
        board_delay_ms(30);
        i++;
            
        led_load();
    } while (i<=LEDC);
        
    /* Back movement */
    i = LEDC;
    do
    {   ledred[i-1] = 250;
        ledblue[i-1] = 0;
        ledgreen[i-1] = 0;
        board_delay_ms(15);

        ledred[i] = 0;
        ledblue[i] = 0;
        ledgreen[i] = 0;
        board_delay_ms(15);
        i--;
        led_load();
    } while (i>0);

    ledred[0] = 0;
    ledblue[0] = 0;
    ledgreen[0] = 0;
}

void led_moveWhite (void)
{
    ledred[0] =  0;
    ledblue[0] =  0;
    ledgreen[0] =  0;

    /* Forward moving */
    unsigned char i = 2;
    do
    {   ledred[i] =  255;
        ledblue[i] =  255;
        ledgreen[i] =  255;
        board_delay_ms(30);
        
        ledred[i-1] =   0;
        ledblue[i-1] =  0;
        ledgreen[i-1] = 0;
        board_delay_ms(30);
        i++;

        led_load();

    } while (i<=LEDC);

    /* Back moving */
    i = LEDC;
    do
    {   ledred[i-1] =    255;
        ledblue[i-1] =  255;
        ledgreen[i-1] =  255;
        board_delay_ms(15);

        ledred[i] =   0;
        ledblue[i] =  0;
        ledgreen[i] = 0;
        board_delay_ms(15);
        i--;
        led_load();
        
    } while (i>=1);
    ledred[0] =   0;
    ledblue[0] =  0;
    ledgreen[0] = 0;
}

void led_lightningBlueGreen (void)
{
    unsigned char i = 0;
    do
    {   ledred[i] =   0;
        ledblue[i] =  255;
        ledgreen[i] = 255;
        board_delay_ms(100);

        i++;
        led_load();

    } while (i<=LEDC);
}

void led_colorLight (void)
{
    unsigned char n = 0;
    do
    {   unsigned char i = 0;
        unsigned char j = 1;
        unsigned char k = 2;

        do
        {   ledred[k] =   0;
            ledblue[i] =  0;
            ledgreen[j] = 0;
        
            ledred[i] =   255;
            ledblue[j] =  255;
            ledgreen[k] = 255;
            board_delay_us(100);

            i=i+3;
            j=j+3;
            k=k+3;

            led_load();
        } while (i<=LEDC);

        i = 0;
        j = 1;
        k = 2;

        do
        {   ledred[i] =   0;
            ledblue[j] =  0;
            ledgreen[k] = 0;

            ledred[j] =   255;
            ledblue[k] =  255;
            ledgreen[i] = 255;
            board_delay_us(100);

            i=i+3;
            j=j+3;
            k=k+3;

            led_load();
        } while (i<=LEDC);

        i = 0;
        j = 1;
        k = 2;
        do
        {   ledred[j] =   0;
            ledblue[k] =  0;
            ledgreen[i] = 0;

            ledred[k] =   255;
            ledblue[i] =  255;
            ledgreen[j] = 255;

            board_delay_us(100);

            i=i+3;
            j=j+3;
            k=k+3;

            led_load();
        }while (i<=LEDC);
        n++;

    }while (n<=50);

    for (int i = 0; i < LEDC; i++)
    {
        ledred[i] =   0;
        ledblue[i] =  0;
        ledgreen[i] = 0;

        led_load();
    }
}

int main(void)
{
    board_setupPin();                    /* LED initialization   */
    board_dwtInit();                     /* Delay initialization */
    
    while (1)  
    {
        led_cometWhite();
        led_moveRed();
        led_moveWhite();
        led_lightningBlueGreen();
        led_сolorLight();
    }
}
