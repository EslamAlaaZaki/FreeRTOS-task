#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include "tm4c123gh6pm.h"

uint32_t SystemCoreClock = 16000000;
uint32_t displayed_value;

void PortB_Init(void)
    {
        uint32_t volatile delay=1; // this variable is using to make delay when connecting bus's clock to the port
        SYSCTL_RCGCGPIO_R |=0x02;  // connect port B to bus's clock
        delay=2; 
			  delay=3; 
        GPIO_PORTB_AMSEL_R = 0;  // disable analog function
        GPIO_PORTB_PCTL_R = 0;   // clear pctl for in/out digital pins
			  GPIO_PORTB_DIR_R |=0xFF; // make pins digital output
        GPIO_PORTB_AFSEL_R =0;         // clear alternative function
        GPIO_PORTB_DEN_R |= 0xFF;           // enable digital function 
			  
    }
void writeDataOnSevenSegment(uint32_t value)
{ 
	uint32_t temp=value%10;
	value/=10;
	value=value<<4;
	value+=temp;
	GPIO_PORTB_DATA_R=value;
	
}

void systick_init(void)
{
    NVIC_ST_CTRL_R = 0;
    NVIC_ST_RELOAD_R = 0x00FFFFFF;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R = 0x05;
}
void systick_wait(uint32_t delay)
{
    NVIC_ST_RELOAD_R = delay-1;
    NVIC_ST_CURRENT_R = 0;
    while(!(NVIC_ST_CTRL_R&0x00010000)){}
}
void systick_wait_1ms(int delay)
{
    uint32_t volatile i;
    for(i=0;i<delay;i++)
    {
        systick_wait(16000);
    }
}

void initUART0(uint32_t buadrate)
{
    SYSCTL_RCGCGPIO_R |=0x1;  // connect port A to bus's clock
    SYSCTL_RCGCUART_R |=0x1; //enable uart0
	  GPIO_PORTA_AMSEL_R &=~0x03; // disable analog method
    GPIO_PORTA_AFSEL_R|=0x03; // enable alternaive function to PA0 & PA1
    GPIO_PORTA_PCTL_R |=(GPIO_PORTA_PCTL_R &0xFFFFFF00) | 0x0011;  // select uart function to PA0 & PA1
    GPIO_PORTA_DEN_R |=0x03; // enable degital function
	
    UART0_CTL_R=~1; // disable uart0 to make configration 
    UART0_IBRD_R=(uint32_t)(SystemCoreClock/(16*buadrate));
    UART0_FBRD_R= (uint32_t)(((float)SystemCoreClock/(16.0*buadrate)) +0.5);
    UART0_LCRH_R |=0x70;
    UART0_CTL_R |=0x301;
    //UART0_CC_R=0x00;
}

// reads 1 byte from the uart transmitter
char UART0_receive_byte(void)
{
    while((UART0_FR_R & 0x0010) !=0){}
    return (char)(UART0_DR_R & 0xFF);
}

// sends 1 byte through the uart receiver
void UART0_send_byte(char c)
{
    while((UART0_FR_R & 0x0020)!=0){}
    UART0_DR_R = (uint32_t)c;
}


void ADC0_Init (void)
{
	  uint32_t volatile delay=0;
    SYSCTL_RCGC2_R |=0x0010; // activate clock for port E
	  delay=1;
    delay=2;
    GPIO_PORTE_DIR_R &=~0x10; // set PE4 to input
    GPIO_PORTE_AFSEL_R |= 0x10; //enable alternative function
    GPIO_PORTE_DEN_R &= ~0x10; // disable digital i/o
    GPIO_PORTE_AMSEL_R |=0x10; // enable analog function	
	  SYSCTL_RCGC0_R |=0x00010000; // activate ADC0
	  SYSCTL_RCGC0_R &=~0x300; // configure for 125K
	  ADC0_SSPRI_R= 0x123; //sequencer 3 is highest priority
	  ADC0_ACTSS_R &=~0x8 ; //disable sampler seq3
	  ADC0_EMUX_R &=~0xF000; //seq3 is software trigger
	  ADC0_SSMUX3_R &=~0x000F; // clear ss3 field
	  ADC0_SSMUX3_R +=9; // set channel Ain9 (PE4)
	  ADC0_SSCTL3_R =0x0006; // no TS0 
	  ADC0_IM_R &=~0x0008 ;// disable interrupts
	  ADC0_ACTSS_R |=0x8; // enable sample seq3
}

uint32_t ADC_seq3(void)
{
	uint32_t result ;
	ADC0_PSSI_R= 0x8; // init ss3
	while((ADC0_RIS_R&0x08)==0){}; // wait for conversion
	result=ADC0_SSFIFO3_R & 0xFFF; // read 12 bit 
	ADC0_ISC_R =0x0008; //acknowledge completion
  return result ;		
}

void task1(void* pvParameters)
{
    while(1)
    {
        displayed_value= ((float)(ADC_seq3()*3.3*100.0))/4096;
        vTaskDelay(1000);
    }
}
void task2(void* pvParameters)
{
    while(1)
    {
        writeDataOnSevenSegment(displayed_value);
        vTaskDelay(1000);
    }
}


int main(void)
{
	
    TaskHandle_t  First_handle,Second_handle;
    PortB_Init();
	  writeDataOnSevenSegment(45);
	  /*ADC0_Init();
    systick_init();
    xTaskCreate(task1, "readTemp",128, NULL,2, &First_handle);
	  xTaskCreate(task2, "display",128, NULL,1, &Second_handle);
    vTaskStartScheduler();*/
    while(1)
        {


        }
	return 0;
}
