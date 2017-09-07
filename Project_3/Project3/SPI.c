#include "MKL25Z4.h"
#include "nrf.h"
#define dataLen 3


void SPI0_init(void);
uint8_t SPI0_write(unsigned char data);
/*
void TPM0_IRQHandler(void){
	TPM0->SC|=0x80;				//clear TOF
	count+=1;
}
*/
void TIMER0_init(){
		//__disable_irq();

		SIM->SOPT2|=0x01000000;		//use mcgfllclk
		SIM->SCGC6|=0x01000000;		//enable clock to tpm0
		TPM0->SC=0;					//disable timer
		TPM0->SC=0x06; 				//prescaler to 64
		TPM0->MOD=0x04;
		TPM0->SC|=0x80;				//clear TOF

}

void delay(){
	TPM0->SC|=0x08;				//enable timer
	while((TPM0->SC & 0x80) == 0) { }/* wait until the TOF is set */
	TPM0->SC |= 0x80; /* clear TOF */
}


uint8_t *Write_Nrf(uint8_t ReadWrite, uint8_t reg, uint8_t *val, uint8_t antVal)
{
	__disable_irq();	//disable global interrupt

	if (ReadWrite == W)
	{
		reg = W_REGISTER + reg;
	}


	static uint8_t ret[dataLen];
	delay();
	PTD->PCOR = 0x01;
	delay();
	SPI0_write(reg);
	delay();

	int i;
	for(i=0; i<antVal; i++)
	{
		if (ReadWrite == R && reg != W_TX_PAYLOAD)
		{
			ret[i]=SPI0_write(NOP);
			delay();
		}
		else
		{
			SPI0_write(val[i]);
			delay();
		}
	}
	PTD->PSOR = 0x01; /* make PTD0 idle high */	//CSN IR_High = nrf-chippet slutar lyssna

	__enable_irq();//enable global interrupt

	return ret;
}

int main(void) {
	unsigned char c;
	SPI0_init(); /* enable SPI0 */
	TIMER0_init();
	delay();		//10us delay
	PTD->PCOR = 0x01; /* make PTD0 low */
	delay();		//10us delay
	uint8_t val[5];

	val[0]=0x1E;
    Write_Nrf(W, CONFIG, val, 1);

    Write_Nrf(R, CONFIG, val, 1);



    Write_Nrf(R, STATUS, val, 1);

    Write_Nrf(R, TX_ADDR, val, 5);

    int i;
    				for(i=0; i<5; i++)
    				{
    					val[i]=0x12;	//RF channel registry 0b10101011 x 5
    				}
    Write_Nrf(W, TX_ADDR, val, 5); //0b0010 1010 write registry

    Write_Nrf(R, TX_ADDR, val, 5);

    val[0]=0x07;
    Write_Nrf(W, RF_SETUP, val, 1);


    Write_Nrf(R, RF_SETUP, val, 1);

    Write_Nrf(R, FIFO_STATUS, val, 1);
/*

*/

	}

void SPI0_init(void) {
	SIM->SCGC5 |= 0x1000; /* enable clock to Port D */
	PORTD->PCR[1] = 0x200; /* make PTD1 pin as SPI SCK */
	PORTD->PCR[2] = 0x200; /* make PTD2 pin as SPI MOSI */
	PORTD->PCR[3] = 0x200; /* make PTD3 pin as SPI MISO */
	PORTD->PCR[0] = 0x100; /* make PTD0 pin as GPIO */
	PORTD->PCR[5] = 0x100; /* make PTD5 pin as GPIO */
	PTD->PDDR |= 0x01; /* make PTD0 as output pin for /SS */
	PTD->PDDR |= 0x01<<5; /* make PTD5 as output pin for /SS */
	PTD->PSOR = 0x01; /* make PTD0 idle high */
	PTD->PCOR = 0x01<<5; /* make PTD5 low */
	SIM->SCGC4 |= 0x400000; /* enable clock to SPI0 */
	SPI0->C1 = 0x10; /* disable SPI and make SPI0 master */
	SPI0->BR = 0x90; /* set Baud rate to 1 MHz */
	SPI0->C1 |= 0x40; /* Enable SPI module */
	}

uint8_t SPI0_write(unsigned char data) {
	volatile char dummy;
	//PTD->PCOR = 1; /* assert /SS */
	while(!(SPI0->S & 0x20)) { } /* wait until tx ready */
	SPI0->D = data; /* send data byte */
	while(!(SPI0->S & 0x80)) { } /* wait until tx complete */
	dummy = SPI0->D; /* clear SPRF */
	//PTD->PSOR = 1; /* deassert /SS */
	return dummy;
	}

