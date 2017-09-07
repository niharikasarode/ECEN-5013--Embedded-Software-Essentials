/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved. */


#include "MKL25Z4.h"
 #include <stdio.h>
#include <string.h>
#include <stdint.h>


#define max_length 5000
#define number_bytes 0x64

uint8_t string1[max_length];
uint8_t string2[max_length];
uint8_t array0[] = {0};
uint8_t *str1, *str2;
int time,count;

#define word  1





int8_t my_memmove(uint8_t *m1, uint8_t *m2, uint32_t u);

void dma_memmove(uint8_t *arr1, uint8_t *arr2, uint32_t bytes);
void dma_memzero(uint8_t *arr1, uint32_t length2clear);


void UART0_initw(void);
void DMA_init(void);
void TIMER0_init();
void send (char c);

int8_t my_memmove(uint8_t *m1, uint8_t *m2, uint32_t u)
{
				int i;

			if (( m1 + u - 1) > m2)
			{
				for(i=u; i>=0 ;i--)
				{
				*(m2+i)=*(m1+i);
				}
			}

			else if (( m2 + u - 1) > m1)
			    {

				for(i=0; i<u ;i++)
				{
				*(m2+i)=*(m1+i);
				}

				}

}

void UART0_initw(void)
{


	SIM->SCGC4 |= 0x0400; /* enable clock for UART0 */
	SIM->SOPT2 |= 0x04000000; /* use FLL output for UART Baud rate generator*/



    UART0->C2 = 0; /* turn off UART0 while changing configurations */
    UART0->BDH = 0x00;
    UART0->BDL = 0x17; /* 115200 Baud */
    UART0->C4 = 0x0F; /* Over Sampling Ratio 16 */
    UART0->C1 = 0x00; /* 8-bit data */
    UART0->C2 = 0x08; /* enable transmit */

    SIM->SCGC5 |= 0x0200; /* enable clock for PORTA */
    PORTA->PCR[2] = 0x0200; /* make PTA2 UART0_Tx pin */
}



void DMA_init(void)
{
	SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
	SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;

	// DMAMUX0->CHCFG[0] = 0xBE;  //Enable channel 0
     DMAMUX0->CHCFG[0] = 0x3E;  //Select Port C as source

#ifdef word
	DMA0->DMA[0].DCR =  0x80480000;
#else
	DMA0->DMA[0].DCR =  0x805a0000;
#endif






}


void dma_memmove(uint8_t *arr1, uint8_t *arr2, uint32_t bytes)
		{

	   DMA_init();

	   DMA0->DMA[0].DSR_BCR |= bytes;

	   DMA0->DMA[0].SAR =(uint32_t) arr1;// set source address
	   DMA0->DMA[0].DAR = (uint32_t)arr2; // set dest address


	   NVIC_EnableIRQ(DMA0_IRQn);
       TPM0->SC|=0x08;
	   DMA0->DMA[0].DCR |=DMA_DCR_START_MASK;//start transfer
       NVIC->ISER[0] = 0x00000001;

		}

void dma_memzero(uint8_t *arr1, uint32_t length2clear)
{
	DMA_init();
    DMA0->DMA[0].DSR_BCR |= length2clear;

#ifdef word
	DMA0->DMA[0].DCR =  0x80080000;
#else
	DMA0->DMA[0].DCR =  0x801a0000;
#endif


    DMA0->DMA[0].SAR =(uint32_t) &array0;// set source address
    DMA0->DMA[0].DAR = (uint32_t)arr1; // set dest address

    NVIC_EnableIRQ(DMA0_IRQn);
    TPM0->SC|=0x08;
    DMA0->DMA[0].DCR |=DMA_DCR_START_MASK;//start transfer
    NVIC->ISER[0] = 0x00000001;
}


void send (char c)
{
	while(!(UART0->S1 & 0x80)) { }
		    UART0->D = c;                /* send a char */


}

void DMA0_IRQHandler(void)
{


	TPM0->SC=0;					//disable timer
	time=3*((count*65536)+TPM0->CNT);

	DMA0->DMA[0].DSR_BCR |= DMA_DSR_BCR_DONE_MASK ;

}


void TPM0_IRQHandler(void){
	TPM0->SC|=0x80;				//clear TOF
	count+=1;
}

void TIMER0_init(){
		__disable_irq();

		SIM->SOPT2|=0x01000000;		//use mcgfllclk
		SIM->SCGC6|=0x01000000;		//enable clock to tpm0
		TPM0->SC=0;					//disable timer
		TPM0->SC=0x06; 				//prescaler to 64
		TPM0->MOD=0xFFFF;
		TPM0->SC|=0x80;				//clear TOF
		TPM0->SC|=0x40;				//enable timeout interrupt
		NVIC->ISER[0]|=0x00020000;
		__enable_irq();
}



int main(void)
{

	UART0_initw();

    str1 = string1;
    str2 = string2;
    TIMER0_init();

    for (int k =0; k< max_length; k++)
    {
    	string1[k] = k;
    }




   //dma_memmove(string1, string2, number_bytes);

    dma_memzero(string1, number_bytes);

while(1);




}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

