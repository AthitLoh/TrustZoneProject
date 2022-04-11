/**
 * Copyright (c) 2019 Microchip Technology Inc.
 *
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

//#include "sam.h"
#include "veneer.h"
#include "hmac-sha256.h"

#include <stdbool.h>
#include "saml11e16a.h"
#include "hal_gpio.h"

//-----------------------------------------------------------------------------
#define PERIOD_FAST     100
#define PERIOD_SLOW     500

HAL_GPIO_PIN(LED,      A, 7)

HAL_GPIO_PIN(BUTTON,   A, 27)

HAL_GPIO_PIN(UART_TX,  A, 24)
HAL_GPIO_PIN(UART_RX,  A, 25)

volatile int val1, val2;

#define F_CPU 16000000

//-----------------------------------------------------------------------------
void TC1_Handler(void)
{
	if (TC1->COUNT16.INTFLAG.reg & TC_INTFLAG_MC(1))
	{
		HAL_GPIO_LED_toggle();
		TC1->COUNT16.INTFLAG.reg = TC_INTFLAG_MC(1);
	}
}

//-----------------------------------------------------------------------------
static void timer_set_period(uint16_t i)
{
	TC1->COUNT16.CC[0].reg = (F_CPU / 1000ul / 256) * i;

	TC1->COUNT16.COUNT.reg = 0;
}

//-----------------------------------------------------------------------------
static void timer_init(void)
{
	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC1;

	GCLK->PCHCTRL[TC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN(0) | GCLK_PCHCTRL_CHEN;
	while (0 == (GCLK->PCHCTRL[TC1_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));

	TC1->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV256 |
	TC_CTRLA_PRESCSYNC_RESYNC;

	TC1->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;

	TC1->COUNT16.COUNT.reg = 0;

	timer_set_period(PERIOD_FAST);

	TC1->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

	TC1->COUNT16.INTENSET.reg = TC_INTENSET_MC(1);
	NVIC_EnableIRQ(TC1_IRQn);
}

//-----------------------------------------------------------------------------
static void uart_init(uint32_t baud)
{
	uint64_t br = (uint64_t)65536 * (F_CPU - 16 * baud) / F_CPU;

	HAL_GPIO_UART_TX_out();
	HAL_GPIO_UART_TX_pmuxen(HAL_GPIO_PMUX_C);
	HAL_GPIO_UART_RX_in();
	HAL_GPIO_UART_RX_pmuxen(HAL_GPIO_PMUX_C);

	MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM0;

	GCLK->PCHCTRL[SERCOM0_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN(0) | GCLK_PCHCTRL_CHEN;
	while (0 == (GCLK->PCHCTRL[SERCOM0_GCLK_ID_CORE].reg & GCLK_PCHCTRL_CHEN));

	SERCOM0->USART.CTRLA.reg =
	SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_MODE(1/*USART_INT_CLK*/) |
	SERCOM_USART_CTRLA_RXPO(3/*PAD3*/) | SERCOM_USART_CTRLA_TXPO(1/*PAD2*/);

	SERCOM0->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN |
	SERCOM_USART_CTRLB_CHSIZE(0/*8 bits*/);

	SERCOM0->USART.BAUD.reg = (uint16_t)br;

	SERCOM0->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
	
	while (SERCOM0->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_ENABLE);
}

//-----------------------------------------------------------------------------
static void uart_putc(char c)
{
	while (!(SERCOM0->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE));
	SERCOM0->USART.DATA.reg = c;
}

//-----------------------------------------------------------------------------
static void uart_puts(char *s)
{
	while (*s)
	uart_putc(*s++);
}

static void uart_puti(int i)
{
	char s[11];
	int last = 0;
	do {
		s[last] = i%10 + '0';
		i = i/10;
		last++;
	} while(i > 0);
	
	for(int j=last-1; j>=0; j--) {
		uart_putc(s[j]);
	}
}


static char uart_getc()
{
	char rx;
	while(!(SERCOM0->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC))   /* wait until Rx full */
	{
		if(SERCOM0->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_ERROR)
		{
			SERCOM0->USART.INTFLAG.reg |= SERCOM_USART_INTFLAG_ERROR; /* clear any errors */
			break;
		}
	}
	rx = (uint8_t)SERCOM0->USART.DATA.reg;
	return rx;
}

static void uart_gets(char* s) {
	int idx=0;
	while(true) {
		char c = uart_getc();
		uart_putc(c);
		// enter key
		if(c==13) {
			uart_putc('A');
			s[idx] = 0;
			break;
		}
		s[idx] = c;
		idx++;
	}
}

static int uart_geti() {
	char s[10];
	uart_gets(&s);
	int out;
	sscanf(s, "%d", &out);
	return out;
}


/* Non-Secure main() */
int main(int argc, const char *const *argv)
{
	
	timer_init();
	
	uart_init(115200);
	
	
	
	/* Call Non-Secure Callable Flash (APPLICATION region) function 1 */
	uart_puts("Enter number: ");
	int in = uart_geti();
	uart_puts("\r\n");
	val1 = secure_func1 (in);
	
	/* Call Non-Secure Callable Flash (APPLICATION region) function 2 */
	val2 = secure_func2 (in);
	
	char msg[64] = "Hi There";
	unsigned char sha256sum[64]; 
	
	/*sha256_context ctx;
	sha256_starts(&ctx);
	sha256_update(&ctx, (uint8 *) msg, strlen(msg));
	sha256_finish(&ctx, sha256sum);*/
	
	hmac_sha256 hmac;
	//sha256 sha;
	uint8_t key[20];
	int i = 0;
	for (i = 0; i < 20; i++)
	{
		key[i] = 0x0b;
	}
	//hmac_sha256_initialize(&hmac, key, 20);
	//hmac_sha256_update(&hmac, (uint8_t *) msg, 20);
	//hmac_sha256_finalize(&hmac, (uint8_t *) msg, strlen(msg));
	//hmac_sha256_get(&hmac,(uint8_t *) msg, strlen(msg), key, strlen(key));
	
	/* Print memory address in each element*/
	uint8_t* start_address = 0X00008000;
	//uint8_t* end_address = 0x00009ef8;
	uint8_t* end_address = 0x00008010;
	uint8_t chl[32] = {0};
	
	
	hmac_sha256_initialize(&hmac, key, 20);
	hmac_sha256_update(&hmac, chl, 32);
	hmac_sha256_finalize(&hmac, (uint8_t *) start_address, (((uint32_t) end_address) - ((uint32_t) start_address)));
	
	char output[65] = {0};
	for( int j = 0; j < 32; j++ )
	{
		sprintf( output + j * 2, "%02x", hmac.digest[j] );
		//uart_puts(output);
		//uart_puts(" ");
	}
	uart_puts(output);
	uart_puts("\r\n\r\n");
	
	// output now contains SHA256("hello world") in hex string form
	
	
	/*uint8_t* ram_address = 0x20002000;
	uint8_t value;*/
	/*uint32_t arr = {1,2,3};*/
	/*uint8_t value2;
	value2 = ram_address[0];
	
	ram_address[0] = 222;
	
	uint8_t value3;
	value3 = ram_address[0];*/
	
	//char hex[3];
	//hex[2] = 0;
	//uint32_t k = 0;
	/*sprintf(hex,"%02x",value2);
	uart_puts("Value before:");
	uart_puts(hex);
	sprintf(hex,"%02x",value3);
	uart_puts("Value after:");
	uart_puts(hex);
	uart_puts("\n");*/
	/*for (uint32_t i = (uint32_t) start_address; i <= (uint32_t) end_address; i++){
		//uart_puts("%p",&arr[i]);
		hmac_sha256_initialize(&hmac, key, 20);
		hmac_sha256_finalize(&hmac, (uint8_t *) start_address, strlen(msg));
		i = start_address, value = start_address[0]
		i = start_address+key, value = start_address[key]
		value = start_address[key];
		key++;
		sprintf(hex,"%02x",value);
		uart_puts(hex);
		uart_puts(" ");
		//uart_puts("%02hhX ",&value[i]);
	}*/
	
	uart_puts("\r\nHello, non-secure world!\r\n Output: ");
	uart_puti(&val1);
	uart_puts(" ");
	uart_puti(&val2);
	uart_puts("\r\n non-secure world is going to sleep\n");
	
	HAL_GPIO_LED_out();
	HAL_GPIO_LED_clr();
	
	
	/*while(1) {
		volatile int ii=0;
		for(ii=0; ii<100000; ii++);
		HAL_GPIO_LED_toggle();
		uart_puts("\r\n non-secure world is going to sleep\n");
	}*/
}