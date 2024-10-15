// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "driver/gpio.h"
#include "psxcontroller.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include <stdio.h>

#define NOP() asm volatile("nop")

#if CONFIG_HW_PSX_ENA
#define PSX_CMD_SET() gpio_set_level(CONFIG_HW_PSX_CMD, 1)
#define PSX_CMD_CLR() gpio_set_level(CONFIG_HW_PSX_CMD, 0)
#define PSX_ATT_SET() gpio_set_level(CONFIG_HW_PSX_ATT, 1)
#define PSX_ATT_CLR() gpio_set_level(CONFIG_HW_PSX_ATT, 0)
#define PSX_CLK_SET() gpio_set_level(CONFIG_HW_PSX_CLK, 1)
#define PSX_CLK_CLR() gpio_set_level(CONFIG_HW_PSX_CLK, 0)
#define PSX_DAT_CHK() gpio_get_level(CONFIG_HW_PSX_DAT)

#define CTRL_BYTE_DELAY 18
#define CTRL_CLK 5
#define CTRL_CLK_HIGH 5

unsigned char PS2data[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void delayMicroseconds(uint32_t us)
{
	uint64_t m = (uint64_t)esp_timer_get_time();
	if (us)
	{
		uint64_t e = (m + us);
		if (m > e)
		{
			while ((uint64_t)esp_timer_get_time() > e)
			{
				NOP();
			}
		}
		while ((uint64_t)esp_timer_get_time() < e)
		{
			NOP();
		}
	}
}

/* Sends and receives a byte from/to the PSX controller using SPI */
static unsigned char psxSendRecv(unsigned char send)
{
	unsigned char i;
	unsigned char ret = 0;
	for (i = 0; i < 8; i++)
	{
		if (send & (1 << i))
			PSX_CMD_SET();
		else
			PSX_CMD_CLR();

		PSX_CLK_CLR();
		delayMicroseconds(CTRL_CLK);

		if (PSX_DAT_CHK())
			ret |= (1UL << i);
		PSX_CLK_SET();
		delayMicroseconds(CTRL_CLK_HIGH);
	}
	PSX_CMD_SET();
	delayMicroseconds(CTRL_BYTE_DELAY);
	return ret;
}

unsigned short psxReadInput()
{
	unsigned char ESP2data[] = {0x01, 0x42, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	unsigned char i, RetryCnt;
	for (RetryCnt = 0; RetryCnt < 5; RetryCnt++)
	{
		PSX_CMD_SET();
		PSX_CLK_SET();
		PSX_ATT_CLR();
		delayMicroseconds(CTRL_BYTE_DELAY);
		for (i = 0; i < 9; i++)
		{
			PS2data[i] = psxSendRecv(ESP2data[i]);
		}
		if (PS2data[1] == 0x79)
		{ // if controller is in full data return mode, get the rest of data
			for (i = 9; i < 21; i++)
			{
				PS2data[i] = psxSendRecv(ESP2data[i]);
			}
		}
		PSX_ATT_SET();
		if ((PS2data[1] & 0xf0) == 0x70)
			break;
	}
	if ((PS2data[1] & 0xf0) != 0x70)
		return 0xffff;
	return (unsigned short)(PS2data[4] << 8) + PS2data[3];
}

void psxcontrollerInit()
{
	gpio_config_t gpioconf[2] = {
		{.pin_bit_mask = BIT(CONFIG_HW_PSX_CLK) | BIT(CONFIG_HW_PSX_CMD) | BIT(CONFIG_HW_PSX_ATT),
		 .mode = GPIO_MODE_OUTPUT,
		 .pull_up_en = GPIO_PULLUP_DISABLE,
		 .pull_down_en = GPIO_PULLDOWN_DISABLE,
		 .intr_type = GPIO_PIN_INTR_DISABLE},
		{.pin_bit_mask = BIT(CONFIG_HW_PSX_DAT),
		 .mode = GPIO_MODE_INPUT,
		 .pull_up_en = GPIO_PULLUP_ENABLE,
		 .pull_down_en = GPIO_PULLDOWN_DISABLE,
		 .intr_type = GPIO_PIN_INTR_DISABLE}};
	gpio_config(&gpioconf[0]);
	gpio_config(&gpioconf[1]);
	PSX_CMD_SET();
	PSX_CLK_SET();
	psxReadInput();
	psxReadInput();

	if (PS2data[1] != 0x41 && PS2data[1] != 0x42 && PS2data[1] != 0x73 && PS2data[1] != 0x79)
	{
		printf("No PSX/PS2 controller detected (0x%X). You will not be able to control the game.\n", PS2data[1]);
	}
	else
	{
		printf("PSX controller type 0x%X\n", PS2data[1]);
	}
}

#else

unsigned short psxReadInput()
{
	return 0xFFFF;
}

void psxcontrollerInit()
{
	printf("PSX controller disabled in menuconfig; no input enabled.\n");
}

#endif