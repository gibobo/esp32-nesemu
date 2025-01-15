#include "esp_event.h"
#include "nofrendo.h"
#include <stdio.h>

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

int app_main(void)
{
	printf("NoFrendo start!\n");
	nofrendo_main(0, NULL);
	printf("NoFrendo died? WtF?\n");
	asm("break.n 1");
    return 0;
}

