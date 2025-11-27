#include "esp_log.h"
#include "constants.h"
#include "lights.h"
#include "esp_timer.h"

using namespace pizda;

Lights lights {};

extern "C" void app_main(void) {
	lights.setup();

	while (true) {
		ESP_LOGI("Main", "pizda");

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
