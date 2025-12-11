
#include <driver/gpio.h>

#include "constants.h"
#include "hardware/ahrs/mpu9250/mpu9250.h"
#include "hardware/ahrs/bmp280.h"

namespace pizda {
	class AHRS {
		public:
			AHRS(gpio_num_t mpu9250ssPin, gpio_num_t bmp280ssPin) :
				mpu9520(MPU9250 {
					constants::spi::miso,
					constants::spi::mosi,
					constants::spi::sck,
					mpu9250ssPin
				}),
				bmp280(BMP280 {
					constants::spi::miso,
					constants::spi::mosi,
					constants::spi::sck,
					bmp280ssPin
				})
			{

			}

			void setup() {
				mpu9520.setup();

				bmp280.setup();
				bmp280.configure(
					BMP280Mode::Normal,
					BMP280Oversampling::X2,
					BMP280Oversampling::X16,
					BMP280Filter::X16,
					BMP280StandbyDuration::Ms125
				);

				xTaskCreate(
					[](void* arg) {
						reinterpret_cast<AHRS*>(arg)->taskBody();
					},
					"ahrs",
					2048,
					this,
					10,
					nullptr
				);
			}

		private:
			MPU9250 mpu9520;
			BMP280 bmp280;

			void taskBody() {
				while (true) {
					const auto pressure = bmp280.readPressure();
					const auto temperature = bmp280.readTemperature();

					ESP_LOGI("AHRS BMP", "Pressure: %f, temp: %f", pressure, temperature);

					vTaskDelay(pdMS_TO_TICKS(1000));
				}
			}
	};
}