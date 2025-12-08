#include "lights.h"

#include <esp_timer.h>
#include "rc.h"

namespace pizda {
	void NavLightsChannelBinding::onChannelValueChanged(bool value) {
		RC::getInstance().lights.setNavigationEnabled(value);
	}

	void StrobeLightsChannelBinding::onChannelValueChanged(bool value) {
		RC::getInstance().lights.setStrobeEnabled(value);
	}

	void LandingLightsChannelBinding::onChannelValueChanged(bool value) {
		RC::getInstance().lights.setLandingEnabled(value);
	}

	void CabinLightsChannelBinding::onChannelValueChanged(bool value) {
		RC::getInstance().lights.setCabinEnabled(value);
	}

	void Lights::setup() {
		cabin.fill(0x00);
		cabin.flush();

//				tail.fill(0x00);
//				tail.flush();

		leftWing.fill(0x00);
		leftWing.flush();
	}

	void Lights::start() {
		xTaskCreate(taskBody, "lights", 2048, this, tskIDLE_PRIORITY, &taskHandle);
	}

	bool Lights::isNavigationEnabled() const {
		return navigationEnabled;
	}

	void Lights::setNavigationEnabled(bool value) {
		if (value == navigationEnabled)
			return;

		navigationEnabled = value;
	}

	bool Lights::isStrobeEnabled() const {
		return strobeEnabled;
	}

	void Lights::setStrobeEnabled(bool value) {
		if (value == strobeEnabled)
			return;

		strobeEnabled = value;
	}

	bool Lights::isLandingEnabled() const {
		return landingEnabled;
	}

	void Lights::setLandingEnabled(bool value) {
		if (value == landingEnabled)
			return;

		landingEnabled = value;
	}

	bool Lights::isCabinEnabled() const {
		return cabinEnabled;
	}

	void Lights::setCabinEnabled(bool value) {
		if (value == cabinEnabled)
			return;

		cabinEnabled = value;

		cabin.fill(cabinEnabled ? 0x22 : 0x00);
		cabin.flush();
	}

	void Lights::setPower(uint8_t index, uint16_t value) {

	}

	void Lights::updateNavOrLanding(Light& light, const uint8_t r, const uint8_t g, const uint8_t b) const {
		// Navigation
		if (navigationEnabled) {
			light.fill(r, g, b);
		}
		else {
			light.fill(0x00);
		}

		// Landing
		if (landingEnabled)
			light.fill(0, light.getLength() / 2, 0xFF, 0xFF, 0xFF);

		light.flush();
	}

	void Lights::updateStrobes(Light& light, const uint8_t r, const uint8_t g, const uint8_t b) {
		if (strobeEnabled) {
			light.fill(0xFF);
			light.flush();
		}
		else {
			updateNavOrLanding(light, r, g, b);
		}
	}

	void Lights::taskBody(void* args) {
		const auto lights = reinterpret_cast<Lights*>(args);

		//             0       500       1000 ms
		//             +--------+---------+
		// Left wing:  WRWRRRRRRRRRRRRRRRRR
		// Right wing: WGWGGGGGGGGGGGGGGGGG
		// Tail:       DDDWDDDDDDDDDDDDDDDD

		while (true) {
			// Left wing (strobe 1)
			lights->updateStrobes(lights->leftWing, 0xFF, 0x00, 0x00);

			// Tail (dimmed)
//					lights->tail.fill(dimmedChannelValue);
//					lights->tail.flush();

			vTaskDelay(pdMS_TO_TICKS(50));

			// Left wing (red)
			lights->updateNavOrLanding(lights->leftWing, 0xFF, 0x00, 0x00);
			vTaskDelay(pdMS_TO_TICKS(50));

			// Left wing (strobe 2)
			lights->updateStrobes(lights->leftWing, 0xFF, 0x00, 0x00);
			vTaskDelay(pdMS_TO_TICKS(50));

			// Left wing (red)
			lights->updateNavOrLanding(lights->leftWing, 0xFF, 0x00, 0x00);

			// Tail (strobe)
//					lights->tail.fill(0xFF);
//					lights->tail.flush();

			vTaskDelay(pdMS_TO_TICKS(50));

			// Tail (dimmed)
//					lights->tail.fill(dimmedChannelValue);
//					lights->tail.flush();

			vTaskDelay(pdMS_TO_TICKS(16 * 50));
		}
	}
}