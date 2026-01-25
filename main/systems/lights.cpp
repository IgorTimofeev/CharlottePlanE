#include "lights.h"

#include <esp_timer.h>
#include "aircraft.h"

namespace pizda {
	void Lights::setup() const {
//				tail.fill(0x00);
//				tail.flush();

		leftWing.fill(0x00);
		leftWing.flush();
	}

	void Lights::start() {
		xTaskCreate(
			[](void* arg) {
				static_cast<Lights*>(arg)->onStart();
			},
			"lights",
			4096,
			this,
			tskIDLE_PRIORITY,
			&taskHandle
		);
	}
	
	void Lights::setCabinEnabled(const bool value) const {
		auto& ac = Aircraft::getInstance();
		
		if (value == ac.settings.lights.cabin)
			return;
		
		ac.settings.lights.cabin = value;
		ac.settings.lights.scheduleWrite();
		
		wake();
	}

	void Lights::setNavigationEnabled(const bool value) const {
		auto& ac = Aircraft::getInstance();
		
		if (value == ac.settings.lights.nav)
			return;
		
		ac.settings.lights.nav = value;
		ac.settings.lights.scheduleWrite();
		
		wake();
	}

	void Lights::setStrobeEnabled(const bool value) const {
		auto& ac = Aircraft::getInstance();
		
		if (value == ac.settings.lights.strobe)
			return;
		
		ac.settings.lights.strobe = value;
		ac.settings.lights.scheduleWrite();
		
		wake();
	}

	void Lights::setLandingEnabled(const bool value) const {
		auto& ac = Aircraft::getInstance();
		
		if (value == ac.settings.lights.landing)
			return;
		
		ac.settings.lights.landing = value;
		ac.settings.lights.scheduleWrite();
		
		wake();
	}

	void Lights::setEmergencyEnabled(const bool value) {
		if (value == _emergency)
			return;

		_emergency = value;
		
		wake();
	}
	
	// -------------------------------- Processing --------------------------------
	
	bool Lights::delay(const uint32_t ms) {
		return ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ms)) > 0;
	}
	
	void Lights::wake() const {
		xTaskNotifyGive(taskHandle);
	}
	
	void Lights::updateNavOrLanding(const Light& light, const uint8_t r, const uint8_t g, const uint8_t b) {
		const auto& ac = Aircraft::getInstance();
		
		// Navigation
		if (ac.settings.lights.nav) {
			light.fill(r, g, b);
		}
		else {
			light.fill(0x00);
		}

		// Landing
		if (ac.settings.lights.landing)
			light.fill(0, light.getLength() / 2, 0xFF, 0xFF, 0xFF);

		light.flush();
	}

	void Lights::updateStrobes(const Light& light, const uint8_t r, const uint8_t g, const uint8_t b) {
		const auto& ac = Aircraft::getInstance();
		
		if (ac.settings.lights.strobe) {
			light.fill(0xFF);
			light.flush();
		}
		else {
			updateNavOrLanding(light, r, g, b);
		}
	}
	
	[[noreturn]] void Lights::onStart() const {
		const auto& ac = Aircraft::getInstance();
		
		//             0       500       1000 ms
		//             +--------+---------+
		// Left wing:  WRWRRRRRRRRRRRRRRRRR
		// Right wing: WGWGGGGGGGGGGGGGGGGG
		// Tail:       DDDWDDDDDDDDDDDDDDDD

		while (true) {
			if (_emergency) {
				// Left wing
				leftWing.fill(0xFF, 0x00, 0x00);
				leftWing.flush();
				
				// Right wing
				leftWing.fill(0xFF, 0x00, 0x00);
				leftWing.flush();
				
				// Tail
				leftWing.fill(0xFF, 0x00, 0x00);
				leftWing.flush();
				
				// Cabin
				cabin.fill(0xFF, 0x00, 0x00);
				cabin.flush();

				if (delay(500))
					continue;
				
				// Left wing
				leftWing.fill(0x00);
				leftWing.flush();
				
				// Right wing
				leftWing.fill(0x00);
				leftWing.flush();
				
				// Tail
				leftWing.fill(0x00);
				leftWing.flush();
				
				// Cabin
				cabin.fill(0x00);
				cabin.flush();
				
				if (delay(500))
					continue;
			}
			else {
				// Cabin
				cabin.fill(ac.settings.lights.cabin ? 0xFF : 0x00);
				cabin.flush();
				
				// Left wing (strobe 1 or red)
				updateStrobes(leftWing, 0xFF, 0x00, 0x00);
				
				// Right wing (strobe 1 or green)
				updateStrobes(rightWing, 0x00, 0xFF, 0x00);

				// Tail (dimmed)
				tail.fill(tailDimmedValue);
				tail.flush();
				
				if (delay(50))
					continue;

				// Left wing (red)
				updateNavOrLanding(leftWing, 0xFF, 0x00, 0x00);
				
				if (delay(50))
					continue;

				// Left wing (strobe 2 or red)
				updateStrobes(leftWing, 0xFF, 0x00, 0x00);
				
				// Right wing (strobe 2 or green)
				updateStrobes(rightWing, 0x00, 0xFF, 0x00);
				
				if (delay(50))
					continue;

				// Left wing (red)
				updateNavOrLanding(leftWing, 0xFF, 0x00, 0x00);
				
				// Right wing (green)
				updateNavOrLanding(rightWing, 0x00, 0xFF, 0x00);
				
				// Tail (strobe)
				tail.fill(0xFF);
				tail.flush();
				
				if (delay(50))
					continue;

				// Tail (dimmed)
				tail.fill(tailDimmedValue);
				tail.flush();
				
				if (delay(16 * 50))
					continue;
			}
		}
	}
}