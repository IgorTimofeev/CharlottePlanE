#pragma once

#include <cmath>
#include <numbers>

namespace pizda {
	class LowPassFilter {
		public:
			static float apply(float oldValue, float newValue, float factor) {
				// Just in case
				factor = std::clamp(factor, 0.f, 1.f);
				
				return oldValue * (1.f - factor) + newValue * factor;
			}
			
			static float applyForAngleRad(float oldValue, float newValue, float factor) {
				auto delta = newValue - oldValue;
				
				// -170 to 170
				// delta = 170 - -170 = 340
				if (delta > std::numbers::pi_v<float>) {
					// delta = 340 - 360 = -20
					delta -= 2.0f * std::numbers::pi_v<float>;
				}
				else if (delta < -std::numbers::pi_v<float>) {
					delta += 2.0f * std::numbers::pi_v<float>;
				}
				
				// oldValue = 170 - -20 = 190;
				oldValue = newValue - delta;
				oldValue = apply(oldValue, newValue, factor);
				
				if (oldValue > std::numbers::pi_v<float>) {
					oldValue -= 2.0f * std::numbers::pi_v<float>;
				}
				else if (oldValue < -std::numbers::pi_v<float>) {
					oldValue += 2.0f * std::numbers::pi_v<float>;
				}
				
				return oldValue;
			}
	};
}