#pragma once

#include <expected>

#include <esp_log.h>
#include <esp_timer.h>

namespace pizda {
	class Logger {
		public:
			static void info(const char* tag, const char* format, ...) {
				va_list args;
				va_start(args, format);
				log("I", tag, format, args);
				va_end(args);
			}

			static void error(const char* tag, const char* format, ...) {
				va_list args;
				va_start(args, format);
				log("E", tag, format, args);
				va_end(args);
			}

			static void error(const char* tag, esp_err_t code) {
				error(tag, esp_err_to_name(code));
			}

			static bool check(const char* tag, esp_err_t code) {
				if (code == ESP_OK)
					return true;

				error(tag, esp_err_to_name(code));
				return false;
			}

		private:
			static void log(const char* type, const char* tag, const char* format, va_list args) {
				const auto timeUs = esp_timer_get_time();
				uint64_t minutes = timeUs / 60'000'000ULL;
				uint64_t remaining = timeUs % 60'000'000ULL;
				uint64_t seconds = remaining / 1'000'000ULL;
				remaining = remaining % 1'000'000ULL;
				uint64_t milliseconds = remaining / 1'000ULL;

				printf("%02llu:%02llu:%03llu | %s | %s | ", minutes, seconds, milliseconds, type, tag);
				vprintf(format, args);
				printf("\n");
			}
	};
}