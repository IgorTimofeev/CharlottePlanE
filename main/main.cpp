#include "aircraft.h"

extern "C" void app_main(void) {
	pizda::Aircraft::getInstance().start();
}