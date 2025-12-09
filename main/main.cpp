#include "aircraft.h"

using namespace pizda;

extern "C" void app_main(void) {
	Aircraft::getInstance().start();
}