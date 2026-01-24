#include "aircraft.h"

using namespace YOBA;

extern "C" void app_main(void) {
	Aircraft::getInstance().start();
}