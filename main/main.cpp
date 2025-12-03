#include "rc.h"

using namespace pizda;

extern "C" void app_main(void) {
	RC::getInstance().start();
}