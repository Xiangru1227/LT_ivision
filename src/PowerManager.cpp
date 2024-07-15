#include "PowerManager.h"
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <fstream>

PowerManager::PowerManager() {
	lowpower = true;
}

PowerManager::~PowerManager() {

}

//query temperature information for each thermal zone, return highest temp.
float PowerManager::getTemperature() {

	int maxTemp = 0;

	for (int i = 0; i < 8; i++) {
		std::stringstream ss;
		ss << "/sys/devices/virtual/thermal/thermal_zone" << i << "/temp";
		
		std::ifstream tempFile;
		tempFile.open(ss.str());
		int temperature = 0;

		if (tempFile.is_open()) {
			tempFile >> temperature;
			tempFile.close();
			if (temperature > maxTemp && temperature != 100000) {
				maxTemp = temperature;
			}
		}
	}

	return .001f*maxTemp;
}

//if in low power mode, switch to high power mode
void PowerManager::setHighPowerMode() {
	if (lowpower) {
		lowpower = false;
		system("echo nvidia | sudo -S nvpmodel -m 0");
		//std::cout << "Set to high power mode." << std::endl;
	}
}

//if in high power mode, switch to low power mode
void PowerManager::setLowPowerMode() {
	if (!lowpower) {
		lowpower = true;
		system("echo nvidia | sudo -S nvpmodel -m 1");
		//std::cout << "Set to low power mode." << std::endl;
	}
}
