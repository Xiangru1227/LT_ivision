#pragma once

class PowerManager {
public:
	PowerManager();
	~PowerManager();

	//get module temperature (Celsius)
	float getTemperature();

	//set module to 10W power mode
	void setHighPowerMode();

	//set module to 5W power mode
	void setLowPowerMode();
private:
	//track whether module is in low or high power mode
	bool lowpower;
};
