#pragma once

class Battery
{
	const int battery_pin = A13;
	const int full_scale = 3646; //full scale output from ADC
	const float full_voltage[] = { 4.2, 3.95, 3.8, 3.75, 3.65, 3.0 }; // from 0 to 100% discharge in 20% increments
	String battery_volts_str; // holds a string voltage
	String battery_cap_str; // holds a string caoacity
public:
	Battery();
	float getBatteryVoltage() const;
	int getBatteryCapacity() const;
	void refreshBatteryCapacity();
	void refreshBatteryVoltage();
	String getVolts();
	String getCapacity();


	//come here to get battery level
	
};
