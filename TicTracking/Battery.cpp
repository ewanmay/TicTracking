#include "Battery.h"

// ================================================================
// ===               GET BATTERY LEVEL                          ===
// ================================================================

float Battery::getBatteryVoltage() const
{
	// measure and return battery voltage
	//Serial.println("ADC reading: " + String(analogRead(battery_pin)));
	//Serial.println(analogRead(battery_pin));
	return float(analogRead(battery_pin)) / float(full_scale) * 3.3 * 2.0;
}

int Battery::getBatteryCapacity() const
{
	// measure and return battery capacity in %
	float battery_level = getBatteryVoltage();
	int battery_capacity = 100;
	for (int i = 0; i <= 5; i++)
	{
		// do a linear interpolation between the points in full_voltage
		float incr = (full_voltage[i + 1] - full_voltage[i]) / 20;
		float test_level = full_voltage[i];
		for (int j = 1; j <= 20; j++)
		{
			if (battery_level >= test_level) return battery_capacity;
			//Serial.println("Voltage : " + String(battery_level) + "Capacity: "+battery_capacity+"test_level "+test_level);
			battery_capacity--;
			test_level += incr;
		}
	}
	return 0;
}

// ================================================================
// ===               REFRESH BATTERY METRICS                    ===
// ================================================================
void Battery::refreshBatteryCapacity()
{
	battery_cap_str = sardprintf("Batt Cap'y: %d/100", tracker_battery.getBatteryCapacity());
}
void Battery::refreshBatteryVoltage()
{
	battery_volts_str = sardprintf("Batt Volts: %f ", getBatteryVoltage());
}

String Battery::getVolts()
{
	return battery_volts_str;
}

String Battery::getCapacity()
{
	return battery_cap_str;
}