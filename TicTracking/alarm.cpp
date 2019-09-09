// 
// 
// 

#include "alarm.h"
#include "Logging.h"

system_status system_Status;
button find_button_pressed(button buttons[]);


//
// initialize status_area struct
// 

system_status init_status_system(area_status area_statuses[], int area_count)
{
	// create the areas and initialize them.  area_statuses[0] is system so use an offset
	for (auto i = 1; i <= area_count; i++)
	{
		system_Status.areas[i - 1].system_area = area_statuses[i].system_area;
		system_Status.areas[i - 1].status_area = unknown;
		system_Status.areas[i - 1].msg = "status_unknown";
	}
	// now finish up with the system level init
	system_Status.max_area_index = area_count;
	system_Status.overall_status.system_area = area_statuses[0].system_area;
	system_Status.overall_status.status_area = unknown;
	system_Status.overall_status.msg = "status_area unknown";

	return system_Status;
}

void set_area_status(const area_status area_status)
// update system_Status with area_status
{
	for (auto i = 0; i < system_Status.max_area_index; i++)
	{
		if (system_Status.areas[i].system_area == area_status.system_area)
		{
			system_Status.areas[i].status_area = area_status.status_area;
			system_Status.areas[i].msg = area_status.msg;
			break;
		}
	}
}

void set_system_status()
// set system status_area based on area statuses
// the logic:
//   - unknown if any component unknown
//   - ok if all ok
//   - worst (highest) of any area between unknown and ok
{
	//area_status unknown_status = {overall, unknown, "1+ areas reporting unknown"};
	char* all_ok_status_msg = "all areas ok";
	char* all_unk_status_msg = "1+ areas reporting unknown";
	system_Status.overall_status.status_area = ok;
	system_Status.overall_status.msg = all_ok_status_msg;

	for (auto i = 0; i < system_Status.max_area_index; i++)
	{
		if (system_Status.areas[i].status_area == ok) continue;
		if (system_Status.areas[i].status_area == unknown)
		{
			system_Status.overall_status.status_area = unknown;
			system_Status.overall_status.msg = all_unk_status_msg;
			break;
		}
		else
		{
			// this code relies on how the enum has been initialized i.e. in order of increasing severity
			if (system_Status.areas[i].status_area > system_Status.overall_status.status_area)
			{
				system_Status.overall_status.status_area = system_Status.areas[i].status_area;
				system_Status.overall_status.msg = system_Status.areas[i].msg;
			}
		}
	}
}

system_status get_system_status()
{
	const system_status temp_status = {
		system_Status.overall_status.system_area, system_Status.overall_status.status_area,
		system_Status.overall_status.msg
	};
	return temp_status;
}

/***********************************
 *
 * display
 *
 ***********************************/
//#include <Wire.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
//
//#define SCREEN_WIDTH 128 // OLED display width, in pixels
//#define SCREEN_HEIGHT 32 // OLED display height, in pixels
//
// // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
//#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//
//
//enum button_function { mom1, mom2, on_off };
//struct button
//{
//	int pin; //GPIO pin
//	button_function function;
//	bool pressed;
//};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
button buttons[total_buttons];
bool read_button_pressed(int pin);

void init_UI()
{
	display.begin();
	//set up io switches
	gpio_num_t pins[] = { (gpio_num_t) SW1,(gpio_num_t)SW2,(gpio_num_t)SW3 };
	button_function function[] = {mom1, mom2, on_off};
	init_buttons(pins, function, total_buttons);
}
/** Initialize buttons.
 * Setup buttons, function and GPIO config
 */
void init_buttons(const gpio_num_t pins[], const button_function function[], const int button_count)
{	
	gpio_num_t button_gpio_num; // = (gpio_num_t)BUTTON_GPIO_NUM_DEFAULT;
	gpio_config_t config;
	const int wakeup_level = 0;  // wakeup on low level

	for (int i = 0; i < button_count; i++)
	{
		buttons[i].function = function[i];
		buttons[i].pin = pins[i];
		//pinMode(buttons[i].pin, INPUT_PULLUP);
		

		// GPIO pin setup
		/* Configure the button GPIO as input, enable wakeup */
		config = {
			.pin_bit_mask = BIT64(buttons[i].pin),
			.mode = GPIO_MODE_INPUT,
			.pull_up_en = GPIO_PULLUP_ENABLE
		};
		ESP_ERROR_CHECK(gpio_config(&config));
		gpio_wakeup_enable((gpio_num_t) buttons[i].pin,
			wakeup_level == 0 ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);

		buttons[i].pressed = read_button_pressed(buttons[i].pin);
	}
}

void blank_display()
{
	display.clearDisplay();
	display.display();
}

void bit_display(byte* image)
{
}

void display_and_blank(char* title, system_status status, char* message)
{
	blank_display();
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(0, 0);
	display.println(title);
	display.println(status.overall_status.msg);
	display.println(message);
	display.setCursor(0, 0);
	display.display(); // actually display all of the above

	// now go to sleep. return either a valid button press or timeout
	/* Wake up in 2 seconds, or when button is pressed */
	esp_sleep_enable_timer_wakeup((uint64_t)DISPLAY_BLANK_INTERVAL);
	esp_sleep_enable_gpio_wakeup();
	/* Enter sleep mode */
	esp_light_sleep_start();

	/* Execution continues here after wakeup */
	/* Determine wake up reason */
	const char* wakeup_reason;
	switch(esp_sleep_get_wakeup_cause())
	{
	case ESP_SLEEP_WAKEUP_TIMER:
		wakeup_reason = "timer";
		break;
	case ESP_SLEEP_WAKEUP_GPIO:
		wakeup_reason = "pin";
		break;
	default:
		wakeup_reason = "other";
		break;
	}
	logger_debug(__LINE__, String("Exiting from d&b sleep reason:") + String(wakeup_reason));
	display.clearDisplay();
	display.display();
	esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
	esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
}


button display_and_return_button(char* title, system_status status, char* message, char* right_label, char* left_label,
                                 button check_buttons)
{// put up a display with button labels.  Sleep until a button is pushed or timeout is reached
	blank_display();
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(0, 0);
	display.println(title);
	display.println(status.overall_status.msg);
	display.println(message);
	// now properly locate the labels
	const auto line_length = 22;
	char label_line[line_length + 1]; // line_length + a null at end for delimiter
	int index;
	for (index = 0; index < line_length, right_label[index] != 0; index++) label_line[index] = left_label[index];
	//left from left
	// index now points to leftmost blank in label_line.  Figure out len of right label and fill in the rest
	int right_length;
	for (right_length = 0; right_label[right_length] != 0; right_length++); //right_length = len of right_label
	for (auto i = index; i <= line_length; i++)
	{
		if (i < (line_length - right_length))
		{
			label_line[i] = 32; // pad blank spaces until it's time to add right_label
		}
		else
		{
			label_line[i] = right_label[i - index]; // add right label
		}
	}
	display.println(label_line);
	display.setCursor(0, 0);
	display.display(); // actually display all of the above

	// now go to sleep. return either a valid button press or timeout
	esp_sleep_enable_timer_wakeup((uint64_t)DISPLAY_BLANK_INTERVAL);
	esp_sleep_enable_gpio_wakeup();
	/* Enter sleep mode */
	esp_light_sleep_start();

	/* Execution continues here after wakeup */
	/* Determine wake up reason */
	const char* wakeup_reason;
	bool was_button = false; //indicates whether it was a switch waking me up
	switch (esp_sleep_get_wakeup_cause())
	{
	case ESP_SLEEP_WAKEUP_TIMER:
		wakeup_reason = "timer";
		break;
	case ESP_SLEEP_WAKEUP_GPIO:
		wakeup_reason = "pin";
		was_button = true;
		break;
	default:
		wakeup_reason = "other";
		break;
	}
	logger_debug(__LINE__, String("Exiting from d&RB sleep reason:") + String(wakeup_reason));
	display.clearDisplay();
	display.display();
	esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
	esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
	if (was_button) return find_button_pressed(buttons);
	logger_error(__LINE__, String("Returning null button.  Should have found real button press"));
	return null_button;
}
/** Find the first pressed button.
 * scan buttons until find one pressed.  Return it
 * TODO decide whether to scan all instead of taking first
 */
button find_button_pressed(button buttons[])
{
	for (int i = 0; i < total_buttons;i++)
	{
		if (gpio_get_level(buttons[i].pin) == BUTTON_PRESSED)
		{
			buttons[i].pressed = read_button_pressed(i);  //TODO decide whether we need to debounce or not
			return buttons[i];
		}
	}
	logger_error(__LINE__, String("Returning null button.  Should have found real button press"));
	return null_button;
}
bool read_button_pressed(int pin)
{
	const auto debounce_time = 20; //wait time
	const auto max_cycles = 10; // maximum number of cycles before returning
	const auto max_consec_debounces = 3;
	auto consec_debounces = 0;

	// arduino 
		//bool switch_state = digitalRead(pin);
		//bool current_state;
	//ESP-IF
	auto gpio_num = (gpio_num_t)pin;
	auto switch_state = gpio_get_level(gpio_num);
	auto current_state = 0;


	for (auto i = 0; i < max_cycles, consec_debounces < max_consec_debounces; i++)
	{
		yield();
		delay(debounce_time);
		// arduino
			//auto current_state = digitalRead(pin);
		//ESP-IF
		auto current_state = gpio_get_level(gpio_num);
		if (switch_state == current_state)
		{
			consec_debounces++;
		}
		else
		{
			consec_debounces = 0;
			switch_state = current_state;
		}
	}
	return switch_state == BUTTON_PRESSED;
}