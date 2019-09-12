// alarm.h
//
// manages alarms, status_area and display/buttons within the system

#ifndef _SYSTEM_PARAMETERS
#include "system_parameters.h"
#endif


#ifndef _ALARM_h
#define _ALARM_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <arduino.h>
#else
// ReSharper disable once CppUnusedIncludeDirective
	#include "WProgram.h"
#endif

/***********************************
 *
 * Alarms and status_area
 *
 ***********************************/

enum status { unknown, ok, minor, major, stopped }; // this code relies on how the enum is initialized i.e. in order of increasing severity. ** be careful when changing **
enum area { unknown_area, overall, rt_clock, sd_card, r_accelerometer, l_accelerometer, battery };  // all the areas where the system can have alarms and problems


//enum area {};

struct area_status
{
	area system_area;
	status status_area;
	char* msg;
};

struct system_status
{
	area_status overall_status;
	int max_area_index;
	area_status areas[];
};

extern system_status system_Status;
system_status init_status_system(area_status area_statuses[], int area_count);
void set_area_status(area_status area_status);
void set_system_status();
system_status get_system_status();

/***********************************
 *
 * display
 *
 ***********************************/
#include <Wire.h>
//#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

 // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
extern Adafruit_SSD1306 display;
#define DISPLAY_BLANK_INTERVAL 3*1000000  // this is how long the system will show a display before blanking (in us)

/***********************************
 *
 * buttons
 *
 ***********************************/
#define BUTTON_UP 1
#define BUTTON_PRESSED 0

enum button_function {not_a_button, mom1, mom2, on_off};
const int total_buttons = 3; // 

struct button
{
	gpio_num_t pin; //GPIO pin
	button_function function;
	bool pressed;};

extern button buttons[];
const button null_button = { .pin = (gpio_num_t) -1 ,.function = not_a_button,.pressed = true };

void init_UI();
void init_buttons(const gpio_num_t pins[], const button_function function[], const int button_count);
bool read_button_pressed(int pin);
String dump_io_pins(int pins[], int num_pins);
void blank_display();
void bit_display(byte* image);
void display_and_hold(String title, char* heading, char* message, int timeout);
void display_and_blank(char* title, system_status status, char*message);
void display_and_blank(String title, String heading, String message);
void display_and_blank(char* title, char* heading, char* message, int button_pins[], int num_pins, int timeout);

button display_and_return_button(const char* title, system_status status, char* message, char* right_label, char* left_label,
	int button_pins[], int num_pins, int timeout);
button display_and_return_button(String title, system_status status, char* message, char* right_label, char* left_label,
	int button_pins[], int num_pins, int timeout);
void enable_wakeup_pin(const int pins[], const int no_pins, const gpio_int_type_t intr_type);
void disable_wakeup_pin(int pins[], int no_pins);

bool button_press_quick_check(int pin);


#endif

