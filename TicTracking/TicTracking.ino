/* TicTracking program
 * 
 * by Ewan May and Richard May
 * 
 * */


//#include <gfxfont.h>
#include <Adafruit_SPITFT_Macros.h>
//#include <Adafruit_SPITFT.h>
#include <Adafruit_GFX.h>
#include "alarm.h"
#include "system_state.h"
#include "Logging.h"
#include <I2Cdev.h>
//#include "MPU6050_6Axis_MotionApps20.h"
#include "Multi_I2C.h"
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <RTClib.h>
#include <cmath>

#include "system_parameters.h"
#include <driver/rtc_io.h>

/*****************************************************************************
 *
 * declarations
 *
 *****************************************************************************/
void setup1();

/*****************************************************************************
 *
 * option switches
 *
 *****************************************************************************/
//#define TIMING
#define LOGGING
#define LOGFILE "/Logs/Log xxxx - yyyy.txt"  //where xxxx = date and yyyy = daily sequence

//#define TEST_L_IMU  // switches to force system to use one IMU only
#define TEST_R_IMU


String msg;
#define BATTERY_MEASURE_INTERVAL  10*60*1000 //number of millis() between measurements
unsigned long last_measure_milli = 0; //records when last measurement was logged


#define SAMPLE_RATE 20 // samples per second
#define WAIT_TIMEOUT_MS  (1000/(SAMPLE_RATE) * 2) //timeout interval (mS) waiting for interrupt from accelerometer
#define GYRO_RATE 1000

/*******************************************************************************
 * 
 * OUTPUT FORMAT
 * 
 *******************************************************************************/
#define RECORD_READABLE_QUATERNION //actual quaternion components in a [w, x, y, z] format
//#define OUTPUT_ALL // output all formats
//#define RECORD_ALL // uncomment "RECORD_ALL" if you want to write the actual data straight out of the FIFO
//#define OUTPUT_READABLE_EULER //Euler angles (in degrees) calculated from the quaternions coming from the FIFO
//#define OUTPUT_READABLE_YAWPITCHROLL //the yaw pitch/roll angles (in degrees) calculated from the quaternions coming from the FIFO. Note this also requires gravity vector calculations.
//#define OUTPUT_READABLE_REALACCEL  //acceleration components with gravity removed. This acceleration reference frame is not compensated for orientation, so +X is always +X according to the sensor, just without the effects of gravity
//#define OUTPUT_READABLE_WORLDACCEL //acceleration components with gravity removed and adjusted for the world frame of reference (yaw is relative to initial orientation, since no magnetometer is present in this case)

// #define OUTPUT_TEAPOT //output = format used for the InvenSense teapot demo

/*****************************************************************************
 *
 * Testing
 *
 *****************************************************************************/
#define TEST_MEASURE_INTERVAL 20*1000 // milliseconds

/*****************************************************************************
 *
 * System state
 *
 *****************************************************************************/
system_state system_State = {none, none, startup}; // start with initialization (from off we go to begin, however)

/*****************************************************************************
 *
 * System status_area, alarms and displays
 *
 *****************************************************************************/
//enum area  { unknown_area, overall, clock, sd_card, r_accelerometer, l_accelerometer, battery};  // all the areas where the system can have alarms and problems
// time to delay shutdown for user cancel (in seconds)
#define SYSTEM_OFF_DELAY 10

/*****************************************************************************
 *
 * Timing measurement
 *
 *****************************************************************************/

long start = 0; // defines top of loop
#ifdef TIMING
#define TIME_START loop_start_micro=micros();
#define TIME_EVENT(msg) record_time(#msg, micros(), __LINE__, loop_start_micro);
// record_time("exit left sleep", micros(), __LINE__, loop_start_micro);	

#else
#define TIME_START
#define TIME_EVENT(msg) 

#endif
void record_time(String msg, long time, int line_no, long start)
{
	String record = String(line_no);
	record.concat("\t");
	record.concat(time);
	record.concat("\t");
	record.concat(time - start);
	record.concat("\t");
	record.concat(msg);
#ifdef TIMING
	Serial.println(record);
	Serial.flush();
#endif
}

/*****************************************************************************
 *
 * Logging
 *
 *****************************************************************************/
RTC_PCF8523 rtc;
File logfile; // for storing all log information
String log_filepath;
String log_filename;

#ifdef LOGGING
//#include "Logging.h"
//#define LOG_START(Logfile, CS) logger_setup(Logfile, CS); //CS is chip select pin
//#define LOG_DEBUG(Logfile, now, msg) logger_debug(Logfile, now, msg);
//#define LOG_DATA(msg) logger_data(__LINE__, msg);
//#define LOG_INFO(Logfile, now, msg)logger_info(Logfile, millis(), msg);
//#define LOG_ERROR(msg) logger_data(__LINE__, msg);
//#define LOG_CRITICAL(Logfile, now, msg) logger_critical(Logfile, millis(), msg);
//#define LOG_READABLE_QUATERNION

#else
#define LOG_START(Logfile, CS) 
#define LOG_DEBUG(msg)
#define LOG_DATA(msg)
#define LOG_INFO(msg)
#define LOG_ERROR(msg)
#define LOG_CRITICAL(msg)
#endif

/*****************************************************************************
 *
 * outputs
 *
 *****************************************************************************/

// uncomment "RECORD_READABLE_QUATERNION" if you want to write the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define RECORD_READABLE_QUATERNION
#ifdef RECORD_READABLE_QUATERNION
#define RECORD_HEADER "mSec, w, x, y, z"
#define FLUSH_LIMIT 20
File myFile;
int line_count = 0; // counts lines prior to flushing
#endif


#ifdef RECORD_ALL
#define RECORD_HEADER "mSec, qw, qx, qy, qz, ax, ay, az, gx, gy, gz"
#define FLUSH_LIMIT 20
int line_count = 0; // counts lines prior to flushing
#endif


/*****************************************************************************
 *
 * MPU 6050 related
 *
 *****************************************************************************/
//#include "MPU6050.h" // not necessary if using MotionApps include file
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
// MPU6050 mpu();

Multi_MPU mpu_left(AD0_PIN_LEFT, 0x69); // <-- use for AD0 
Multi_MPU mpu_right(AD0_PIN_RIGHT, 0x69); // <-- use for AD0 high
Multi_MPU* mpu;
bool blinkState = false;

// MPU control/init_status_string vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus_left; // holds actual interrupt init_status_string byte from MPU
uint8_t mpuIntStatus_right; // holds actual interrupt init_status_string byte from MPU
uint8_t mpuIntStatus; // holds actual interrupt init_status_string byte from MPU
uint8_t devStatus_right; // return init_status_string after each device operation (0 = success, !0 = error)
uint8_t devStatus_left; // return init_status_string after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint16_t fifoCount_left = 0; // count of all bytes currently in FIFO
uint16_t fifoCount_right = 0; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z]         quaternion container
VectorInt16 aa; // [x, y, z]            accel sensor measurements
VectorInt16 aaReal; // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3]; // [psi, theta, phi]    Euler angle container
float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

auto readLeft = false;  // for testing, forces the system to read only the Left IMU
long old_millis = 0; //for calculating dump intervals


// ================================================================
// ===               INTERRUPT SERVICE ROUTINES                ===
// ================================================================

volatile bool on_off_pushed = false; // indicates whether on/off switch has been pushed
void IRAM_ATTR on_off_switch_set()
{
	on_off_pushed = true;
}

void on_off_switch_reset()
{
	on_off_pushed = false;
}

volatile bool mpuInterruptRight = false; // indicates whether MPU interrupt pin has gone high
volatile bool mpuInterruptLeft = false; // indicates whether MPU interrupt pin has gone high

// ================================================================
// ===               GET BATTERY LEVEL                          ===
// ================================================================

//come here to get battery level

const int battery_pin = A13;
const int full_scale = 3646; //full scale output from ADC
const float full_voltage[] = {4.2, 3.95, 3.8, 3.75, 3.65, 3.0}; // from 0 to 100% discharge in 20% increments
String battery_volts_str; // holds a string voltage
String battery_capy_str; // holds a string caoacity

float get_battery_voltage()
{
	// measure and return battery voltage
	//Serial.println("ADC reading: " + String(analogRead(battery_pin)));
	//Serial.println(analogRead(battery_pin));
	return float(analogRead(battery_pin)) / float(full_scale) * 3.3 * 2.0;
}

int get_battery_capacity()
{
	// measure and return battery capacity in %
	const float battery_level = get_battery_voltage();
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
}



// ================================================================
// ===               LOG NAME                                   ===
// ================================================================

String getstring()
{
	// return a String from the keyboard
	String msg = "";
	while (Serial.available() > 0)
	{
		int incomingchar = Serial.read();
		switch (incomingchar)
		{
		case 13: //carriage return
			return msg;
		case 8: //backspace
			msg.remove(msg.length() - 1, 1);
		}
	}
	return msg;
}

// ================================================================
// ===                    INITIALIZE DMP                        ===
// ================================================================

void initializeDmp(int devStatus, int interruptPin, Multi_MPU& mpu)
{
	if (devStatus == 0)
	{
		// Calibration Time: generate offsets and calibrate our MPU6050
		mpu.CalibrateAccel(6);
		mpu.CalibrateGyro(6);
		//mpu.PrintActiveOffsets();
		// turn on the DMP, now that it's ready
		mpu.setDMPEnabled(true);

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();

		// look at FIFO Rate Divisor int the DMP Firmware Memory
		unsigned char dmpUpdate[2];
		mpu.readMemoryBlock(dmpUpdate, 0x02, 0x02, 0x16);
		// Lets read the dmpUpdate data from the Firmware image, we have 2 bytes to write in bank 0x02 with the Offset 0x16
		Serial.printf("DMP sample divisor dump.  Should be 0x0, 0x0: ");
		for (auto i = 0; i < (sizeof(dmpUpdate) / sizeof(dmpUpdate[0])); i++)
		{
			Serial.printf("%d \t", dmpUpdate[i]);
		}
		Serial.println("done");
	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		//Serial.print(F("DMP Initialization failed (code "));
		//Serial.print(devStatus);
		//Serial.println(F(")"));
	}
}

bool recover_MPU(Multi_MPU& mpu)
// come here as a last resort to recover an MPU that has lost its mind
{
	uint8_t devStatus; // return init_status_string after each device operation (0 = success, !0 = error)
	// initialize device
	Serial.printf(F
	("Recovering %s MPU...at address "), mpu.label);

	mpu.initialize();
	//mpu_right.initialize();

	Serial.println(mpu.getDeviceID(), HEX);
	//Serial.println(" and " + String(mpu_right.getDeviceID(), HEX));

	//logger_debug(__LINE__, mpu_left.getStatusString());
	//logger_debug(__LINE__, mpu_right.getStatusString());

	//mpu_left.label = "Left ";
	//mpu_left.prefix = "L";
	//mpu_right.prefix = "R";
	//mpu_right.label = "Right ";

	// verify connection
	//Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection()
		               ? F("MPU6050 connection successful")
		               : F("MPU6050 connection failed"));
	//Serial.println(mpu_left.testConnection()
	//	? F("MPU6050 left side connection successful")
	//	: F("MPU6050 left side connection failed"));

	// load and configure the DMP`s
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
	//mpu_right.setXGyroOffset(220);
	//mpu_right.setYGyroOffset(76);
	//mpu_right.setZGyroOffset(-85);
	//mpu_right.setZAccelOffset(1788); // 1688 
	//boolean const DmpReady = initializeDmp(devStatus_right, INTERRUPT_PIN_RIGHT, mpu_right);
	mpu.CalibrateAccel(6);
	mpu.CalibrateGyro(6);
	//mpu.PrintActiveOffsets();
	// turn on the DMP, now that it's ready
	mpu.setDMPEnabled(true);

	// look at FIFO Rate Divisor int the DMP Firmware Memory
	unsigned char dmpUpdate[2];
	mpu.readMemoryBlock(dmpUpdate, 0x02, 0x02, 0x16);
	// Lets read the dmpUpdate data from the Firmware image, we have 2 bytes to write in bank 0x02 with the Offset 0x16
	Serial.printf("DMP sample divisor dump.  Should be 0x0, 0x0: ");
	for (int i = 0; i < (sizeof(dmpUpdate) / sizeof(dmpUpdate[0])); i++)
	{
		Serial.printf("%d \t", dmpUpdate[i]);
	}
	Serial.println("done");

	uint8_t smplrt_div = (GYRO_RATE / SAMPLE_RATE) - 1;
	mpu.setRate(smplrt_div);

	if (devStatus == 0)
	{
		logger_info(__LINE__, sardprintf("%s MPU recovered", mpu.label));
		mpu.init_status_string = mpu.getStatusString(); //update the status string
	}
	else
	{
		logger_error(__LINE__, sardprintf("%s MPU FAILED recovery. Code: %d", mpu.label, devStatus));
	}
	return devStatus;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
	setup1();
} // end setup()

/**
 * \brief setup1
 * This function is essentially setup() but in a separate function so I can call it whenever I need it
 */
void setup1()
{
	system_State.current = startup;

	logger_debug(__LINE__, "Starting startup state");
	if (!Serial)
	{
		Serial.begin(230400);
		delay(5000); // TODO remove this one once you figure out how to slow things down
	}
	while (!Serial); // wait for Leonardo enumeration, others continue immediately
	//Serial.println("Hello");


	// 
	// set up UI
	//
	init_UI();
	logger_debug(__LINE__, "Display > Starting up");
	display_and_blank("Starting up", system_Status, "Please stand by");

	//
	// Set up on/off button interrupt
	//
	attachInterrupt(digitalPinToInterrupt(SW3), on_off_switch_set, HIGH);

	// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin(); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

	// pins for address and interrupt on MPU's
	pinMode(AD0_PIN_LEFT, OUTPUT);
	pinMode(AD0_PIN_RIGHT, OUTPUT);
	pinMode(INTERRUPT_PIN_LEFT, INPUT);
	pinMode(INTERRUPT_PIN_RIGHT, INPUT);

#ifdef LOGGING
	// Set up logging
	// Log_Level excludes[] = { ignore, info, error,critical }; //exclude printing data logs because so voluminous
	// Log_Level excludes[] = { data }; //exclude printing data logs because so voluminous
	// set_logger_filter(excludes, sizeof(excludes) / sizeof(excludes[0]), 10);
	while (!SD.begin())
	{
		logger_debug(__LINE__, "SD initialization failed! ... retrying");
		delay(1000);
	}

	//Serial.println("SD initialization success");
	logger_debug(__LINE__, "Display > SD initialization ok");
	display_and_blank("SD card ok", system_Status, "Please stand by.");


	//Next, turn off the MPU6050's by moving their I2C addresses to the higher value AD0
	digitalWrite(AD0_PIN_LEFT, HIGH);
	digitalWrite(AD0_PIN_RIGHT, HIGH);
	if (!rtc.begin())
	{
		//Serial.println("Couldn't find RTC... pausing and restarting");
		logger_error(__LINE__, "Couldn't find RTC... pausing and restarting");
		delay(1000);
		system_State.previous = startup;
		system_State.next = begin;
		return;
	}
	else
	{
		logger_debug(__LINE__, "display > RTC initialization ok");
		display_and_blank("RTC ok", system_Status, "Please stand by..");
		//Serial.println("RTC initialization success");
	}
	// now get the time and prepare a log file.  
	DateTime now = rtc.now();
	String date_stamp = now.timestamp(DateTime::TIMESTAMP_DATE);
	String file_path_yyyy = String(LOGFILE);
	file_path_yyyy.replace("xxxx", date_stamp);

	// now look for an available logging file
	for (int i = 1; i < 1000; i++)
	{
		char seq[4];
		sprintf(seq, "%04d", i);
		log_filepath = file_path_yyyy;
		log_filepath.replace("yyyy", seq);
		//Serial.println(log_filepath);
		if (SD.exists(log_filepath))
		{
			//Serial.printf("Log file: %s exists. Will try another...\n\r", log_filepath.c_str());
		}
		else
		{
			//Serial.printf("Log file: %s is available\n\r", log_filepath.c_str());
			logfile = logger_setup(log_filepath);
			logger_debug(__LINE__, sardprintf("Checking filename start: %d  end:%d", log_filepath.indexOf(String("Log ")),
				log_filepath.length() - 4));
			log_filename = log_filepath.substring(log_filepath.indexOf(String("Log ")),
				log_filepath.length()-4);
			break;
		}
	}
	// put some initial entries into the file
	//Serial.printf("Log file: %s is available\n\r", log_filepath.c_str());

	//Serial.println(msg);
	//Serial.println(sardprintf("Battery voltage: %f Capacity: %d/100", get_battery_voltage(),
	//	get_battery_capacity()));
	msg = "Opening " + log_filepath + " at " + now.timestamp();
	logger_info(__LINE__, msg);
	battery_capy_str = sardprintf("Batt Cap'y: %d/100", get_battery_capacity());
	battery_volts_str = sardprintf("Batt Volts: %f ", get_battery_voltage());
	logger_info(__LINE__, battery_volts_str + battery_capy_str);
	logger_debug(__LINE__, "Display > Battery status");
	display_and_blank(battery_volts_str, "", battery_capy_str);


#if 0 //TODO get this to work
	Serial.println("Input description for this trial run (return or ignore for none) :");
	//Serial.setTimeout(3000); //3 seconds to wait for input
	while (Serial.available() == 0) {}       //Wait for user input
	Serial.println(Serial.available());
	msg = Serial.readString();
	//msg = "<none>";
	//String temp =  Serial.readString();
	msg = getstring();
	if (msg.length() == 0) { msg = "<none provided>"; }
	logger_info(__LINE__, msg);
#endif

#endif
 
	{
		// verify connection
		//Serial.println(mpu_right.testConnection()
		//	               ? F("MPU6050 right side connection successful")
		//	               : F("MPU6050 right side connection failed"));
		//Serial.println(mpu_left.testConnection()
		//	               ? F("MPU6050 left side connection successful")
		//	               : F("MPU6050 left side connection failed"));

		// initialize devices
		
		//Serial.print(F("Initializing I2C devices...at "));
		mpu_left.initialize();
		mpu_right.initialize();
		logger_debug(__LINE__, sardprintf("Initializing IMU devices...at ", String(mpu_left.getDeviceID(), HEX), String(mpu_right.getDeviceID(), HEX)));

		//Serial.print(mpu_left.getDeviceID(), HEX);
		//Serial.println(" and " + String(mpu_right.getDeviceID(), HEX));

		//logger_debug(__LINE__, mpu_left.getStatusString());
		//logger_debug(__LINE__, mpu_right.getStatusString());

		mpu_left.label = "Left ";
		mpu_left.prefix = "L";
		mpu_right.prefix = "R";
		mpu_right.label = "Right ";


		// load and configure the DMP`s
		//Serial.println(F("Initializing right DMP..."));
		devStatus_right = mpu_right.dmpInitialize();
		if (!devStatus_right)
		{
			logger_debug(__LINE__, "Display > Right IMU ok");
			display_and_blank("Right IMU init ok", system_Status, "Please stand by...");
		}
		//Serial.println(F("Initializing left DMP..."));

		devStatus_left = mpu_left.dmpInitialize();
		if (!devStatus_left)
		{
			logger_debug(__LINE__, "Display > Left MPU ok");
			display_and_blank("Left MPU init ok", system_Status, "Please stand by....");
		}
		// supply your own gyro offsets here, scaled for min sensitivity
		mpu_left.setXGyroOffset(220);
		mpu_left.setYGyroOffset(76);
		mpu_left.setZGyroOffset(-85);
		mpu_left.setZAccelOffset(1788); // 1688 factory default for my test chip
		mpu_right.setXGyroOffset(220);
		mpu_right.setYGyroOffset(76);
		mpu_right.setZGyroOffset(-85);
		mpu_right.setZAccelOffset(1788); // 1688 
		logger_debug(__LINE__, "Display > DMP calibration");
		display_and_blank("MPU's warming up", system_Status, "Please stand by.....");
		initializeDmp(devStatus_right, INTERRUPT_PIN_RIGHT, mpu_right);
		initializeDmp(devStatus_left, INTERRUPT_PIN_LEFT, mpu_left);

#if defined( TEST_R_IMU)
		dmpReady = (devStatus_right == 0);
#elif defined (TEST_L_IMU)
		dmpReady = (devStatus_left == 0);
#else
		dmpReady= (devStatus_right == 0) && (devStatus_left == 0);
#endif
	}

	// set data rates void setRate(uint8_t rate);
	uint8_t smplrt_div = (GYRO_RATE / SAMPLE_RATE) - 1;
	mpu_right.setRate(smplrt_div);
	mpu_left.setRate(smplrt_div);


	// store and print MPU configs
	logger_info(__LINE__, "Initial MPU configs ------------");
	mpu_left.init_status_string = mpu_left.getStatusString();
	logger_info(__LINE__, mpu_left.init_status_string);
	mpu_right.init_status_string = mpu_right.getStatusString();
	logger_info(__LINE__, mpu_right.init_status_string);
	Serial.flush();

	// now get ready for next state
	system_State.previous = startup;
	system_State.next = waiting;
	logger_debug(__LINE__, "Setup complete");
	logger_debug(__LINE__, "Display > All ok");
	display_and_blank("All ok", system_Status, "Please stand by.....!");
} // end setup1()

// ================================================================
// ===                      RECORD LOOP                         ===
// ================================================================
void record_loop()
{
	// Serial.println("Starting loop");
	if (!dmpReady) //TODO this should go to recording_stopped
	{
		Serial.println("Something dun fucked up");
		return;
	}
	auto tabs = "R\t\t\t\t\t";
	record_time("entering loop", micros(), __LINE__, 0L);

	// wait for and process accelerometer data to be ready, doing other things in the meantime
	while (true)
	{
		long loop_start = millis(); // for calculating wait timeout
		long loop_start_micro = micros(); // for calculating ms timing

#if defined( TEST_R_IMU)
		readLeft = false; //TODO for testing only.  It forces system to read only one accelerometer
#elif defined (TEST_L_IMU)
		readLeft = true; //TODO for testing only.  It forces system to read only one accelerometer
#endif

		TIME_EVENT(top of loop)
		// wait for data from an accelerometer.  exit when you have some
		while (true)
		{
			// before waiting, always look for ON/OFF button push
			if (on_off_pushed)
			{
				logger_info(__LINE__, "On_off switch push detected,  Going to going_off");
				system_State.previous = system_State.current;
				system_State.next = going_off;
				//  TODO insert any required cleanup code here

				return;
			}
			// before waiting, if either SW1 or SW2 is pressed return to calling state
			// if (button_press_quick_check(SW1) || button_press_quick_check(SW2)) return;

			// look for accelerometer data and process if found
			fifoCount_left = mpu_left.getFIFOCount();
			fifoCount_right = mpu_right.getFIFOCount();
			//record_time("got FIFO's", micros(), __LINE__, loop_start_micro);
			TIME_EVENT(got FIFOs)
			if (fifoCount_right >= 42 && mpuInterruptRight && !readLeft)
			{
				// Data ready on the right
				mpuInterruptRight = false;
				mpu = &mpu_right;
				readLeft = true;
				//record_time("breaking", micros(), __LINE__, loop_start_micro);
				TIME_EVENT(breaking)
				break; // start processing
			}
			if (fifoCount_left >= 42 && mpuInterruptLeft && readLeft)
			{
				// Data ready on the left
				mpuInterruptLeft = false;
				mpu = &mpu_left;
				tabs = "L ";
				readLeft = false;
				//record_time("breaking", micros(), __LINE__, loop_start_micro);
				TIME_EVENT(breaking)
				break; // start processing
			}

			// 
			// now wait
			// before sleep-waiting log battery voltage if we should
			//
			if (millis() > last_measure_milli + BATTERY_MEASURE_INTERVAL)
			{
				battery_capy_str = sardprintf("Batt Cap'y: %d/100", get_battery_capacity());
				battery_volts_str = sardprintf("Battery: %fV ", get_battery_voltage());
				logger_info(__LINE__, battery_volts_str + battery_capy_str);
				logger_info(__LINE__, sardprintf("Logfile size: %d", logfile.size()));
				last_measure_milli = millis();
			}

			// before sleep-waiting blank the display if we should

			if (get_turn_display_off())
			{
				display.ssd1306_command(0xAE);
				// a direct command to blank the display and go into sleep.  Hopefully it works!
				reset_turn_display_off(); // reset the blank display
				logger_info(__LINE__, "Display turned OFF after timed interval interrupt");
				//logger_debug(__LINE__, sardprintf("timerRead: %d", get_display_timer()));
			}
			//else
			//{
			//	logger_debug(__LINE__, sardprintf("timer: %d", get_display_timer()));
			//}


			// set an GPIO driven sleep depending on readLeft
#if 1
			long start = micros(); //start of sleep
			esp_sleep_enable_gpio_wakeup();
			if (readLeft)
			{
				gpio_wakeup_enable((gpio_num_t)INTERRUPT_PIN_LEFT, GPIO_INTR_HIGH_LEVEL);
				//logger_debug(__LINE__, sardprintf("Going 2 sleep on L intpt elapsed: %d us", micros() - loop_start_micro));

				//int pins[] = { INTERRUPT_PIN_LEFT ,INTERRUPT_PIN_RIGHT };
				//int num_pins = sizeof(pins) / sizeof(pins[0]);
				//while (true) logger_debug(__LINE__, dump_io_pins(pins, num_pins)); 

				esp_light_sleep_start();

				//logger_debug(__LINE__, sardprintf("Awaking fr L intpt after %d us (%d)", micros() - loop_start_micro, mpu_left.getFIFOCount()));
				// clear interrupt
				// mpu_left.getIntStatus();
				gpio_wakeup_disable((gpio_num_t)INTERRUPT_PIN_LEFT);
				mpuInterruptLeft = true;
				record_time("exit left sleep", micros(), __LINE__, loop_start_micro);
				if (!button_press_quick_check(SW3)) on_off_pushed = true;
				// check to see if the ok/off button pushed  TODO find a better way to do this
			}
#endif
#if 1
			else
			{
				gpio_wakeup_enable((gpio_num_t)INTERRUPT_PIN_RIGHT, GPIO_INTR_HIGH_LEVEL);
				//logger_debug(__LINE__, sardprintf("Going 2 sleep on R intpt elapsed: %d us", micros() - loop_start_micro));

				//int pins[] = { INTERRUPT_PIN_LEFT ,INTERRUPT_PIN_RIGHT };
				//int num_pins = sizeof(pins) / sizeof(pins[0]);
				//while (true) logger_debug(__LINE__, dump_io_pins(pins, num_pins));

				esp_light_sleep_start();

				//logger_debug(__LINE__, sardprintf("Awaking from right interrupt after %d us (%d) ", micros() - loop_start_micro, mpu_right.getFIFOCount()));
				// clear interrupt
				// mpu_right.getIntStatus();
				gpio_wakeup_disable((gpio_num_t)INTERRUPT_PIN_RIGHT);
				mpuInterruptRight = true;
				record_time("exit right sleep", micros(), __LINE__, loop_start_micro);
				if (!button_press_quick_check(SW3)) on_off_pushed = true;
				// check to see if the ok/off button pushed  TODO find a better way to do this
			}
			esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
#endif

			if ((millis() - loop_start) > WAIT_TIMEOUT_MS) //WAIT_TIMEOUT_MS
			{
				logger_error(__LINE__, sardprintf("Too long in interrupt wait loop (%d|%d|%d).  Checking MPU *********",
				                                  loop_start, millis() - loop_start, WAIT_TIMEOUT_MS));
				// TODO fix this problem - the following code is a KLUGE
				String status_now = mpu->getStatusString();
				if (mpu->init_status_string.equals(status_now))
				{
					logger_info(__LINE__, sardprintf("%s MPU status OK", mpu->label));
				}
				else
				{
					logger_error(__LINE__, sardprintf("%s MPU status *BAD* dumping config and recovering", mpu->label));
					logger_error(__LINE__, sardprintf("%s MPU original status: %s", mpu->label,
					                                  mpu->init_status_string.c_str()));
					logger_error(__LINE__, sardprintf("%s MPU current status: %s", mpu->label, status_now.c_str()));
					recover_MPU(*mpu);
				}
				loop_start = millis(); //reset loop_start to wait again
			}
		} // END While waiting for data

		record_time("end data wait", micros(), __LINE__, loop_start_micro);
		// 
		// now fetch and process the data
		//
		mpuIntStatus = mpu->getIntStatus();

		// get current FIFO count
		fifoCount = mpu->getFIFOCount();
		record_time("process INT", micros(), __LINE__, loop_start_micro);
		if (fifoCount < packetSize)
		{
			logger_error(__LINE__, sardprintf(
				             "What? %s fifoCount -%d- is now less than packetSize -%d- ! ..resetting FIFO",
				             mpu->label, fifoCount,
				             packetSize));
			mpu->resetFIFO();
			fifoCount = mpu->getFIFOCount(); // will be zero after reset
			if (fifoCount != 0)
			{
				logger_error(__LINE__, sardprintf("%s FIFO *BAD* reset. Checking MPU status", mpu->label));
				// check status
				String status_now = mpu->getStatusString();
				if (mpu->init_status_string.equals(status_now))
					logger_info(__LINE__, sardprintf("%s MPU status OK", mpu->label));
				else
				{
					logger_error(__LINE__, sardprintf("%s MPU status *BAD* dumping config and recovering", mpu->label));
					logger_error(__LINE__, sardprintf("%s MPU original status: %s", mpu->label,
					                                  mpu->init_status_string.c_str()));
					logger_error(__LINE__, sardprintf("%s MPU current status: %s", mpu->label, status_now.c_str()));
					recover_MPU(*mpu);
				}
			}
			else
			{
				logger_info(__LINE__, sardprintf("%s FIFO reset OK", mpu->label));
			}
			//Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
			// This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
		}

			// check for overflow (this should never happen unless our code is too inefficient)
		else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
		{
			logger_error(__LINE__, sardprintf("Resetting %s FIFO due to overflow.  Status:(%d) Count:(%d)", mpu->prefix, mpuIntStatus,
			                                  fifoCount));
			//logger_error(__LINE__, sardprintf("resetting %s FIFO", mpu->label));
			mpu->resetFIFO();
			fifoCount = mpu->getFIFOCount(); // will be zero after reset no need to ask
			if (fifoCount != 0)
			{
				logger_error(__LINE__, sardprintf("%s FIFO *BAD* reset. Checking MPU status", mpu->label));
				// check status
				String status_now = mpu->getStatusString();
				if (mpu->init_status_string.equals(status_now))
					logger_info(__LINE__, sardprintf("%s MPU status OK", mpu->label));
				else
				{
					logger_error(__LINE__, sardprintf("%s MPU status *BAD* dumping config and recovering", mpu->label));
					logger_error(__LINE__, sardprintf("%s MPU original status: %s", mpu->label,
					                                  mpu->init_status_string.c_str()));
					logger_error(__LINE__, sardprintf("%s MPU current status: %s", mpu->label, status_now.c_str()));
					recover_MPU(*mpu);
				}
			}
			else
			{
				logger_info(__LINE__, sardprintf("%s FIFO reset OK", mpu->label));
			}
		}
		else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
		{
			// read a packet from FIFO
			String msg = "fetching FIFO: " + String(fifoCount);
			record_time(msg, micros(), __LINE__, loop_start_micro);
			while (fifoCount >= packetSize)
			{
				// Lets catch up to NOW, someone is using the dreaded delay()!
				mpu->getFIFOBytes(fifoBuffer, packetSize);
				// track FIFO count here in case there is > 1 packet available
				// (this lets us immediately read more without waiting for an interrupt)
				fifoCount -= packetSize;
				// Serial.println(F("fifoBuffer loaded"));
			}
			record_time("packets fetched", micros(), __LINE__, loop_start_micro);

#ifdef RECORD_READABLE_QUATERNION
			// write quaternion values in easy matrix form: w x y z
			mpu->dmpGetQuaternion(&q, fifoBuffer);
			if ((q.w * q.w + q.y * q.y + q.x * q.x + q.z * q.z) > 1.05)
			{
				logger_error(__LINE__, sardprintf("%s quaternion out of range.  Resetting FIFO", mpu->prefix));
				logger_error(__LINE__, sardprintf("%s,%f,%f,%f,%f", mpu->prefix, q.w, q.x, q.y, q.z));
				mpu->resetFIFO();
				fifoCount = mpu->getFIFOCount(); // will be zero after reset no need to ask
				if (fifoCount != 0)
				{
					logger_error(__LINE__, sardprintf("%s FIFO *BAD* reset", mpu->label));
				}
				else
				{
					logger_info(__LINE__, sardprintf("%s FIFO reset OK", mpu->label));
				}
			}
			else
			{
				logger_data(__LINE__, sardprintf("%s,%f,%f,%f,%f", mpu->prefix, q.w, q.x, q.y, q.z));
			}
#endif

#ifdef RECORD_ALL
			// write the entire FIFO
			{
				mpu->dmpGetAccel(&aa, fifoBuffer);
				// mpu->dmpGetGyro(&gg, fifoBuffer);
				mpu->dmpGetQuaternion(&q, fifoBuffer);
				if (std::sqrt(q.w * q.w + q.y * q.y + q.x * q.x + q.z * q.z) > 1.05)
				{
					logger_error(__LINE__, sardprintf("%s quaternion out of range.  Resetting FIFO", mpu->prefix));
					logger_error(__LINE__, sardprintf("%s,%f,%f,%f,%f", mpu->prefix, q.w, q.x, q.y, q.z));

					mpu->resetFIFO();
					fifoCount = mpu->getFIFOCount(); // will be zero after reset no need to ask
					if (fifoCount != 0)
					{
						logger_error(__LINE__, sardprintf("%s FIFO *BAD* reset", mpu->label));
					}
					else
					{
						logger_info(__LINE__, sardprintf("%s FIFO reset OK", mpu->label));
					}
				}
				else
				{
					logger_data(__LINE__, sardprintf("%s,%f,%f,%f,%f,%f,%f,%f,%f", mpu->prefix, q.w, q.x, q.y, q.z, aa.x, aa.y, aa.z));
				}
			}
#endif

#ifdef OUTPUT_READABLE_QUATERNION
			// display quaternion values in easy matrix form: w x y z
			mpu->dmpGetQuaternion(&q, fifoBuffer);
			//Serial.print("!");
			Serial.print(tabs);
			long j = millis();
			Serial.print(j);
			Serial.print(" (");
			Serial.print(j - old_millis);
			old_millis = j;
			Serial.print(")\t");
			Serial.print(q.w);
			Serial.print("\t");
			Serial.print(q.x);
			Serial.print("\t");
			Serial.print(q.y);
			Serial.print("\t");
			Serial.println(q.z);
#endif

#ifdef LOG_READABLE_QUATERNION
			// log quaternion values in easy matrix form: w x y z
			mpu->dmpGetQuaternion(&q, fifoBuffer);

			String msg = "";


			//Serial.print("!");
			//Serial.print(tabs);
			//long now = millis();
			////Serial.print(k);
			//msg += (" (");
			//msg += String(now - old_millis);
			//old_millis = now;
			//msg+=(")  ");
			msg = String(*tabs)[0] + " ";
			msg = String(*tabs);
			msg += (q.w);
			msg += ("\t");
			msg += (q.x);
			msg += ("\t");
			msg += (q.y);
			msg += ("\t");
			msg += (q.z);
			//Serial.println(msg);
			LOG_DATA(msg);
#endif

#ifdef OUTPUT_READABLE_EULER
			// display Euler angles in degrees
			mpu->dmpGetQuaternion(&q, fifoBuffer);
			mpu->dmpGetEuler(euler, &q);
			Serial.print("euler\t");
			Serial.print(euler[0] * 180 / M_PI);
			Serial.print("\t");
			Serial.print(euler[1] * 180 / M_PI);
			Serial.print("\t");
			Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
			// display Euler angles in degrees
			mpu->dmpGetQuaternion(&q, fifoBuffer);
			mpu->dmpGetGravity(&gravity, &q);
			mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);
			Serial.print("ypr\t");
			Serial.print(ypr[0] * 180 / M_PI);
			Serial.print("\t");
			Serial.print(ypr[1] * 180 / M_PI);
			Serial.print("\t");
			Serial.println(ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
			// display real acceleration, adjusted to remove gravity
			mpu->dmpGetQuaternion(&q, fifoBuffer);
			mpu->dmpGetAccel(&aa, fifoBuffer);
			mpu->dmpGetGravity(&gravity, &q);
			mpu->dmpGetLinearAccel(&aaReal, &aa, &gravity);
			Serial.print("areal\t");
			Serial.print(aaReal.x);
			Serial.print("\t");
			Serial.print(aaReal.y);
			Serial.print("\t");
			Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
			// display initial world-frame acceleration, adjusted to remove gravity
			// and rotated based on known orientation from quaternion
			mpu->dmpGetQuaternion(&q, fifoBuffer);
			mpu->dmpGetAccel(&aa, fifoBuffer);
			mpu->dmpGetGravity(&gravity, &q);
			mpu->dmpGetLinearAccel(&aaReal, &aa, &gravity);
			mpu->dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
			Serial.print("aworld\t");
			Serial.print(aaWorld.x);
			Serial.print("\t");
			Serial.print(aaWorld.y);
			Serial.print("\t");
			Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
			// display quaternion values in InvenSense Teapot demo format:
			teapotPacket[2] = fifoBuffer[0];
			teapotPacket[3] = fifoBuffer[1];
			teapotPacket[4] = fifoBuffer[4];
			teapotPacket[5] = fifoBuffer[5];
			teapotPacket[6] = fifoBuffer[8];
			teapotPacket[7] = fifoBuffer[9];
			teapotPacket[8] = fifoBuffer[12];
			teapotPacket[9] = fifoBuffer[13];
			Serial.write(teapotPacket, 14);
			teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif
		} // end of recording accelerometer data
		else
		{
			// we have an unrecognized situation
			logger_error(__LINE__, sardprintf(
				             "Unrecognized DMP situation on %s side.  Interrupt status: %d  Fifocount: %d",
				             mpu->label, mpuIntStatus, fifoCount));
			logger_error(__LINE__, "MPU configs ------------");
			logger_error(__LINE__, mpu_left.getStatusString());
			logger_error(__LINE__, mpu_right.getStatusString());
			//logger_error(__LINE__, sardprintf("Recovering %s MPU and continuing", mpu->label));
			//recover_MPU(* mpu);

			String status_now = mpu->getStatusString();
			//		mpu->init_status_string.concat("1"); //TODO remove this it's a test
			if (mpu->init_status_string.equals(status_now))
				logger_info(__LINE__, sardprintf("%s MPU status is unchanged, no recovery will be tried", mpu->label));
			else
			{
				logger_error(
					__LINE__, sardprintf("%s MPU status has *CHANGED*.  Dumping config and recovering", mpu->label));
				logger_error(
					__LINE__, sardprintf("%s MPU original status: %s", mpu->label, mpu->init_status_string.c_str()));
				logger_error(__LINE__, sardprintf("%s MPU current status: %s", mpu->label, status_now.c_str()));
				recover_MPU(*mpu);
			}
		}
	}
}

// ================================================================
// ===                      STATES                              ===
// ================================================================

void recording_state()
{
	system_State.current = recording;
	logger_debug(__LINE__, "Starting recording");
	logger_data(__LINE__, "Recording started"); // log the start of recording in DATA

	//log_core(__LINE__);

	// display a splash display
	int button_pins[] = {SW1, SW2}; // these are the buttons we'll look for input from
	int num_pins = sizeof(button_pins) / sizeof(button_pins[0]);
	//display_and_blank("Starting to record", "", "Please carry on!", button_pins, num_pins, DISPLAY_BLANK_INTERVAL);
	logger_debug(__LINE__, "Display > Starting to record"); 
	display_and_blank(String("Starting to record"), log_filename, String("Chive on!!!"));

	// gather and record loop
	// NOTE: this loop is used by several states and has calling-state-specific logic.  If the code returns
	// the next state has been changed simply return.  If *unchanged* figure out what to do
	while (system_State.next == recording)
	{
		record_loop();
		/* we've come here because there's been a menu button pressed.  Flash status and resume */
		yield();
		delay(100);
		logger_debug(__LINE__, String("Status button push"));
		logger_info(__LINE__, "Display > STATUS UPDATE");
		display_and_blank(sardprintf("STATUS:%s",system_Status.overall_status.msg), battery_capy_str, 
			sardprintf("Log size: %d", logfile.size()));
	}
	// if we fall out of this loop it's because next state != this state
}


/**
 * \brief Waiting state
 * This state is waiting on the user to either calibrate or start recording
 */
void waiting_state()
{
	system_State.current = waiting;
	logger_debug(__LINE__, "Starting waiting state");
	logger_debug(__LINE__, "Display > Calibration choice");

	display_and_hold("", "CHOOSE", "", "record", "calibrate");

	// waiting for the interrupt to trigger TODO fix this
	for (auto i = 0; i < 10; i++)
	{
		//logger_debug(__LINE__, sardprintf("timerRead: %d.", get_display_timer(),
		//                                  get_turn_display_off()));
		delay(100);
	}
	if (get_turn_display_off()) reset_turn_display_off();

	int button_pins[] = {SW1, SW2}; // these are the buttons we'll look for input from
	int num_pins = sizeof(button_pins) / sizeof(button_pins[0]);

	// gather and record loop
	// NOTE: this loop is used by several states and has calling-state-specific logic.  If the code returns
	// the next state has been changed simply return.  If *unchanged* figure out what to do
	while (system_State.next == waiting)
	{
		record_loop();
		/* we've come here because there's been a menu button pressed.  Find out what it is
		 *  record = SW1
		 *  calibrate = SW2 */
		button temp_button = find_button_pressed(button_pins, num_pins);
		if (temp_button.pin == (gpio_num_t)SW1) // record
		{
			yield();
			delay(250); // let the switch disengage - avoid false second trigger
			system_State.next = recording;
			logger_debug(__LINE__, String("Going to record "));
			break;
		}
		else if (temp_button.pin == (gpio_num_t)SW2) // calibrate
		{
			yield();
			delay(250); // let the switch disengage - avoid false second trigger
			system_State.next = calibrate;
			logger_debug(__LINE__, String("Going to calibration "));
			break;
		}
		else
		{
			// we exited record_loop without a valid key pressed.  Log and reenter
			yield();
			delay(250); // let the switch disengage - avoid false second trigger
			logger_error(__LINE__, String("Did not get valid button"));
			display_and_blank("Exited record loop", "", "Without button");
		}
	}
	// if we fall out of this loop it's because next state != this state, so go there
}

/**
 * \brief Calibration
 * This state involves having the user indicate when his/her head is level
 */
void calibration()
{
	system_State.current = calibrate;

	display_and_hold("", "Press any", "button", "this", "that");

	int button_pins[] = {SW1, SW2}; // these are the buttons we'll look for input from
	int num_pins = sizeof(button_pins) / sizeof(button_pins[0]);

	while (system_State.next == calibrate)
	{
		record_loop();
		/* we've come here because there's been a menu button pressed.  Find out what it is
		 *  record = SW1
		 *  calibrate = SW2 */
		const button temp_button = find_button_pressed(button_pins, num_pins);
		if (temp_button.pin == (gpio_num_t)SW1 | temp_button.pin == (gpio_num_t)SW2)
		{
			yield();
			logger_data(__LINE__, "Calibration button pushed"); // message for data collection
			logger_debug(__LINE__, String("Going to record "));
			delay(200); // let the switch disengage - avoid false second trigger
			system_State.next = recording;
			break;
		}

		else
		{
			// we exited record_loop without a valid key pressed.  Log and reenter
			yield();
			delay(250); // let the switch disengage - avoid false second trigger
			logger_error(__LINE__, String("Did not get valid button"));
			display_and_blank("No button!", "", "Recalibrating");
		}
	}
	// if we fall out of this loop it's because next state != this state, so go there
}

/**
 * \brief going_off_state
 * This state involves having the user confirm he/she wants to turn off the system.
 * Otherwise go back to begin if SW3 is pressed and latched
 */
void going_off_state()
{
	system_State.current = going_off;

	logger_debug(__LINE__, " Starting going_off state");

	//int button_pins[] = {SW3}; // these are the buttons we'll look for input from
	//int num_pins = sizeof(button_pins) / sizeof(button_pins[0]);

	const int checks_per_interval = 2;

	// display a countdown while looking for SW3 to be released
	for (auto i = SYSTEM_OFF_DELAY; i > 0; i--)
	{
		logger_debug(__LINE__, sardprintf("going_off cycle %d", i));
		// show a countdown message
		//button temp_button = display_and_return_button(sardprintf("Power down in %d secs", i).c_str(), system_Status,
		//                                               "To cancel release SW3", "", "", button_pins, num_pins,
		//                                               DISPLAY_BLANK_INTERVAL/3);

		display_and_hold(sardprintf("Power down in %d secs", i), "To cancel re-press SW3", "");

		yield();
		delayMicroseconds(DISPLAY_BLANK_INTERVAL / 3);

		if (read_button_pressed(SW3))
		{
			logger_info(__LINE__, "Shutdown cancelled by button push");
			on_off_switch_reset();
			system_State.previous = system_State.current;
			system_State.next = begin;
			delay(250);
			display_and_blank("Cancel shutdown", system_Status, "");
			return;
		}
		//for (auto j = 0; j < num_pins; j++)
		//{
		//	if (temp_button.pin == (gpio_num_t)button_pins[j])
		//	{
		//		// we have a valid cancel button press.  Cancel and restart

		//	}
		//}
		////logger_debug(__LINE__, "No cancel button press detected");
	}
	logger_info(__LINE__, "Timeout reached.  Shutdown starts");
	system_State.previous = system_State.current;
	system_State.next = off;
}

/**
 * \brief off_state
 * This state involves going into low power mode, waiting for SW3 to wake up
 */
void off_state()
{
	logger_debug(__LINE__, " Starting off state");

	// shut down peripherals
	blank_display();
	mpu_left.setSleepEnabled(true);
	mpu_right.setSleepEnabled(true);

	// system comes out of sleep when SW3 is off and pullups take input to HIGH
#define LOGIC_LEVEL 0
	esp_sleep_enable_ext0_wakeup((gpio_num_t)SW3, LOGIC_LEVEL);

	// "Configured all RTC Peripherals to be powered down in sleep
	//esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);

	rtc_gpio_pullup_en((gpio_num_t)SW3);
	esp_deep_sleep_start();
}

/**
 * \brief begin state
 * This state involves initializing things before taking on startup
 */
void begin_state()
{
	system_State.current = going_off;
	logger_debug(__LINE__, "Starting begin state");
	// TODO actually write this

	system_State.previous = system_State.current;
	system_State.next = startup; //TODO set this to startup and get it working!
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
	// This is a state machine implementation
	//logger_debug(__LINE__, "Starting state loop");

	switch (system_State.next)
	{
	case going_off:
		going_off_state();
		break;
	case off:
		off_state();
		break;
	case begin:
		begin_state();
		break;
	case startup:
		setup1();
		break;
	case waiting:
		waiting_state();
		break;
	case calibrate:
		calibration();
		break;
	case recording:
		recording_state();
		break;

	case none:
	default:
		logger_error(__LINE__, "Unexpected none state.  Restarting");
		system_State.previous = none;
		system_State.next = begin;
		break;
	}
}