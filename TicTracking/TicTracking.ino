// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//


/* ============================================
I2Cdev device library code is placed under the MIT license

===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

#include "Logging.h"
#include <I2Cdev.h>
//#include "MPU6050_6Axis_MotionApps20.h"
#include "Multi_I2C.h"
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <RTClib.h>

/*****************************************************************************
 *
 * option switches
 *
 *****************************************************************************/
//#define TIMING
#define LOGGING
#define LOGFILE "/Logs/Log xxxx - yyyy.txt"  //where xxxx = date and yyyy = daily sequence

String msg;
#define BATTERY_MEASURE_INTERVAL  10*60*1000 //number of millis() between measurements
unsigned long last_measure_milli = 0; //records when last measurement was logged


#define SAMPLE_RATE 10
#define WAIT_TIMEOUT_MS  (1000/(SAMPLE_RATE) * 2) //timeout interval (mS) waiting for interrupt from accelerometer
#define GYRO_RATE 1000

#define RECORD_READABLE_QUATERNION
// #define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_ALL
//#define RECORD_ALL // uncomment "RECORD_ALL" if you want to write the actual data straight out of the FIFO

/////////////////////////////////////
/// Testing
/////////////////////////////////////
long last_test_milli = 0; // these two parameters allow for periodic testing
#define TEST_MEASURE_INTERVAL 20*1000 // milliseconds


/*****************************************************************************
 *
 * Timing measurement
 *
 *****************************************************************************/

long start = 0; // defines top of loop
#ifdef TIMING
#define TIME_START start=millis();
#define TIME_EVENT(msg) record_time(#msg,millis(), __LINE__, start);
	

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
#endif
}

/*****************************************************************************
 *
 * Logging
 *
 *****************************************************************************/
RTC_PCF8523 rtc;
File logfile; // for storing all log information
String filepath;

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


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)


// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
// #define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT


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

#define INTERRUPT_PIN_LEFT 13  // use pin 2 on Arduino Uno & most boards
#define AD0_PIN_LEFT 12  // use pin 2 on Arduino Uno & most boards
#define INTERRUPT_PIN_RIGHT 14   // use pin 2 on Arduino Uno & most boards
#define AD0_PIN_RIGHT 32  // use pin 2 on Arduino Uno & most boards
// #define LED_PIN 2 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
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

auto readLeft = false;
long old_millis = 0; //for calculating dump intervals


// ================================================================
// ===               INTERRUPT SERVICE ROUTINE                ===
// ================================================================

volatile bool mpuInterruptRight = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReadyRight()
{
	mpuInterruptRight = true;
#ifdef OUTPUT_INTERRUPT
	Serial.print("ri-");
	Serial.println(millis());
#endif
}


volatile bool mpuInterruptLeft = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReadyLeft()
{
	mpuInterruptLeft = true;
#ifdef OUTPUT_INTERRUPT
	Serial.print("li-");
	Serial.println(millis());
#endif
}

// ================================================================
// ===               GET BATTERY LEVEL                          ===
// ================================================================

//come here to get battery level

const int battery_pin = A13;
const int full_scale = 3646; //full scale output from ADC
const float full_voltage[] = {4.2, 3.95, 3.8, 3.75, 3.65, 3.0}; // from 0 to 100% discharge in 20% increments

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
// ===                      LIST DIRECTORIES                    ===
// ================================================================

void listDir(fs::FS& fs, const char* dirname, uint8_t levels)
{
	Serial.printf("Listing directory: %s\n", dirname);

	File root = fs.open(dirname);
	if (!root)
	{
		Serial.println("Failed to open directory");
		return;
	}
	if (!root.isDirectory())
	{
		Serial.printf("%s is not a directory\n", dirname);
		return;
	}

	File file = root.openNextFile();
	while (file)
	{
		if (file.isDirectory())
		{
			Serial.print("  DIR : ");
			Serial.println(file.name());
			if (levels)
			{
				listDir(fs, file.name(), levels - 1);
			}
		}
		else
		{
			Serial.print("  FILE: ");
			Serial.print(file.name());
			Serial.print("  SIZE: ");
			Serial.println(file.size());
		}
		file = root.openNextFile();
	}
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
		mpu.PrintActiveOffsets();
		// turn on the DMP, now that it's ready
		//Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		//Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
		//Serial.print(digitalPinToInterrupt(interruptPin));
		//Serial.println(F(")..."));
		//		if (interruptPin == INTERRUPT_PIN_LEFT)
		//		{
		//attachInterrupt(digitalPinToInterrupt(interruptPin), dmpDataReadyLeft, HIGH);
		//mpuIntStatus_left = mpu.getIntStatus();
		//Serial.println("Set left interrupt pin");
		//	Serial.println("NOT SET: left interrupt pin");
		//	Serial.print("Left ");
		//}
		//else if (interruptPin == INTERRUPT_PIN_RIGHT)
		//{
		//attachInterrupt(digitalPinToInterrupt(interruptPin), dmpDataReadyRight,HIGH );
		//mpuIntStatus_right = mpu.getIntStatus();
		//Serial.println("Set right interrupt pin");
		//	Serial.println("NOT SET: right interrupt pin");
		//	Serial.print("Right ");
		//}

		// instead set interrupt to be latching /  positive clearing
		//mpu.setInterruptLatch(false);
		//mpu.setInterruptLatchClear(true);

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready!"));

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();

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
	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
}

bool recover_MPU(Multi_MPU& mpu)
// come here as a last resort to recover an MPU that has lost its mind
{
	uint8_t devStatus; // return init_status_string after each device operation (0 = success, !0 = error)
	// initialize device
	Serial.printf(F("Recovering %s MPU...at address "), mpu.label);

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
// ===                      WRITE FILES                         ===
// ================================================================

void writeFile(fs::FS& fs, const char* path, const char* message)
{
	Serial.printf("Writing file: %s\n", path);

	File file = fs.open(path, FILE_WRITE);
	if (!file)
	{
		Serial.println("Failed to open file for writing");
		return;
	}
	if (file.print(message))
	{
		Serial.println("File written");
	}
	else
	{
		Serial.println("Write failed");
	}
	file.close();
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
	Serial.begin(230400);

	delay(5000); // TODO remove this one once you figure out how to slow things down
	while (!Serial); // wait for Leonardo enumeration, others continue immediately
	//Serial.println("Hello");

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
	Log_Level excludes[] = {data}; //exclude data logs because it's so voluminous
	set_logger_filter(excludes, sizeof(excludes) / sizeof(excludes[0]), 10);
	if (!SD.begin())
	{
		Serial.println("SD initialization failed! ... stopping");
		while (1);
	}
	else
	{
		Serial.println("SD initialization success");
	}

	//Next, turn off the MPU6050's by moving their I2C addresses to the higher value AD0
	digitalWrite(AD0_PIN_LEFT, HIGH);
	digitalWrite(AD0_PIN_RIGHT, HIGH);
	if (!rtc.begin())
	{
		Serial.println("Couldn't find RTC... stopping");
		while (1);
	}
	else
	{
		Serial.println("RTC initialization success");
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
		filepath = file_path_yyyy;
		filepath.replace("yyyy", seq);
		//Serial.println(filepath);
		if (SD.exists(filepath))
		{
			//Serial.printf("Log file: %s exists. Will try another...\n\r", filepath.c_str());
		}
		else
		{
			Serial.printf("Log file: %s is available\n\r", filepath.c_str());
			logfile = logger_setup(filepath);
			break;
		}
	}
	// put some initial entries into the file
	//Serial.printf("Log file: %s is available\n\r", filepath.c_str());

	//Serial.println(msg);
	//Serial.println(sardprintf("Battery voltage: %f Capacity: %d/100", get_battery_voltage(),
	//	get_battery_capacity()));
	msg = "Opening " + filepath + " at " + now.timestamp();
	logger_info(__LINE__, msg);
	logger_info(__LINE__, sardprintf("Battery voltage: %f Capacity: %d/100", get_battery_voltage(),
	                                 get_battery_capacity()));


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


	while (!dmpReady)
	{
		// initialize device
		Serial.print(F("Initializing I2C devices...at "));
		// Serial.println(mpu_left.getDeviceID());
		mpu_left.initialize();
		mpu_right.initialize();

		Serial.print(mpu_left.getDeviceID(), HEX);
		Serial.println(" and " + String(mpu_right.getDeviceID(), HEX));

		//logger_debug(__LINE__, mpu_left.getStatusString());
		//logger_debug(__LINE__, mpu_right.getStatusString());

		mpu_left.label = "Left ";
		mpu_left.prefix = "L";
		mpu_right.prefix = "R";
		mpu_right.label = "Right ";

		// verify connection
		//Serial.println(F("Testing device connections..."));
		Serial.println(mpu_right.testConnection()
			               ? F("MPU6050 right side connection successful")
			               : F("MPU6050 right side connection failed"));
		Serial.println(mpu_left.testConnection()
			               ? F("MPU6050 left side connection successful")
			               : F("MPU6050 left side connection failed"));

		// load and configure the DMP`s
		Serial.println(F("Initializing right DMP..."));
		devStatus_right = mpu_right.dmpInitialize();
		Serial.println(F("Initializing left DMP..."));
		devStatus_left = mpu_left.dmpInitialize();
		// supply your own gyro offsets here, scaled for min sensitivity
		mpu_left.setXGyroOffset(220);
		mpu_left.setYGyroOffset(76);
		mpu_left.setZGyroOffset(-85);
		mpu_left.setZAccelOffset(1788); // 1688 factory default for my test chip
		mpu_right.setXGyroOffset(220);
		mpu_right.setYGyroOffset(76);
		mpu_right.setZGyroOffset(-85);
		mpu_right.setZAccelOffset(1788); // 1688 
		initializeDmp(devStatus_right, INTERRUPT_PIN_RIGHT, mpu_right);
		initializeDmp(devStatus_left, INTERRUPT_PIN_LEFT, mpu_left);

		dmpReady = (devStatus_right == 0) && (devStatus_left == 0);
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

	// pinMode(LED_PIN, OUTPUT);
} // end setup()


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
	// Serial.println("Starting loop");
	if (!dmpReady)
	{
		Serial.println("Something dun fucked up");
		return;
	}

	// wait for MPU interrupt or extra packet(s) available
	TIME_START

	//TIME_EVENT"loop start"

	char* tabs = "R\t\t\t\t\t";
	long loop_start = millis(); // for calculating wait timeout
	long loop_start_micro = micros(); // for calculating wait timeout

	record_time("loop start", micros(), __LINE__, loop_start_micro);

	//readLeft = false; //TODO for testing only.  It forces system to read only one accelerometer
	while (true)
	{
		fifoCount_left = mpu_left.getFIFOCount();
		fifoCount_right = mpu_right.getFIFOCount();
		record_time("got FIFO's", micros(), __LINE__, loop_start_micro);
		// Serial.println(readLeft);
		if (fifoCount_right >= 42 && mpuInterruptRight && !readLeft)
		{
			// Data ready on the right
			mpuInterruptRight = false;
			mpu = &mpu_right;
			readLeft = true;
			record_time("breaking", micros(), __LINE__, loop_start_micro);
			break;
		}

		if (fifoCount_left >= 42 && mpuInterruptLeft && readLeft)
		{
			// Data ready on the left
			mpuInterruptLeft = false;
			mpu = &mpu_left;
			tabs = "L ";
			readLeft = false;
			record_time("breaking", micros(), __LINE__, loop_start_micro);
			break;
		}

		//
		// we are now waiting until there is data
		//

		// first log battery voltage if we should
		if (millis() > last_measure_milli + BATTERY_MEASURE_INTERVAL)
		{
			logger_info(__LINE__, sardprintf("Battery voltage: %f Capacity: %d/100", get_battery_voltage(),
			                                 get_battery_capacity()));
			last_measure_milli = millis();
			logger_info(__LINE__, sardprintf("Logfile size: %d", logfile.size()));
			last_measure_milli = millis();
		}


		// now set an interrupt driven sleep depending on readLeft
#if 1
		long start = micros(); //start of sleep
		esp_sleep_enable_gpio_wakeup();
		if (readLeft)
		{
			gpio_wakeup_enable((gpio_num_t)INTERRUPT_PIN_LEFT, GPIO_INTR_HIGH_LEVEL);
			//logger_debug(__LINE__, sardprintf("Going 2 sleep on L intpt elapsed: %d us", micros() - loop_start_micro));
			esp_light_sleep_start();

			//logger_debug(__LINE__, sardprintf("Awaking fr L intpt after %d us (%d)", micros() - loop_start_micro, mpu_left.getFIFOCount()));
			// clear interrupt
			// mpu_left.getIntStatus();
			gpio_wakeup_disable((gpio_num_t)INTERRUPT_PIN_LEFT);
			mpuInterruptLeft = true;
			record_time("exit left sleep", micros(), __LINE__, loop_start_micro);
		}
#endif
#if 1
		else
		{
			gpio_wakeup_enable((gpio_num_t)INTERRUPT_PIN_RIGHT, GPIO_INTR_HIGH_LEVEL);
			//logger_debug(__LINE__, sardprintf("Going 2 sleep on R intpt elapsed: %d us", micros() - loop_start_micro));

			esp_light_sleep_start();

			//logger_debug(__LINE__, sardprintf("Awaking from right interrupt after %d us (%d) ", micros() - loop_start_micro, mpu_right.getFIFOCount()));
			// clear interrupt
			// mpu_right.getIntStatus();
			gpio_wakeup_disable((gpio_num_t)INTERRUPT_PIN_RIGHT);
			mpuInterruptRight = true;
			record_time("exit right sleep", micros(), __LINE__, loop_start_micro);
		}

#endif

#if 0
	// we're going to loop and wait for an interrupt.  Let's try sleeping
#define uS_TO_mS_FACTOR 1000  /* Conversion factor for micro seconds to milliseconds */
#define TIME_TO_SLEEP  1 
		esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_mS_FACTOR);
		//Serial.println(sardprintf("z-%d",micros()));
		esp_light_sleep_start();

		esp_sleep_wakeup_cause_t wakeup_reason;

		wakeup_reason = esp_sleep_get_wakeup_cause();

		switch (wakeup_reason)
		{
		case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal using RTC_IO"); break;
		case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
		case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
		case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
		case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup caused by ULP program"); break;
		default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
		}
#endif

		if (millis() - loop_start > WAIT_TIMEOUT_MS) //WAIT_TIMEOUT_MS
		{
			logger_error(__LINE__, "Too long in interrupt wait loop.  Checking MPU *********");
			// TODO fix this problem - the following codes is a KLUGE
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
	} // END While true

	record_time("end true", micros(), __LINE__, loop_start_micro);
	mpuIntStatus = mpu->getIntStatus();

	//  some testing code to remove later
#if 0
	if (millis() > (last_measure_milli + TEST_MEASURE_INTERVAL))
	{
		//mpuIntStatus |= _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT);
		mpuIntStatus |= _BV(MPU6050_INTERRUPT_I2C_MST_INT_BIT);
		mpuIntStatus &= !(_BV(MPU6050_INTERRUPT_DMP_INT_BIT));
		char* test_msg = "Changing MPU interrupt init_status_string to MPU6050_INTERRUPT_I2C_MST_INT";
		logger_info(__LINE__, sardprintf("A test condition injection- %s", test_msg));
		last_measure_milli = millis();
		Serial.println(mpuIntStatus, BIN);
	}
#endif

	// get current FIFO count
	fifoCount = mpu->getFIFOCount();
	record_time("process INT", micros(), __LINE__, loop_start_micro);
	if (fifoCount < packetSize)
	{
		logger_error(__LINE__, sardprintf("What? %s fifoCount -%d- is now less than packetSize -%d- ! ..resetting FIFO",
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
		logger_error(__LINE__, sardprintf("%s FIFO overflow.  Status:(%d) Count:(%d)", mpu->prefix, mpuIntStatus,
		                                  fifoCount));
		logger_error(__LINE__, sardprintf("resetting %s FIFO", mpu->label));
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

			//myFile.print(",");
			//myFile.print(millis());
			//myFile.print(",");
			//myFile.print(String(tabs[0])[0]);
			//myFile.print(",");
			//myFile.print(q.w);
			//myFile.print(",");
			//myFile.print(q.x);
			//myFile.print(",");
			//myFile.print(q.y);
			//myFile.print(",");
			//myFile.println(q.z);
			//line_count++;

			//if (line_count >= FLUSH_LIMIT) {
			//	// write the lines into the SD card if at FLUSH_LIMIT
			//	myFile.flush();
			//	line_count = 0;
			//}
		}
#endif

#ifdef RECORD_ALL
		// write the entire FIFO
		mpu->dmpGetAccel(&aa, fifoBuffer);
		mpu->dmpGetGyro(&gg, fifoBuffer);
		mpu->dmpGetQuaternion(&q, fifoBuffer);
		myFile.print(millis());
		myFile.print(",");
		myFile.print(q.w);
		myFile.print(",");
		myFile.print(q.x);
		myFile.print(",");
		myFile.print(q.y);
		myFile.print(",");
		myFile.print(q.z);
		myFile.print(",");

		myFile.print(aa.x);
		myFile.print(",");
		myFile.print(aa.y);
		myFile.print(",");
		myFile.print(aa.z);
		myFile.print(",");

		myFile.print(gg.x);
		myFile.print(",");
		myFile.print(gg.y);
		myFile.print(",");
		myFile.println(gg.z);
		line_count++;

		if (line_count >= FLUSH_LIMIT) {
			// write the lines into the SD card if at FLUSH_LIMIT
			myFile.flush();
			line_count = 0;
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
		Serial.print(j-old_millis);
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
		msg = String(*tabs)[0]+ " ";
		msg = String(*tabs);
		msg+=(q.w);
		msg+=("\t");
		msg+=(q.x);
		msg+=("\t");
		msg+=(q.y);
		msg+=("\t");
		msg+=(q.z);
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
		//logger_debug(__LINE__, sardprintf("End of accel loop %d us", micros() - loop_start_micro));
	} // end of recording accelerometer data
	else
	{
		// we have an unrecognized situation
		logger_error(__LINE__, sardprintf(
			             "Unrecognized DMP situation on %s side.  Interrupt init_status_string: %d  Fifocount: %d",
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
