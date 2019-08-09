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
// #include "MPU6050_6Axis_MotionApps20.h"
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
#define TIMING
//#define LOGGING
#define LOGFILE "/Logs/Log xxxx - yyyy.txt"  //where xxxx = date and yyyy = daily sequence
String msg;

#define SAMPLE_RATE 50
#define WAIT_TIMEOUT_MS  1000/SAMPLE_RATE * 2 //timeout interval (mS) waiting for interrupt from accelerometer
#define GYRO_RATE 1000

#define RECORD_READABLE_QUATERNION
// #define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_ALL
//#define RECORD_ALL // uncomment "RECORD_ALL" if you want to write the actual data straight out of the FIFO



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

//#define OUTPUT_INTERRUPT // a preprocessor option to print interrupt information

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


//
//#define OUTPUT_ALL
//#define RECORD_ALL
#ifdef RECORD_ALL
#define RECORD_HEADER "mSec, qw, qx, qy, qz, ax, ay, az, gx, gy, gz"
#define FLUSH_LIMIT 20
int line_count = 0; // counts lines prior to flushing
#endif
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
Multi_MPU *mpu;
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

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus_left;   // holds actual interrupt status byte from MPU
uint8_t mpuIntStatus_right;   // holds actual interrupt status byte from MPU
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus_right;      // return status after each device operation (0 = success, !0 = error)
uint8_t devStatus_left;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint16_t fifoCount_left = 0;     // count of all bytes currently in FIFO
uint16_t fifoCount_right = 0;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

auto readLeft = false;

long old_millis = 0;  //for calculating dump intervals



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterruptRight = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReadyRight() {
	mpuInterruptRight = true;
#ifdef OUTPUT_INTERRUPT
	Serial.print("ri-");
	Serial.println(millis());
#endif
}


volatile bool mpuInterruptLeft = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReadyLeft() {
	mpuInterruptLeft = true;
#ifdef OUTPUT_INTERRUPT
	Serial.print("li-");
	Serial.println(millis());
#endif
}

String getstring()
{
	// return a String from the keyboard
	String msg = "";
	while (Serial.available() > 0){
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

void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
	Serial.printf("Listing directory: %s\n", dirname);

	File root = fs.open(dirname);
	if (!root) {
		Serial.println("Failed to open directory");
		return;
	}
	if (!root.isDirectory()) {
		Serial.printf("%s is not a directory\n", dirname);
		return;
	}

	File file = root.openNextFile();
	while (file) {
		if (file.isDirectory()) {
			Serial.print("  DIR : ");
			Serial.println(file.name());
			if (levels) {
				listDir(fs, file.name(), levels - 1);
			}
		}
		else {
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

boolean initializeDmp(int devStatus, int interruptPin, Multi_MPU& mpu)
{
	if (devStatus == 0) {
		// Calibration Time: generate offsets and calibrate our MPU6050
		mpu.CalibrateAccel(6);
		mpu.CalibrateGyro(6);
		mpu.PrintActiveOffsets();
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
		Serial.print(digitalPinToInterrupt(interruptPin));
		Serial.println(F(")..."));
		if (interruptPin == INTERRUPT_PIN_LEFT)
		{
			attachInterrupt(digitalPinToInterrupt(interruptPin), dmpDataReadyLeft, RISING);
			mpuIntStatus_left = mpu.getIntStatus();
			Serial.println("Set left interrupt pin");
		}
		else if (interruptPin == INTERRUPT_PIN_RIGHT)
		{
			attachInterrupt(digitalPinToInterrupt(interruptPin), dmpDataReadyRight, RISING);
			mpuIntStatus_right = mpu.getIntStatus();
			Serial.println("Set right interrupt pin");
		}

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
		return true;

	}
	// ERROR!
	// 1 = initial memory load failed
	// 2 = DMP configuration updates failed
	// (if it's going to break, usually the code will be 1)
	Serial.print(F("DMP Initialization failed (code "));
	Serial.print(devStatus);
	Serial.println(F(")"));
	return false;

}

// ================================================================
// ===                      WRITE FILES                         ===
// ================================================================

void writeFile(fs::FS &fs, const char * path, const char * message) {
	Serial.printf("Writing file: %s\n", path);

	File file = fs.open(path, FILE_WRITE);
	if (!file) {
		Serial.println("Failed to open file for writing");
		return;
	}
	if (file.print(message)) {
		Serial.println("File written");
	}
	else {
		Serial.println("Write failed");
	}
	file.close();
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

	// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin(); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

	Serial.begin(230400);
	while (!Serial); // wait for Leonardo enumeration, others continue immediately

	pinMode(AD0_PIN_LEFT, OUTPUT);
	pinMode(AD0_PIN_RIGHT, OUTPUT);
	pinMode(INTERRUPT_PIN_LEFT, INPUT);
	pinMode(INTERRUPT_PIN_RIGHT, INPUT);

#ifdef LOGGING
	// Set up logging

	if (!SD.begin()) {
		Serial.println("SD initialization failed!");
		while (1);
	}
	else{
		Serial.println("SD initialization success");
	}
	if (!rtc.begin()) {
		Serial.println("Couldn't find RTC");
		while (1);
	}
	else {
		Serial.println("RTC initialization success");
	}
	// now get the time and prepare a log file.  First, turn off the MPU6050's by moving their I2C addresses to
	// the higher value AD0
	digitalWrite(AD0_PIN_LEFT, HIGH);
	digitalWrite(AD0_PIN_LEFT, HIGH);
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
			Serial.printf("Log file: %s exists. Will try another...\n\r", filepath.c_str());
		}
		else {
			Serial.printf("Log file: %s is available\n\r", filepath.c_str());
			logger_setup(filepath);
			//logfile = SD.open(filepath,FILE_WRITE);
			break;
		}
	}
	// put some initial entries into the file
	msg = "Opening " + filepath + " at " + now.timestamp();
	logger_info(__LINE__, msg);

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
		Serial.println(F("Initializing I2C devices..."));
		// Serial.println(mpu_left.getDeviceID());
		mpu_left.initialize();
		mpu_left.label = "Left ";
		mpu_left.prefix = "L";

		mpu_right.initialize();
		mpu_right.prefix = "R";
		mpu_right.label = "Right ";

		Serial.println(mpu_left.getDeviceID());
		Serial.println(mpu_right.getDeviceID());
		// verify connection
		Serial.println(F("Testing device connections..."));
		Serial.println(mpu_left.testConnection() ? F("MPU6050 left side connection successful") : F("MPU6050 left side connection failed"));
		Serial.println(mpu_right.testConnection() ? F("MPU6050 right side connection successful") : F("MPU6050 right side connection failed"));

		// load and configure the DMP`
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
		boolean const rightDmpReady = initializeDmp(devStatus_right, INTERRUPT_PIN_RIGHT, mpu_right);
		boolean const leftDmpReady = initializeDmp(devStatus_left, INTERRUPT_PIN_LEFT, mpu_left);

		dmpReady = rightDmpReady && leftDmpReady;
	}

	// set data rates void setRate(uint8_t rate);
	uint8_t smplrt_div = (GYRO_RATE / SAMPLE_RATE) - 1;
	mpu_left.setRate(smplrt_div);
	mpu_right.setRate(smplrt_div);

	logger_info(__LINE__, "Initial DMP config ------------");
	logger_info(__LINE__, mpu_left.printEverything());
	logger_info(__LINE__, mpu_right.printEverything());
	//mpu_left.printEverything();
	//mpu_right.printEverything();

	// pinMode(LED_PIN, OUTPUT);

} // end setup()





// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

	// Serial.println("Starting loop");
	if (!dmpReady)
	{
		Serial.println("Something dun fucked up");
		return;
	}

	// wait for MPU interrupt or extra packet(s) available
	TIME_START
	record_time("loop start", millis(), __LINE__, start);
	//TIME_EVENT"loop start"
	
	char* tabs = "R\t\t\t\t\t";
	long loop_start = millis(); // for calculating wait timeout

	//readLeft = false; //TODO remove this.  It forces system to read only one accelerometer
	while (true)
	{
		fifoCount_left = mpu_left.getFIFOCount();
		fifoCount_right = mpu_right.getFIFOCount();
		// Serial.println(readLeft);
		if (fifoCount_right >= 42 && mpuInterruptRight && !readLeft)
		{
			// Data ready on the right
			mpuInterruptRight = false;
			mpu = &mpu_right;
			readLeft = true;
			break;
		}

		if (fifoCount_left >= 42 && mpuInterruptLeft && readLeft)
		{
			// Data ready on the left
			mpuInterruptLeft = false;
			mpu = &mpu_left;
			tabs = "L ";
			readLeft = false;
			break;
		}

		//if(fifoCount_left == 0)
		//{
		//	mpu_left.resetFIFO();
		//	Serial.print("resetting fifo");
		//}
		//Serial.print(".");
#if 0 // skip over this troubleshooting stuff but don't comment it out
		Serial.print((readLeft) ? "L" : "R"); 
		Serial.print(" r/l: ");
		Serial.print(fifoCount_right);
		Serial.print(",");
		Serial.print(fifoCount_left);
		Serial.print(" r/l: ");
		//Serial.print(",");
		
		Serial.print(mpuInterruptRight); 
		Serial.print(",");		
		Serial.print(mpuInterruptLeft);
		Serial.print(",");
		Serial.print((fifoCount_right >= 42 && mpuInterruptRight && !readLeft) ? "GoR" : "Rx ");
		Serial.print(((fifoCount_left >= 42 && mpuInterruptLeft && readLeft)) ? "GoL" : "Lx");
		Serial.println("|");
#endif
		if (millis() - loop_start > WAIT_TIMEOUT_MS) //WAIT_TIMEOUT_MS
		{
			logger_error(__LINE__, "Too long in interrupt wait loop.  Dumping accelerometers ------------");
			logger_error(__LINE__, mpu_left.printEverything());
			logger_error(__LINE__, mpu_right.printEverything());
			loop_start = millis(); //reset loop_start to wait again

			//myFile.println("Right");
			//myFile.println(mpu_right.printEverything());
			//myFile.println("Left");
			//myFile.println(mpu_left.printEverything());
			//Serial.println("R");
			//mpu_right.printEverything();
			//Serial.println("L");
			//mpu_left.printEverything();

			// TODO fix this problem - the following codes is a KLUGE
			logger_error(__LINE__, "Interrupt wait timeout.  Re-enabling DMP's");
			mpu_left.setDMPEnabled(true);
			mpu_right.setDMPEnabled(true);
		}
	}
	mpuIntStatus = mpu->getIntStatus();
	// Serial.println(mpuIntStatus,BIN);

	// get current FIFO count
	fifoCount = mpu->getFIFOCount();
	// Serial.println("Fifo retrieved!");
	// Serial.println(fifoCount);
	if (fifoCount < packetSize)
	{
		Serial.println("Mismatch fifo size!");
		logger_error(__LINE__, ("What? fifoCount is now less than packetSize!"));
		//Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
		// This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
	}
		// check for overflow (this should never happen unless our code is too inefficient)
	else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
	{
		// reset so we can continue cleanly
		// mpu_right.resetFIFO();
		// mpu_left.resetFIFO();

		//Serial.print(tabs);
		//Serial.print(F("FIFO overflow: "));
		//Serial.println(fifoCount);
		String msg = String(mpu->prefix) + " FIFO overflow: " + String(fifoCount) + ((fifoCount > 83)
			                                                                             ? "*************** over"
			                                                                             : "ok");
		logger_error(__LINE__, msg);

		mpu->resetFIFO();
		fifoCount = mpu->getFIFOCount(); // will be zero after reset no need to ask
		//Serial.println(fifoCount);
	}
	else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
	{
		// read a packet from FIFO
		String msg = "fetching FIFO: " + String(fifoCount);
		record_time(msg, millis(), __LINE__, start);
		while (fifoCount >= packetSize)
		{
			// Lets catch up to NOW, someone is using the dreaded delay()!
			mpu->getFIFOBytes(fifoBuffer, packetSize);
			// track FIFO count here in case there is > 1 packet available
			// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= packetSize;
			// Serial.println(F("fifoBuffer loaded"));
		}
		record_time("packets fetched", millis(), __LINE__, start);

#ifdef RECORD_READABLE_QUATERNION
		// write quaternion values in easy matrix form: w x y z
		mpu->dmpGetQuaternion(&q, fifoBuffer);
		if ((q.w * q.w + q.y * q.y + q.x * q.x + q.z * q.z) > 1.05)
		{
			logger_error(__LINE__, sardprintf("%s quaternion out of range.  Resetting FIFO", mpu->prefix));
			logger_error(__LINE__, sardprintf("%s,%f,%f,%f,%f", mpu->prefix, q.w, q.x, q.y, q.z));
			mpu->resetFIFO();
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
		// blink LED to indicate activity
	}
}
