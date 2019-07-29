// 
// 
// 

#include "Logging.h"
#include <SD.h>

File Logfile;

File logger_setup(char* filepath, int CS)
{
	//  set up logging to an SD card

	if (!SD.begin(CS)) {
		Serial.println("SD initialization failed!");
		// pass the error message back in the filename
		return SD.open("init failed", FILE_READ);
	}
	Logfile = SD.open(filepath, FILE_WRITE);
	return  Logfile; // will evaluate to false if failure

}


bool logger_data( int line_no, const String msg)
{
	int log_lvl = 5;
	return bool(write_log_record(line_no,  msg, log_lvl));
}
//bool logger_debug(File logfile, long milli, const String* msg)
//{
//	int log_lvl = 10;
//	return bool(write_log_record(logfile, milli, msg, log_lvl));
//}
//bool logger_info(File logfile, long milli, const String* msg)
//{
//	int log_lvl = 20;
//	return bool(write_log_record(logfile, milli, msg, log_lvl));
//}
bool logger_error(int line_no, const String msg)
{
	int log_lvl = 30;
	return bool(write_log_record(line_no, msg, log_lvl));
}
//bool logger_critical(File logfile, long milli, const String* msg)
//{
//	int log_lvl = 40;
//	return bool(write_log_record(logfile, milli, msg, log_lvl));
//}
bool write_log_record(int line_no, String msg, int log_lvl)
{
	String debug_string;
	switch (log_lvl)
	{
	case 5:
		debug_string = "DATA";
		break;
	case 10:
		debug_string = "DEBUG";
		break;
	case 20:
		debug_string = "INFO";
		break;
	case 30:
		debug_string = "ERROR";
		break;
	case 40:
		debug_string = "CRITICAL";
		break;
	default:
		debug_string = "unk";
		break;

	}
	const String log_record = String(line_no) + "\t" + String(millis()) + "\t" + debug_string + "\t" + msg;
	return write_record(log_record);
}


bool write_record(const String log_record)
// write to logfile and return a simple boolean verification
{
	Serial.print("Logrecord: ");
	Serial.println(log_record);
		char charbuf[log_record.length()];
	log_record.toCharArray(charbuf, log_record.length());
	int i = Logfile.println(charbuf);
	Logfile.flush();
	return  (i == log_record.length());
}

