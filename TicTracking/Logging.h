// Logging.h

#ifndef _LOGGING_h
#define _LOGGING_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include <arduino.h>
#else
	#include "WProgram.h"
#endif

#include <SD.h>


// prototypes
File logger_setup(String filepath);
bool logger_data(int line_no, const String msg);
//bool logger_debug(File logfile, long milli, const String*  msg);
bool logger_info(int line_no, const String msg);
bool logger_error(int line_no, const String msg);;
//bool logger_critical(File logfile, long milli, const String*  msg);
bool write_log_record(int line_no, String msg, int log_lvl);
bool write_record(const String log_record);

String sardprintf(char *str, ...);

//int log_lvl;

struct Log_Message
{
	char* message;
	char* level;
	long millis;	
} ;

#endif

