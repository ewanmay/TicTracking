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
enum Log_Level: char { ignore ,data, debug, info, error,critical};
File logger_setup(String filepath);
bool logger_data(int line_no, const String msg);
bool logger_debug(int line_no, const String msg);
bool logger_info(int line_no, const String msg);
bool logger_error(int line_no, const String msg);;
bool set_logger_filter(Log_Level exclude[], int sizeof_exclude, int heartbeat_exclude);
//bool logger_critical(File logfile, long milli, const String*  msg);
bool write_log_record(int line_no, String msg, Log_Level log_lvl);
bool write_record(String log_record, Log_Level log_lvl);

String sardprintf(char *str, ...);

//int log_lvl;

struct Log_Message
{
	char* message;
	char* level;
	long millis;	
} ;


#endif

