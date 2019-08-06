// 
// 
// 

#include "Logging.h"
//#include <SD.h>

File Logfile;


File logger_setup(String filepath)
{
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
bool logger_info(int line_no, const String msg)
{
	int log_lvl = 20;
	return bool(write_log_record(line_no, msg, log_lvl));
}
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
	String log_record = String(line_no) + "\t" + String(millis()) + "\t" + debug_string + "\t" + msg;
	return write_record(log_record);
}


bool write_record(String log_record)
// write to logfile and return a simple boolean verification
{
	Serial.print("Log:\t");
	Serial.println(log_record);
	//Serial.print(" to ");
	//Serial.println(Logfile.name());
	//char charbuf[log_record.length()];
	//log_record.toCharArray(charbuf, log_record.length());
	int i = Logfile.println(log_record);
	Logfile.flush();
	return  (i == log_record.length());
}

#ifndef SARDPRINTF
#define SARDPRINTF
#define ARDBUFFER 16
#include <stdarg.h>
#include <Arduino.h>

String sardprintf(char *str, ...)
{
	int i, count = 0, j = 0, flag = 0;
	char temp[ARDBUFFER + 1];
	String msg = "";
	for (i = 0; str[i] != '\0'; i++)  if (str[i] == '%')  count++;

	va_list argv;
	va_start(argv, count);
	for (i = 0, j = 0; str[i] != '\0'; i++)
	{
		if (str[i] == '%')
		{
			temp[j] = '\0';
			msg.concat(temp);
			j = 0;
			temp[0] = '\0';

			switch (str[++i])
			{
			case 'd': msg.concat(va_arg(argv, int));
				break;
			case 'l': msg.concat(va_arg(argv, long));
				break;
			case 'f': msg.concat(va_arg(argv, double));
				break;
			case 'c': msg.concat((char)va_arg(argv, int));
				break;
			case 's': msg.concat(va_arg(argv, char *));
				break;
			default:;
			};
		}
		else
		{
			temp[j] = str[i];
			j = (j + 1) % ARDBUFFER;
			if (j == 0)
			{
				temp[ARDBUFFER] = '\0';
				msg.concat(temp);
				temp[0] = '\0';
			}
		}
	};
	msg.concat("\n\r");
	return msg;
}
#undef ARDBUFFER
#endif