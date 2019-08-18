// 
// 
// 

#include "Logging.h"
//#include <SD.h>

File Logfile;

Log_Level exclude_levels[10] = {ignore}; // list of excluded levels.  Will initialize totally to ignore
unsigned int max_heartbeat_excludes = 10;
// output a character (most likely a period) after this number of excluded log entries
unsigned int heartbeat_excludes = 0; // current excludes count
bool exclusion_on = false; // if true then we are in the exclusion zone
//bool exclude_now = false; // if true then we are in the exclusion zone


File logger_setup(String filepath)
{
	Logfile = SD.open(filepath, FILE_WRITE);
	return Logfile; // will evaluate to false if failure
}


bool logger_data(int line_no, const String msg)
{
	const auto log_lvl = data;
	return bool(write_log_record(line_no, msg, log_lvl));
}

bool logger_debug(int line_no, const String msg)
{
	const auto log_lvl = debug;
	return bool(write_log_record(line_no, msg, log_lvl));
}

bool logger_info(int line_no, const String msg)
{
	const auto log_lvl = info;
	return bool(write_log_record(line_no, msg, log_lvl));
}

bool logger_error(int line_no, const String msg)
{
	const auto log_lvl = error;
	return bool(write_log_record(line_no, msg, log_lvl));
}

//bool logger_critical(File logfile, long milli, const String* msg)
//{
//	int log_lvl = 40;
//	return bool(write_log_record(logfile, milli, msg, log_lvl));
//}
bool write_log_record(int line_no, String msg, Log_Level log_lvl)
{
	String debug_string;
	switch (log_lvl)
	{
	case data:
		debug_string = "DATA";
		break;
	case debug:
		debug_string = "DEBUG";
		break;
	case info:
		debug_string = "INFO";
		break;
	case error:
		debug_string = "ERROR";
		break;
	case critical:
		debug_string = "CRITICAL";
		break;
	default:
		debug_string = "unk";
		break;
	}
	String log_record = String(line_no) + "\t" + String(millis()) + "\t" + debug_string + "\t" + msg;
	return write_record(log_record, log_lvl);
}


bool write_record(String log_record, Log_Level log_lvl)
// write to Logfile and return a simple boolean verification
{
	//Serial.print("Log:\t");

	// TODO figure out what to do when ndef LOGGING.  Turn all off or just no file write?
	//Serial.print(" to ");
	//Serial.println(Logfile.name());
	//char charbuf[log_record.length()];
	//log_record.toCharArray(charbuf, log_record.length());
	unsigned int good_write = (unsigned int)Logfile.println(log_record);
	//Serial.printf("SD Write length: %d", i);
	//logger_debug(__LINE__, sardprintf("SD Write length result: %d", i));
	Logfile.flush();
	// now filter based on exclude
	bool exclude_now = false;
	for (unsigned int i = 0; i <= sizeof(exclude_levels) / sizeof(exclude_levels[0]); i++)
	{
		//Serial.printf("Log level: %d  Exclude: %d\n", (int)log_lvl, (int)exclude_levels[i]);
		if (log_lvl == exclude_levels[i])
		{
			exclude_now = true;
			break;
		}
	}
	if (exclude_now)
	{
		//Serial.printf("Heartbeats: %d max: %d", heartbeat_excludes ,max_heartbeat_excludes);
		if (!exclusion_on)
		{
			// a first excluded entry: output a character and start counting
			//Serial.printf("Exclusion starts");
			exclusion_on = true;
			heartbeat_excludes = 1;
			Serial.print(".");
		}
		else if (heartbeat_excludes <= max_heartbeat_excludes)
		{
			// an excluded entry to be counted but with no output
			//Serial.printf("Heartbeat increment: %d", heartbeat_excludes);
			heartbeat_excludes++;
		}
		else
		{
			// reached maximum excluded entries: output a character and restart counting
			//Serial.printf("Exclusion rolls over");
			Serial.print(".");
			heartbeat_excludes = 1;
		}
	}
	else // print this log entry
	{
		// if coming off exclusion, end the characters and start on a new line
		if (exclusion_on)
		{
			//Serial.printf("Exclusion ends"); 
			Serial.println();
			exclusion_on = false;
		}
		// print an asterisk if we can detect a write problem
		if (good_write < log_record.length()) Serial.printf("%d/%d", good_write, log_record.length());
		//TODO figure out why these two numbers do not equate
		//Serial.println(log_record.length());
		Serial.println(log_record);
	}
	return (good_write > log_record.length());
}

bool set_logger_filter(Log_Level exclude[], int sizeof_exclude, int heartbeat_exclude)
{
	max_heartbeat_excludes = heartbeat_exclude;
	// output a character (most likely a period) after this number of excluded log entries
	// try to make this additive rather that pure initialization
	//Serial.printf("Logging filter. Size of exclude: %d Size of exclude_levels: %d \n", sizeof_exclude, sizeof(exclude_levels) / sizeof(exclude_levels[0]));
	//int exclude_this = 0; //pointer into exclude[]
	for (unsigned int i = 0, exclude_this = 0; i < (sizeof(exclude_levels) / sizeof(exclude_levels[0])); i++)
	{
		//Serial.printf("i: %d exclude_this; %d Exclude: %d exclude_levels: %d \n", i, exclude_this, exclude[exclude_this], exclude_levels[i]); 
		if (exclude_this >= sizeof_exclude) break; // break if we have exceeded the number of exclusions

		if (exclude_levels[i] != ignore) continue; // put the exclusion in the array if it's unoccupied
		exclude_levels[i] = exclude[exclude_this];
		exclude_this++; // point to the next exclusion
	}
}
#ifndef SARDPRINTF
#define SARDPRINTF
#define ARDBUFFER 255
#include <stdarg.h>
#include <Arduino.h>

String sardprintf(char* str, ...)
{
	//Serial.printf("\n%s   ", str);
	int i, count = 0, j = 0, flag = 0;
	char temp[ARDBUFFER + 1];
	String msg = "";
	for (i = 0; str[i] != '\0'; i++) if (str[i] == '%') count++;
	//Serial.printf("%d arguments \n", count);

	va_list argv;
	va_start(argv, count);
	for (i = 0, j = 0; str[i] != '\0'; i++)
	{
		if (str[i] == '%')
		{
			temp[j] = '\0';
			msg.concat(temp);
			//Serial.printf("adding temp= %s \n", temp);
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
			default: ;
			
			};
			//Serial.printf("adding arg, msg= %s \n", msg.c_str());
		}
		else
		{
			temp[j] = str[i];
			j = (j + 1) % ARDBUFFER;
			if (j == 0) //buffer is full so add it and keep going
			{
				temp[ARDBUFFER] = '\0';
				msg.concat(temp);
				//Serial.printf(" ovflow msg now :%s \n", msg.c_str());
				temp[0] = '\0';
			}
		}
	};
	temp[j] = '\0'; // add any remaining non-argument characters left in temp
	//Serial.printf("%d characters left in j: %s \n", j, temp);
	msg.concat(temp);
	return msg;
}
#undef ARDBUFFER
#endif
