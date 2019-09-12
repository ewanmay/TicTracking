// system_state.h

#ifndef _SYSTEM_STATE_h
#define _SYSTEM_STATE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
enum state {none, going_off, off, begin, startup, startup_stopped, waiting, calibrate, calibrate_stopped, recording, recording_stopped, paused};
struct system_state
{
	state previous;
	state current;
	state next;
};

#endif

