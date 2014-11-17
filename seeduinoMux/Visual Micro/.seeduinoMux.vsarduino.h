#ifndef _VSARDUINO_H_
#define _VSARDUINO_H_
//Board = Arduino Mega (ATmega1280)
#define __AVR_ATmega1280__
#define ARDUINO 101
#define __AVR__
#define F_CPU 16000000L
#define __cplusplus
#define __attribute__(x)
#define __inline__
#define __asm__(x)
#define __extension__
#define __ATTR_PURE__
#define __ATTR_CONST__
#define __inline__
#define __asm__ 
#define __volatile__
#define __builtin_va_list
#define __builtin_va_start
#define __builtin_va_end
#define __DOXYGEN__
#define prog_void
#define PGM_VOID_P int
#define NOINLINE __attribute__((noinline))

typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {;}

bool slvPort(char* label, HardwareSerial* port);
bool slvGetAPpacket();
bool slvGetGCSpacket();
bool slvGetBEAGLEpacket();
//
//

#include "C:\Program Files (x86)\Arduino\hardware\arduino\cores\arduino\wprogram.h"
#include "G:\ArduRepo\seeduinoMux\seeduinoMux.pde"
#endif
