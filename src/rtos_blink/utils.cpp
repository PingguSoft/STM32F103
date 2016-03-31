/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 see <http://www.gnu.org/licenses/>
*/

#include <stdarg.h>
#include "config.h"
#include "common.h"
#include "utils.h"

#ifdef CONFIG_DBG_SERIAL
void Utils::printf(char *fmt, ... )
{
    char buf[100];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    CONFIG_DBG_SERIAL.print(buf);
}

void Utils::printf(const __FlashStringHelper *fmt, ... )
{
    char buf[100];
    va_list args;

    va_start (args, fmt);
#ifdef __AVR__
    vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args); // progmem for AVR
#else
    vsnprintf(buf, sizeof(buf), (const char *)fmt, args); // for the rest of the world
#endif
    va_end(args);
    CONFIG_DBG_SERIAL.print(buf);
}
#else
void Utils::printf(char *fmt, ... )
{
}
void Utils::printf(const __FlashStringHelper *fmt, ... )
{
}
#endif
