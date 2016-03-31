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

#ifndef _UTILS_H_
#define _UTILS_H_
#include <Arduino.h>
#include "config.h"

// Bit vector from bit position
#define BV(bit) (1 << (bit))

class Utils {
private:
public:
    static void printf(char *fmt, ... );
    static void printf(const __FlashStringHelper *fmt, ... );
};

#endif
