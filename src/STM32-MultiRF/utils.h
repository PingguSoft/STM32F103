/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is derived from deviationTx project for Arduino.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 see <http://www.gnu.org/licenses/>
*/

#ifndef _UTILS_H_
#define _UTILS_H_
#include "Common.h"

#undef  F
#define F(str)  str

// Bit vector from bit position
#define BV(bit) (1 << (bit))

u32  rand32_r(u32 *seed, u8 update);
u32  rand32();


template <typename T> void PROGMEM_read(const T * sce, T& dest)
{
    memcpy_P(&dest, sce, sizeof (T));
}

template <typename T> T PROGMEM_get(const T * sce)
{
    static T temp;
    memcpy_P(&temp, sce, sizeof (T));
    return temp;
}


#if __DEBUG__
    void LOG(char *fmt, ... );
    void DUMP(char *name, u8 *data, u16 cnt);
    #define __PRINT_FUNC__  //LOG(F("%08ld : %s\n"), millis(), __PRETTY_FUNCTION__);
#else
    #define LOG(...)
    #define DUMP(...)
    #define __PRINT_FUNC__
#endif

#endif
