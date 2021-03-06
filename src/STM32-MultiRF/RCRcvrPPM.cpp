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

#include <SPI.h>

#include "common.h"
#include "utils.h"
#include "RCRcvrPPM.h"

#define PIN_PPM     PB11

static void calcPPM();

u8 RCRcvrPPM::getChCnt(void)
{
    return CH_CNT;
}

void RCRcvrPPM::init(void)
{
    for (u8 i = 0; i < sizeof(sRC); i++)
        sRC[i] = CHAN_MID_VALUE;

    sRC[RFProtocol::CH_THROTTLE] = CHAN_MIN_VALUE;

    pinMode(PIN_PPM, INPUT);
    attachInterrupt(PIN_PPM, calcPPM, RISING);
}

void RCRcvrPPM::close(void)
{
    detachInterrupt(PIN_PPM);
}

static void calcPPM()
{
    static u8   ch;
    static u32  lastTS;

    u32 ts = micros();
    u32 diff = ts - lastTS;

    if (diff > 2500) {
        ch = 0;
    } else {
        if (ch < CH_CNT) {
            u16 val = constrain(diff, PPM_MIN_VALUE, PPM_MAX_VALUE);
            RCRcvr::sRC[ch++] = map(val, PPM_MIN_VALUE, PPM_MAX_VALUE, CHAN_MIN_VALUE, CHAN_MAX_VALUE);
        }
    }
    lastTS = ts;
}

