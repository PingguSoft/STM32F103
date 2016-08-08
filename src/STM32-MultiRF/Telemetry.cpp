/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Deviation is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Deviation.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "Common.h"
#include "Telemetry.h"
#include "utils.h"

Telemetry::Telemetry()
{
//    Serial3.begin(115200);
}

Telemetry::~Telemetry()
{
//    Serial3.end();
}

u8 Telemetry::buildAltInfo(u8 *buf)
{
    memset(buf, 0, 18);
    buf[0] = 0x12;
    buf[1] = 0x00;
    buf[2] = (getGPS()->altitude >> 8) & 0xff;
    buf[3] = getGPS()->altitude & 0xff;
    buf[4] = 0x00;
    buf[5] = 0x12;

    return 18;
}

u8 Telemetry::buildPowerInfo(u8 *buf)
{
    memset(buf, 0, 18);
    buf[0] = 0x0A;
    buf[1] = 0x00;
    buf[2] = (getVolt(0) >> 8) & 0xff;  // 0.01V
    buf[3] = getVolt(0) & 0xff;
    buf[4] = (getVolt(1) >> 8) & 0xff;
    buf[5] = getVolt(1) & 0xff;
    buf[6] = 0x08;                      // cap 1mAh, ex:2200mAh
    buf[7] = 0x98;
    buf[8] = 0x08;
    buf[9] = 0x98;

    return 18;
}

u8 Telemetry::buildRSSI(u8 *buf)
{
    memset(buf, 0, 18);
    buf[0] = 0x0A;
    buf[1] = 0x00;
    buf[2] = (getVolt(0) >> 8) & 0xff;  // 0.01V
    buf[3] = getVolt(0) & 0xff;
    buf[4] = (getVolt(1) >> 8) & 0xff;
    buf[5] = getVolt(1) & 0xff;
    buf[6] = 0x00;                      // cap 1mAh
    buf[7] = 0x12;
    buf[8] = 0x00;
    buf[9] = 0x12;

    return 18;
}

#define FRSKY_START_STOP    0x7e
#define FRSKY_BYTESTUFF     0x7d
#define FRSKY_STUFF_MASK    0x20

void Telemetry::frameFRSky(u8 *buf, u8 size)
{
    Serial3.write(FRSKY_START_STOP);
    for (u8 i = 0; i < size; i++) {
        if (buf[i] == FRSKY_START_STOP || buf[i] == FRSKY_BYTESTUFF) {
            Serial3.write(FRSKY_BYTESTUFF);
            buf[i] ^= FRSKY_BYTESTUFF;
        }
        Serial3.write(buf[i]);
    }
    Serial3.write(FRSKY_START_STOP);
}

void Telemetry::frameDSM(u8 type, u8 *buf, u8 size)
{
    Serial3.write(0xAA);
    Serial3.write(type);
    for (u8 i = 0; i < size; i++)
        Serial3.write(buf[i]);
}

int step = 0;

void Telemetry::update(void)
{
    u8 size;

    switch ((step++) % 2) {
        case 0:
            size = buildAltInfo(mBuffer);
            break;

        case 1:
            size = buildPowerInfo(mBuffer);
            break;
    }

    frameDSM(0x01, mBuffer, size);
}
