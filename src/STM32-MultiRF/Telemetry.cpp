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
    mUpdateMask = 0;
    memset(&mVolt, 0, sizeof(mVolt));
    memset(&mTemp, 0, sizeof(mTemp));
    memset(&mRPM, 0, sizeof(mRPM));
    mRSSI = 0;
    mBattCap = 0;
    memset(&mGPS, 0, sizeof(mGPS));
}

Telemetry::~Telemetry()
{

}


/*
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
*/


//0[00] 18(0x12)
//1[01] 00
//2[02] Altitude MSB (Hex)
//3[03] Altitude LSB (Hex) 16bit signed integer, in 0.1m
//4[04] Max Altitude MSB (Hex)
//5[05] Max Altitude LSB (Hex) 16bit signed integer, in 0.1m
//6[05] Unknown
//7[07] Unknown
//8[08] Unknown
//9[09] Unknown
//10[0A] Unknown
//11[0B] Unknown
//12[0C] Unknown
//13[0D] Unknown
//14[0E] Unknown
//15[0F] Unknown
u8 Telemetry::buildAltInfo(u8 *buf)
{
    memset(buf, 0, 18);
    buf[0] = 0x12;
    buf[1] = 0x00;
    buf[2] = (mBaroAlt >> 8) & 0xff;
    buf[3] = mBaroAlt & 0xff;
    buf[4] = 0x00;
    buf[5] = 0x96;

    return 18;
}

//0 [00] 0x0A
//1 [01] 00
//2 [02] V1 MSB (Hex)
//3 [03] V1 LSB (Hex) //In 0.01V
//4 [04] V2 MSB (Hex)
//5 [05] V2 LSB (Hex) //In 0.01V
//6 [06] Cap1 MSB (Hex)
//7 [07] Cap1 LSB (Hex) //In 1mAh
//8 [08] Cap2 MSB (Hex)
//9 [09] Cap2 LSB (Hex) //In 1mAh
//10 [0A] 00
//11 [0B] 00
//12 [0C] 00
//13 [0D] 00
//14 [0E] 00
//15 [0F] Alarm // The fist bit is alarm V1, the second V2, the third Capacity 1, the 4th capacity 2.
u8 Telemetry::buildPowerInfo(u8 *buf)
{
    u8  idx = 0;

    if (getVolt(idx) == 0) {
        idx++;
    }
    memset(buf, 0, 18);
    buf[0] = 0x0A;
    buf[1] = 0x00;
    buf[2] = (getVolt(idx) >> 8) & 0xff;  // 0.01V
    buf[3] = getVolt(idx) & 0xff;
    buf[4] = (getVolt(idx + 1) >> 8) & 0xff;
    buf[5] = getVolt(idx + 1) & 0xff;
    buf[6] = 0x08;                      // cap 1mAh, ex:2200mAh
    buf[7] = 0x98;
    buf[8] = 0x08;
    buf[9] = 0x98;

    return 18;
}


//0[00] 22(0x16)
//1[01] 00
//2[02] Altitude LSB (Decimal) //In 0.1m
//3[03] Altitude MSB (Decimal)
//4[04] 1/100th of a degree second latitude (Decimal) (XX YY.SSSS)
//5[05] degree seconds latitude (Decimal)
//6[06] degree minutes latitude (Decimal)
//7[07] degrees latitude (Decimal)
//8[08] 1/100th of a degree second longitude (Decimal) (XX YY.SSSS)
//9[09] degree seconds longitude (Decimal)
//10[0A] degree minutes longitude (Decimal)
//11[0B] degrees longitude (Decimal)
//12[0C] Heading LSB (Decimal)
//13[0D] Heading MSB (Decimal) Divide by 10 for Degrees
//14[0E] Unknown
//15[0F] First bit for latitude: 1=N(+), 0=S(-);
//Second bit for longitude: 1=E(+), 0=W(-);
//Third bit for longitude over 99 degrees: 1=+-100 degrees
u8 Telemetry::buildGPSInfo(u8 *buf)
{
    memset(buf, 0, 18);
    buf[0] = 0x16;
    buf[1] = 0x00;
    buf[2] = (getGPS()->altitude >> 8) & 0xff;
    buf[3] = getGPS()->altitude & 0xff;

    buf[7] =  getGPS()->latitude / 3600000;     // hour
    getGPS()->latitude -= (buf[7] * 3600000);
    buf[6] =  getGPS()->latitude / 60000;       // min
    getGPS()->latitude -= (buf[6] * 60000);
    buf[5] =  getGPS()->latitude / 6;           // sec
    getGPS()->latitude -= (buf[5] * 6);
    buf[4] =  getGPS()->latitude;               // 1/100

    buf[11] =  getGPS()->longitude / 3600000;   // hour
    getGPS()->longitude -= (buf[7] * 3600000);
    buf[10] =  getGPS()->longitude / 60000;     // min
    getGPS()->longitude -= (buf[6] * 60000);
    buf[9] =  getGPS()->longitude / 6;          // sec
    getGPS()->longitude -= (buf[5] * 6);
    buf[8] =  getGPS()->longitude;              // 1/100

    buf[12] = getGPS()->heading & 0xff;
    buf[13] = (getGPS()->heading >> 8) & 0xff;

    buf[15] = 0xC0;

    return 18;
}


//0[00] 7E or FE
//1[01] 00
//2[02] RPM MSB (Hex)
//3[03] RPM LSB (Hex) //RPM = 120000000 / number_of_poles(2, 4, ... 32) / gear_ratio(0.01 - 30.99) / Value
//4[04] Volt MSB (Hex)
//5[05] Volt LSB (Hex) //In 0.01V
//6[06] Temp MSB (Hex)
//7[07] Temp LSB (Hex) //Value (Decimal) is in Fahrenheit, for Celsius (Value (Decimal) - 32) * 5) / 9)
//8[08] Unknown
//9[09] Unknown
//10[0A] Unknown
//11[0B] Unknown
//12[0C] Unknown
//13[0D] Unknown	// Assan when 7E type is Rx RSSI
//14[0E] Unknown
//15[0F] Unknown
u8 Telemetry::buildTMInfo(u8 *buf)
{
    u8  idx = 0;

    if (getVolt(idx) == 0) {
        idx++;
    }

    memset(buf, 0, 18);
    buf[0] = 0x7E;
    buf[1] = 0x00;
    buf[2] = (getRPM(0) >> 8) & 0xff;
    buf[3] = getRPM(0)  & 0xff;
    buf[4] = (getVolt(idx) >> 8) & 0xff;  // 0.01V
    buf[5] = getVolt(idx) & 0xff;
    buf[6] = (getTemp(idx) >> 8) & 0xff;
    buf[7] = getTemp(idx) & 0xff;

    return 18;
}


void Telemetry::frameDSM(u8 rssi, u8 *buf, u8 size)
{
#if 0
    Serial3.write(0xAA);
    Serial3.write(rssi);
    for (u8 i = 0; i < size; i++)
        Serial3.write(buf[i]);
#endif
}

void Telemetry::update(void)
{
    u8 size;
    u8 rssi;

    if (isMasked(MASK_RSSI)) {
        rssi = constrain(getRSSI(), 0, 0x1f);
    } else {
        rssi = 0x1f;
    }

    if (isMasked(MASK_VOLT)) {
//        LOG(F("VOLT - V1:%d, V2:%d, V3:%d\n"), mVolt[0], mVolt[1], mVolt[2]);
        size = buildPowerInfo(mTeleBuf);
        frameDSM(rssi, mTeleBuf, size);
        clearMask(MASK_VOLT);
    }

    if (isMasked(MASK_TEMP | MASK_RPM)) {
//        LOG(F("RPM  - R1:%d, R2:%d, R3:%d\n"), mRPM[0], mRPM[1], mRPM[2]);
//        LOG(F("TEMP - T1:%d, T2:%d, T3:%d, T4:%d\n"), mTemp[0], mTemp[1], mTemp[2], mTemp[3]);
        size = buildTMInfo(mTeleBuf);
        frameDSM(rssi, mTeleBuf, size);
        clearMask(MASK_TEMP | MASK_RPM);
    }

    if (isMasked(MASK_BARO_ALT)) {
        size = buildAltInfo(mTeleBuf);
        frameDSM(rssi, mTeleBuf, size);
        clearMask(MASK_BARO_ALT);
    }

    if (isMasked(MASK_VELOCITY)) {

    }

    if (isMasked(MASK_GPS)) {
        size = buildGPSInfo(mTeleBuf);
        frameDSM(rssi, mTeleBuf, size);
        clearMask(MASK_GPS);
    }
}

u8 Telemetry::handleTX(u8 *data)
{
    return 0;
}

