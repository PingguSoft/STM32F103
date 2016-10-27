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

#ifndef _RCVR_SERIAL_H_
#define _RCVR_SERIAL_H_
#include "Common.h"
#include "RCRcvr.h"

#define MAX_PACKET_SIZE 26

#define FUNC_AUTO_BIND  BV(0)
#define FUNC_BIND       BV(1)
#define FUNC_RANGE      BV(2)
#define FUNC_POWER_HI   BV(3)

class RCRcvrERSkySerial : public RCRcvr
{
public:
    virtual void init(void);
    virtual void close(void);
    virtual u8   getChCnt(void);
    virtual u32  loop(void);
    virtual void handleRX(u8 data);

private:
    u32 handlePacket(u8 *data, u8 size);

    // variables
    u8   mRxPacket[MAX_PACKET_SIZE];

    u8   mState;
    u8   mOffset;
    u8   mDataSize;

    u8   mProto;
    u8   mSubProto;
    u8   mFunc;

    u8   mFinalProto;
    u8   mFinalSubProto;
    u8   mFinalFunc;
    u8   mProtoChCnt;
};

#endif
