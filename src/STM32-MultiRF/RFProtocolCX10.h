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

#ifndef _PROTOCOL_CX10_H_
#define _PROTOCOL_CX10_H_

#include "DeviceNRF24L01.h"
#include "RFProtocol.h"

class RFProtocolCX10 : public RFProtocol
{
#define MAX_PACKET_SIZE     21
#define ADDR_BUF_SIZE       5
#define MAX_RF_CHANNELS     4
#define MAX_TX_ID_SIZE      4

public:
    RFProtocolCX10(u32 id):RFProtocol(id) { }
    ~RFProtocolCX10() { close(); }

// for protocol
    virtual int  init(u8 bind);
    virtual int  close(void);
    virtual int  reset(void);
    virtual int  getInfo(s8 id, u8 *data);
    virtual u16  callState(u32 now, u32 expected);

private:
    void init1(void);
    void sendPacket(u8 bind);
    void initTxID(void);
    u16  convChannel(u8 num);
    u8   getVideoState(u8 channel, u8 video_state);

// variables
    DeviceNRF24L01  mDev;
    u32  mPacketCtr;
    u16  mBindCtr;
    u16  mPacketPeriod;
    u8   mRFChanBufs[MAX_RF_CHANNELS];
    u8   mPacketBuf[MAX_PACKET_SIZE];
    u8   mTxID[MAX_TX_ID_SIZE];
    u8   mRxTxAddrBuf[ADDR_BUF_SIZE];

    u8   mCurRFChan;
    u8   mRFChanCnt;
    u8   mPacketSize;
    u8   mState;
    u8   mBindState;

protected:

};

#endif
