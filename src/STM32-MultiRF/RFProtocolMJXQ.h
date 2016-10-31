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

#ifndef _PROTOCOL_MJQX_H_
#define _PROTOCOL_MJQX_H_

#include "DeviceNRF24L01.h"
#include "RFProtocol.h"

class RFProtocolMJXQ : public RFProtocol
{
#define MAX_PACKET_SIZE     16
#define ADDR_BUF_SIZE       5
#define MAX_RF_CHANNELS     4
#define MAX_TX_ID_SIZE      3

public:
    RFProtocolMJXQ(u32 id):RFProtocol(id) { }
    ~RFProtocolMJXQ() { close(); }

// for protocol
    virtual int  init(u8 bind);
    virtual int  close(void);
    virtual int  reset(void);
    virtual int  getInfo(s8 id, u8 *data);
    virtual u16  callState(u32 now, u32 expected);

private:
    u8   calcCheckSum(void);
    u8   calcPanTilt(void);
    void sendPacket(u8 bind);
    void init1(void);
    void init2(void);
    void initTxID(void);
    u8   convChannel(u8 num);

// variables
    DeviceNRF24L01  mDev;
    u32  mPacketCtr;
    u16  mBindCtr;
    u8   mTxID[MAX_TX_ID_SIZE];
    u8   mRFChanBufs[MAX_RF_CHANNELS];
    u8   mPacketBuf[MAX_PACKET_SIZE];
    u8   mRxTxAddrBuf[ADDR_BUF_SIZE];

    u8   mCurRFChan;
    u8   mRFChanCnt;
    u8   mPacketSize;
    u8   mState;

protected:

};

#endif
