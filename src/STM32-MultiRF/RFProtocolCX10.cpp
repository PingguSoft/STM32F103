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
#include "RFProtocolCX10.h"
#include "utils.h"

#define BIND_COUNT          4360    // 6 seconds
#define CX10_PACKET_SIZE	15
#define CX10A_PACKET_SIZE	19      // CX10 blue board packets have 19-byte payload
#define Q282_PACKET_SIZE	21
#define CX10_PACKET_PERIOD	1316    // Timeout for callback in uSec
#define CX10A_PACKET_PERIOD	6000
#define INITIAL_WAIT        500

// flags
#define FLAG_FLIP       0x1000      // goes to rudder channel
#define FLAG_MODE_MASK  0x0003
#define FLAG_HEADLESS   0x0004
// flags2
#define FLAG_VIDEO      0x0002
#define FLAG_SNAPSHOT   0x0004

// frequency channel management
#define RF_BIND_CHANNEL 0x02

enum {
    FORMAT_CX10_GREEN = 0,
    FORMAT_CX10_BLUE,
    FORMAT_DM007,
    FORMAT_Q282,
    FORMAT_JC3015_1,
    FORMAT_JC3015_2,
    FORMAT_MK33041,
    FORMAT_Q242,
};

enum {
    CX10_INIT1 = 0,
    CX10_BIND1,
    CX10_BIND2,
    CX10_DATA
};

// Channel values are servo time in ms, 1500ms is the middle,
// 1000 and 2000 are min and max values
u16 RFProtocolCX10::convChannel(u8 num)
{
    s32 ch = getControl(num);
    if (ch < CHAN_MIN_VALUE) {
        ch = CHAN_MIN_VALUE;
    } else if (ch > CHAN_MAX_VALUE) {
        ch = CHAN_MAX_VALUE;
    }
    return (u16) ((ch * 500 / CHAN_MAX_VALUE) + 1500);
}

u8 RFProtocolCX10::getVideoState(u8 channel, u8 video_state)
{
    if (getControl(channel) > 0) {
        if (!(video_state & 0x20)) return video_state ^ 0x21;
    } else {
        if (video_state & 0x20) return video_state & 0x01;
    }
    return video_state;
}

#define CHANNEL_LED         CH_AUX1
#define CHANNEL_FLIP        CH_AUX2
#define CHANNEL_PICTURE     CH_AUX3
#define CHANNEL_VIDEO       CH_AUX4
#define CHANNEL_HEADLESS    CH_AUX5
#define CHANNEL_RTH         CH_AUX6
#define CHANNEL_XCAL        CH_AUX7
#define CHANNEL_YCAL        CH_AUX8
#define GET_FLAG(ch, mask) (RFProtocol::getControl(ch) > 0 ? mask : 0)

void RFProtocolCX10::sendPacket(u8 bind)
{
    u16 aileron  = convChannel(CH_AILERON);
    u16 elevator = 3000 - convChannel(CH_ELEVATOR);
    u16 throttle = convChannel(CH_THROTTLE);
    u16 rudder   = 3000 - convChannel(CH_RUDDER);

    switch(getProtocolOpt()) {
        case FORMAT_DM007:
            aileron = 3000 - aileron;
            break;

        case FORMAT_Q282:
        case FORMAT_Q242:
            aileron = 3000 - aileron;
            rudder = 3000 - rudder;
            break;

        case FORMAT_JC3015_1:
        case FORMAT_JC3015_2:
            aileron = 3000 - aileron;
            elevator = 3000 - elevator;
            break;

        case FORMAT_MK33041:
            elevator = 3000 - elevator;
            break;
    }

    u8 offset=0;
    if(getProtocolOpt() == FORMAT_CX10_BLUE)
        offset = 4;
    mPacketBuf[0] = bind ? 0xAA : 0x55;
    mPacketBuf[1] = mTxID[0];
    mPacketBuf[2] = mTxID[1];
    mPacketBuf[3] = mTxID[2];
    mPacketBuf[4] = mTxID[3];
    // for CX-10A [5]-[8] is aircraft id received during bind
    mPacketBuf[5+offset] = aileron & 0xff;
    mPacketBuf[6+offset] = (aileron >> 8) & 0xff;
    mPacketBuf[7+offset] = elevator & 0xff;
    mPacketBuf[8+offset] = (elevator >> 8) & 0xff;
    mPacketBuf[9+offset] = throttle & 0xff;
    mPacketBuf[10+offset] = (throttle >> 8) & 0xff;
    mPacketBuf[11+offset] = rudder & 0xff;
    mPacketBuf[12+offset] = ((rudder >> 8) & 0xff) | GET_FLAG(CHANNEL_FLIP, 0x10);  // 0x10 here is a flip flag

    // rate mode is 2 lsb of byte 13
    mPacketBuf[13+offset] = 0;
    if (getControl(CH_AUX1) > 0) {
        if (getControl(CH_AUX1) < CHAN_MAX_VALUE / 2)
            mPacketBuf[13+offset] |= 1;
        else
            mPacketBuf[13+offset] |= 2; // headless on CX-10A
    }

    switch(getProtocolOpt()) {
        case FORMAT_CX10_BLUE:
            if(getControl(CHANNEL_PICTURE) <= 0)
                mPacketBuf[13+offset] |= 0x10;
            mPacketBuf[13+offset] |= GET_FLAG(CHANNEL_VIDEO, 0x08);
            break;

        case FORMAT_Q282:
        case FORMAT_Q242:
            mPacketBuf[14] = GET_FLAG(CHANNEL_FLIP, 0x80)
                       | GET_FLAG(CHANNEL_LED, 0x40)
                       | GET_FLAG(CHANNEL_HEADLESS, 0x08)
                       | GET_FLAG(CHANNEL_XCAL, 0x04)
                       | GET_FLAG(CHANNEL_YCAL, 0x02);

            if (getProtocolOpt() == FORMAT_Q282) {
                mPacketBuf[13] = 0x03 | GET_FLAG(CHANNEL_RTH, 0x80);
                mPacketBuf[14] |= GET_FLAG(CHANNEL_PICTURE, 0x10)
                            | getVideoState(CHANNEL_VIDEO, mPacketBuf[14] & 0x21);
                memcpy(&mPacketBuf[15], "\x10\x10\xaa\xaa\x00\x00", 6);
            } else {
                mPacketBuf[13] = 0x02 | GET_FLAG(CHANNEL_RTH, 0x80);
                mPacketBuf[14] |= GET_FLAG(CHANNEL_PICTURE, 0x01)
                            | GET_FLAG(CHANNEL_VIDEO, 0x10);
                memcpy(&mPacketBuf[15], "\x10\x10\x00\x00\x00\x00", 6);
            }
            break;

        case FORMAT_DM007:
            mPacketBuf[13] |= GET_FLAG(CHANNEL_HEADLESS, FLAG_HEADLESS);
            mPacketBuf[14] = GET_FLAG(CHANNEL_PICTURE, FLAG_SNAPSHOT)
                       | GET_FLAG(CHANNEL_VIDEO, FLAG_VIDEO);
            break;

        case FORMAT_JC3015_1:
            mPacketBuf[14] = GET_FLAG(CHANNEL_PICTURE, BV(3))
                       | GET_FLAG(CHANNEL_VIDEO, BV(4));
            break;

        case FORMAT_JC3015_2:
            mPacketBuf[14] = GET_FLAG(CHANNEL_PICTURE, BV(3)); // this channel controls lights
            if(getControl(CHANNEL_FLIP) > CHAN_MAX_VALUE / 2) { // double flip
                mPacketBuf[12] &= ~0x10;
                mPacketBuf[14] |= BV(4);
            }
            break;

        case FORMAT_MK33041:
            mPacketBuf[13] |= GET_FLAG(CHANNEL_PICTURE, BV(7))
                        | GET_FLAG(CHANNEL_RTH, BV(2));
            mPacketBuf[14] = GET_FLAG(CHANNEL_VIDEO, BV(0))
                       | GET_FLAG(CHANNEL_HEADLESS, BV(5));
            break;
    }

    // Power on, TX mode, 2byte CRC
    // Why CRC0? xn297 does not interpret it - either 16-bit CRC or nothing
    mDev.XN297_configure(BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PWR_UP));
    if (bind) {
        mDev.writeReg(NRF24L01_05_RF_CH, RF_BIND_CHANNEL);
    } else {
        mDev.writeReg(NRF24L01_05_RF_CH, mRFChanBufs[mCurRFChan++]);
        mCurRFChan %= MAX_RF_CHANNELS;
    }
    // clear mPacketBuf status bits and TX FIFO
    mDev.writeReg(NRF24L01_07_STATUS, 0x70);
    mDev.flushTx();
    mDev.XN297_writePayload(mPacketBuf, mPacketSize);

    if (isRFPowerUpdated()) {
        mDev.setRFPower(getRFPower());
        clearRFPowerUpdated();
    }
}

void RFProtocolCX10::init1(void)
{
    mDev.initialize();
    mDev.setRFMode(RF_TX);

    // SPI trace of stock TX has these writes to registers that don't appear in
    // nRF24L01 or Beken 2421 datasheets.  Uncomment if you have an XN297 chip?
    // NRF24L01_WriteRegisterMulti(0x3f, "\x4c\x84\x67,\x9c,\x20", 5);
    // NRF24L01_WriteRegisterMulti(0x3e, "\xc9\x9a\xb0,\x61,\xbb,\xab,\x9c", 7);
    // NRF24L01_WriteRegisterMulti(0x39, "\x0b\xdf\xc4,\xa7,\x03,\xab,\x9c", 7);

    mDev.XN297_setTxAddr(mRxTxAddrBuf, 5);
    mDev.XN297_setRxAddr(mRxTxAddrBuf, 5);
    mDev.flushTx();
    mDev.flushRx();
    mDev.writeReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    mDev.writeReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
    mDev.writeReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0 only
    mDev.writeReg(NRF24L01_11_RX_PW_P0, mPacketSize); // bytes of data payload for rx pipe 1
    mDev.writeReg(NRF24L01_05_RF_CH, RF_BIND_CHANNEL);
    mDev.writeReg(NRF24L01_06_RF_SETUP, 0x07);
    mDev.setBitrate(NRF24L01_BR_1M);
    mDev.setRFPower(getRFPower());

    // this sequence necessary for module from stock tx
    mDev.readReg(NRF24L01_1D_FEATURE);
    mDev.activate(0x73);                            // Activate feature register
    mDev.readReg(NRF24L01_1D_FEATURE);

    mDev.writeReg(NRF24L01_1C_DYNPD, 0x00);       // Disable dynamic payload length on all pipes
    mDev.writeReg(NRF24L01_1D_FEATURE, 0x00);     // Set feature bits on
}

u16 RFProtocolCX10::callState(u32 now, u32 expected)
{
    switch (mState) {
    case CX10_INIT1:
        mState = mBindState;
        break;

    case CX10_BIND1:
        if (mBindCtr == 0) {
            mState = CX10_DATA;
        } else {
            sendPacket(1);
            mBindCtr -= 1;
        }
        break;

    case CX10_BIND2:
        if (mDev.readReg(NRF24L01_07_STATUS) & BV(NRF24L01_07_RX_DR)) { // RX fifo data ready
            mDev.XN297_readPayload(mPacketBuf, mPacketSize);
            mDev.setRFMode(RF_IDLE);
            mDev.setRFMode(RF_TX);
            if(mPacketBuf[9] == 1) {
                mState = CX10_BIND1;
            }
        } else {
            mDev.setRFMode(RF_IDLE);
            mDev.setRFMode(RF_TX);
            sendPacket(1);
            delayMicroseconds(300);

            // switch to RX mode
            mDev.setRFMode(RF_IDLE);
            mDev.flushRx();
            mDev.setRFMode(RF_RX);
            mDev.XN297_configure(BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO)
                          | BV(NRF24L01_00_PWR_UP) | BV(NRF24L01_00_PRIM_RX));
        }
        break;

    case CX10_DATA:
        sendPacket(0);
        break;
    }

    return mPacketPeriod;
}

// Generate address to use from TX id and manufacturer id (STM32 unique id)
void RFProtocolCX10::initTxID(void)
{
    u32 id   = getControllerID();
    u32 lfsr = 0xb2c54a2ful;

    for (int i = 0; i < 4; ++i) {
        rand32_r(&lfsr, (id & 0xff));
        id >>= 8;
    }

    // Pump zero bytes for LFSR to diverge more
    for (u8 i = 0; i < sizeof(lfsr); ++i)
        rand32_r(&lfsr, 0);

    // tx id
    mTxID[0] = (lfsr >> 24) & 0xFF;
    mTxID[1] = ((lfsr >> 16) & 0xFF) % 0x30;
    mTxID[2] = (lfsr >> 8) & 0xFF;
    mTxID[3] = lfsr & 0xFF;
    // rf channels
    switch (getProtocolOpt()) {
    case FORMAT_Q282:
    case FORMAT_Q242:
        mRFChanBufs[0] = 0x46;
        mRFChanBufs[1] = 0x48;
        mRFChanBufs[2] = 0x4a;
        mRFChanBufs[3] = 0x4c;
        if(getProtocolOpt() == FORMAT_Q242)
            for (u8 i=0; i < 4; i++)
                mRFChanBufs[i] += 2;
        break;

    default:
        mRFChanBufs[0] = 0x03 + (mTxID[0] & 0x0F);
        mRFChanBufs[1] = 0x16 + (mTxID[0] >> 4);
        mRFChanBufs[2] = 0x2D + (mTxID[1] & 0x0F);
        mRFChanBufs[3] = 0x40 + (mTxID[1] >> 4);
    }
}

int RFProtocolCX10::init(u8 bind)
{
    __PRINT_FUNC__;
    RFProtocol::registerCallback(this);
    mPacketCtr = 0;
    mCurRFChan = 0;
    memcpy(mRxTxAddrBuf, "\xcc\xcc\xcc\xcc\cc", sizeof(mRxTxAddrBuf));

    mPacketSize = 0;
    switch (getProtocolOpt()) {
        case FORMAT_Q282:
        case FORMAT_Q242:
            mPacketSize = Q282_PACKET_SIZE - CX10_PACKET_SIZE;      // difference in packet size

        case FORMAT_CX10_GREEN:
        case FORMAT_DM007:
        case FORMAT_JC3015_1:
        case FORMAT_JC3015_2:
        case FORMAT_MK33041:
            mPacketSize += CX10_PACKET_SIZE;
            mPacketPeriod = CX10_PACKET_PERIOD;
            mBindState = CX10_BIND1;
            mBindCtr = BIND_COUNT;
            break;

        case FORMAT_CX10_BLUE:
            mPacketSize = CX10A_PACKET_SIZE;
            mPacketPeriod = CX10A_PACKET_PERIOD;
            mBindState = CX10_BIND2;
            mBindCtr=0;
            for(u8 i=0; i<4; i++)
                mPacketBuf[5+i] = 0xFF;                             // clear aircraft id
            mPacketBuf[9] = 0;
            break;
    }
    initTxID();
    init1();
    mState = CX10_INIT1;
    startState(INITIAL_WAIT);

    return 0;
}

int RFProtocolCX10::close(void)
{
    mDev.initialize();
    return (mDev.reset() ? 1L : -1L);
}

int RFProtocolCX10::reset(void)
{
    return close();
}

int RFProtocolCX10::getInfo(s8 id, u8 *data)
{
    u8 size;

    size = RFProtocol::getInfo(id, data);
    if (size == 0) {
        switch (id) {
            case INFO_STATE:
                *data = mState;
                size = 1;
                break;

            case INFO_CHANNEL:
                *data = mRFChanBufs[mCurRFChan];
                size = 1;
                break;

            case INFO_PACKET_CTR:
                size = sizeof(mPacketCtr);
                *((u32*)data) = mPacketCtr;
                break;
        }
    }
    return size;
}

