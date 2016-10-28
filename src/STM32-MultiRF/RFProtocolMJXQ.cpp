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
#include "RFProtocolMJXQ.h"
#include "utils.h"

#define BIND_COUNT          150
#define PACKET_PERIOD       4000 // Timeout for callback in uSec

#define INITIAL_WAIT        500
#define ADDRESS_LENGTH      5

enum {
    FORMAT_WLH08 = 0,
    FORMAT_X600,
    FORMAT_X800,
    FORMAT_H26D,
    FORMAT_E010,
};

enum {
    MJXq_INIT1 = 0,
    MJXq_BIND1,
    MJXq_DATA
};

#define CHANNEL_LED         CH_AUX1
#define CHANNEL_FLIP        CH_AUX2
#define CHANNEL_PICTURE     CH_AUX3
#define CHANNEL_VIDEO       CH_AUX4
#define CHANNEL_HEADLESS    CH_AUX5
#define CHANNEL_RTH         CH_AUX6
#define CHANNEL_AUTOFLIP    CH_AUX7  // X800, X600
#define CHANNEL_PAN         CH_AUX8  // H26D
#define CHANNEL_TILT        CH_AUX9

// haven't figured out txid<-->rf channel mapping for MJX models
struct id_ch_map {
    u8 txid[3];
    u8 rfchan[4];
};

const struct id_ch_map TBL_TX_RF_MAP[] =
{
    {{0xF8, 0x4F, 0x1C}, {0x0A, 0x46, 0x3A, 0x42}},
    {{0xC8, 0x6E, 0x02}, {0x0A, 0x3C, 0x36, 0x3F}},
    {{0x48, 0x6A, 0x40}, {0x0A, 0x43, 0x36, 0x3F}}
};

u8 RFProtocolMJXQ::calcCheckSum(void)
{
    u8 sum = mPacketBuf[0];

    for (u8 i = 1; i < MAX_PACKET_SIZE - 1; i++)
        sum += mPacketBuf[i];
    return sum;
}

#define LIMIT_CHAN(X)   (X < CHAN_MIN_VALUE ? CHAN_MIN_VALUE : (X > CHAN_MAX_VALUE ? CHAN_MAX_VALUE : X))

#define PAN_TILT_COUNT  16   // for H26D - match stock tx timing
#define PAN_DOWN        0x08
#define PAN_UP          0x04
#define TILT_DOWN       0x20
#define TILT_UP         0x10

u8 RFProtocolMJXQ::calcPanTilt(void)
{
    static u8 count;
    u8 pan = 0;

    count++;

    s32 ch = LIMIT_CHAN(RFProtocol::getControl(CHANNEL_PAN));
    if ((ch < CHAN_MIN_VALUE / 2 || ch > CHAN_MAX_VALUE / 2) && (count & PAN_TILT_COUNT))
        pan = ch < 0 ? PAN_DOWN : PAN_UP;

    ch = LIMIT_CHAN(RFProtocol::getControl(CHANNEL_TILT));
    if ((ch < CHAN_MIN_VALUE / 2 || ch > CHAN_MAX_VALUE / 2) && (count & PAN_TILT_COUNT))
        return pan + (ch < 0 ? TILT_DOWN : TILT_UP);

    return pan;
}

#define GET_FLAG(ch, mask) (RFProtocol::getControl(ch) > 0 ? mask : 0)
#define GET_FLAG_INV(ch, mask) (RFProtocol::getControl(ch) < 0 ? mask : 0)

//#define CHAN2TRIM(X) ((((X) & 0x80 ? 0xff - (X) : 0x80 + (X)) >> 1) + 0x00)
#define CHAN2TRIM(X) (((X) & 0x80 ? (X) : 0x7f - (X)) >> 1)

void RFProtocolMJXQ::sendPacket(u8 bind)
{
    mPacketBuf[0] = getControl_8b(CH_THROTTLE);             // throttle
    mPacketBuf[1] = getControl_s8b(CH_RUDDER);              // rudder
    mPacketBuf[4] = 0x40;                                   // rudder does not work well with dyntrim

    mPacketBuf[2] = getControl_s8b(CH_ELEVATOR);            // elevator
    // driven trims cause issues when headless is enabled
    mPacketBuf[5] = GET_FLAG(CHANNEL_HEADLESS, 1) ? 0x40 : CHAN2TRIM(mPacketBuf[2]); // trim elevator

    mPacketBuf[3] = getControl_s8b(CH_AILERON);             // aileron
    mPacketBuf[6] = GET_FLAG(CHANNEL_HEADLESS, 1) ? 0x40 : CHAN2TRIM(mPacketBuf[3]); // trim aileron

    mPacketBuf[7] = mTxID[0];
    mPacketBuf[8] = mTxID[1];
    mPacketBuf[9] = mTxID[2];

    mPacketBuf[10] = 0;   // overwritten below for feature bits
    mPacketBuf[11] = 0;   // overwritten below for X600
    mPacketBuf[12] = 0;
    mPacketBuf[13] = 0;

    mPacketBuf[14] = 0xc0;  // bind value

    switch (getProtocolOpt()) {
        case FORMAT_H26D:
            mPacketBuf[10] = calcPanTilt();
            // fall through on purpose - no break

        case FORMAT_WLH08:
        case FORMAT_E010:
            mPacketBuf[10] += GET_FLAG(CHANNEL_RTH, 0x02)
                        | GET_FLAG(CHANNEL_HEADLESS, 0x01);
            if (!bind) {
                mPacketBuf[14] = 0x00    // 0x04 : high rate
                           | GET_FLAG(CHANNEL_FLIP, 0x01)
                           | GET_FLAG(CHANNEL_PICTURE, 0x08)
                           | GET_FLAG(CHANNEL_VIDEO, 0x10)
                           | GET_FLAG_INV(CHANNEL_LED, 0x20); // air/ground mode
            }
            break;

        case FORMAT_X600:
            mPacketBuf[10] = GET_FLAG_INV(CHANNEL_LED, 0x02);
            mPacketBuf[11] = GET_FLAG(CHANNEL_RTH, 0x01);
            if (!bind) {
                mPacketBuf[14] = 0x02      // always high rates by bit2 = 1
                           | GET_FLAG(CHANNEL_FLIP, 0x04)
                           | GET_FLAG(CHANNEL_AUTOFLIP, 0x10)
                           | GET_FLAG(CHANNEL_HEADLESS, 0x20);
            }
            break;

        case FORMAT_X800:
        default:
            mPacketBuf[10] = 0x10
                       | GET_FLAG_INV(CHANNEL_LED, 0x02)
                       | GET_FLAG(CHANNEL_AUTOFLIP, 0x01);
            if (!bind) {
                mPacketBuf[14] = 0x02      // always high rates by bit2 = 1
                           | GET_FLAG(CHANNEL_FLIP, 0x04)
                           | GET_FLAG(CHANNEL_PICTURE, 0x08)
                           | GET_FLAG(CHANNEL_VIDEO, 0x10);
            }
    }
    mPacketBuf[15] = calcCheckSum();

    // Power on, TX mode, 2byte CRC
    if (getProtocolOpt() == FORMAT_H26D) {
        mDev.setRFMode(RF_TX);
    } else {
        mDev.XN297_configure(BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PWR_UP));
    }

    mDev.writeReg(NRF24L01_05_RF_CH, mRFChanBufs[mCurRFChan++ / 2]);
    mCurRFChan %= 2 * MAX_RF_CHANNELS;  // channels repeated

    mDev.writeReg(NRF24L01_07_STATUS, 0x70);
    mDev.flushTx();

    if (getProtocolOpt() == FORMAT_H26D) {
        mDev.writePayload(mPacketBuf, MAX_PACKET_SIZE);
    } else {
        mDev.XN297_writePayload(mPacketBuf, MAX_PACKET_SIZE);
    }

    // Check and adjust transmission power. We do this after
    // transmission to not bother with timeout after power
    // settings change -  we have plenty of time until next
    // mPacketBuf.
    if (isRFPowerUpdated()) {
        mDev.setRFPower(getRFPower());
        clearRFPowerUpdated();
    }
}

void RFProtocolMJXQ::init1()
{
    u8 rx_tx_addr[ADDRESS_LENGTH];

    memcpy(rx_tx_addr, "\x6d\x6a\x77\x77\x77", sizeof(rx_tx_addr));

    if (getProtocolOpt() == FORMAT_WLH08) {
        memcpy(mRFChanBufs, "\x12\x22\x32\x42", sizeof(mRFChanBufs));
    } else if (getProtocolOpt() == FORMAT_H26D || getProtocolOpt() == FORMAT_E010) {
        memcpy(mRFChanBufs, "\x36\x3e\x46\x2e", sizeof(mRFChanBufs));
    } else {
        memcpy(mRFChanBufs, "\x0a\x35\x42\x3d", sizeof(mRFChanBufs));
        memcpy(rx_tx_addr, "\x6d\x6a\x73\x73\x73", sizeof(rx_tx_addr));
    }

    __PRINT_FUNC__;
    mDev.initialize();
    mDev.setRFMode(RF_TX);

    // SPI trace of stock TX has these writes to registers that don't appear in
    // nRF24L01 or Beken 2421 datasheets.  Uncomment if you have an XN297 chip?
    // NRF24L01_WriteRegisterMulti(0x3f, "\x4c\x84\x67,\x9c,\x20", 5);
    // NRF24L01_WriteRegisterMulti(0x3e, "\xc9\x9a\xb0,\x61,\xbb,\xab,\x9c", 7);
    // NRF24L01_WriteRegisterMulti(0x39, "\x0b\xdf\xc4,\xa7,\x03,\xab,\x9c", 7);

    if (getProtocolOpt() == FORMAT_H26D) {
        mDev.writeRegMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, sizeof(rx_tx_addr));
    } else {
        mDev.XN297_setTxAddr(rx_tx_addr, sizeof(rx_tx_addr));
    }

    mDev.flushTx();
    mDev.flushRx();
    mDev.writeReg(NRF24L01_07_STATUS,     0x70);     // Clear data ready, data sent, and retransmit
    mDev.writeReg(NRF24L01_01_EN_AA,      0x00);      // No Auto Acknowldgement on all data pipes
    mDev.writeReg(NRF24L01_02_EN_RXADDR,  0x01);
    mDev.writeReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
    mDev.writeReg(NRF24L01_11_RX_PW_P0,   MAX_PACKET_SIZE);

    if (getProtocolOpt() == FORMAT_E010) {
        mDev.setBitrate(NRF24L01_BR_250K);
    } else {
        mDev.setBitrate(NRF24L01_BR_1M);
    }
    mDev.setRFPower(getRFPower());

    mDev.activate(0x73);                          // Activate feature register
    mDev.writeReg(NRF24L01_1C_DYNPD, 0x00);       // Disable dynamic payload length on all pipes
    mDev.writeReg(NRF24L01_1D_FEATURE, 0x00);
    mDev.activate(0x73);
}

void RFProtocolMJXQ::init2()
{
        __PRINT_FUNC__;
    if (getProtocolOpt() == FORMAT_H26D) {
        memcpy(mRFChanBufs, "\x32\x3e\x42\x4e", sizeof(mRFChanBufs));
    } else if (getProtocolOpt() != FORMAT_WLH08 && getProtocolOpt() != FORMAT_E010) {
        memcpy(mRFChanBufs, TBL_TX_RF_MAP[getControllerID() % (sizeof(TBL_TX_RF_MAP) / sizeof(TBL_TX_RF_MAP[0]))].rfchan, MAX_RF_CHANNELS);
    }
}

u16 RFProtocolMJXQ::callState(u32 now, u32 expected)
{
    switch (mState) {
        case MJXq_INIT1:
            mState = MJXq_BIND1;
            break;

        case MJXq_BIND1:
            if (mBindCtr == 0) {
                init2();
                mState = MJXq_DATA;
                LOG(F("DATA STATE\n"));
            } else {
                sendPacket(1);
                mBindCtr -= 1;
            }
            break;

        case MJXq_DATA:
            sendPacket(0);
            break;
    }

    return PACKET_PERIOD;
}

void RFProtocolMJXQ::initTxID(void)
{
    u32 id   = getControllerID();
    u32 lfsr = id;

    __PRINT_FUNC__;
    // Pump zero bytes for LFSR to diverge more
    for (u8 i = 0; i < sizeof(lfsr); ++i)
      rand32_r(&lfsr, 0);

    if (getProtocolOpt() == FORMAT_E010) {
        LOG("PROTOCOL E010\n");
        // mTxID must be multiple of 8
        mTxID[0] = (lfsr >> 16) & 0xf8;
        mTxID[1] = ((lfsr >> 8 ) & 0xf0) | 0x0c;
        mTxID[2] = lfsr & 0xf0;
    }
    else if (getProtocolOpt() == FORMAT_WLH08) {
        // mTxID must be multiple of 8
        mTxID[0] = (lfsr >> 16) & 0xf8;
        mTxID[1] = (lfsr >> 8 ) & 0xff;
        mTxID[2] = lfsr & 0xff;
    } else {
        memcpy(mTxID, TBL_TX_RF_MAP[id % (sizeof(TBL_TX_RF_MAP) / sizeof(TBL_TX_RF_MAP[0]))].txid, sizeof(mTxID));
    }
}

int RFProtocolMJXQ::init(u8 bind)
{
    __PRINT_FUNC__;
    RFProtocol::registerCallback(this);
    mPacketCtr = 0;
    mCurRFChan = 0;

    mBindCtr = BIND_COUNT;
    initTxID();
    init1();
    mState = MJXq_INIT1;
    startState(INITIAL_WAIT);

    return 0;
}

int RFProtocolMJXQ::close(void)
{
    mDev.initialize();
    return (mDev.reset() ? 1L : -1L);
}

int RFProtocolMJXQ::reset(void)
{
    return close();
}

int RFProtocolMJXQ::getInfo(s8 id, u8 *data)
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

