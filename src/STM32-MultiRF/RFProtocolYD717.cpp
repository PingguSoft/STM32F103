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
#include "RFProtocolYD717.h"
#include "utils.h"


#define PAYLOADSIZE          8  // receive data pipes set to this size, but unused

#define MAX_BIND_COUNT      60
#define PACKET_PERIOD_uS  8000
#define INITIAL_WAIT_uS  50000
#define PACKET_CHKTIME_uS  500  // Time to wait if packet not yet acknowledged or timed out

// Stock tx fixed frequency is 0x3C. Receiver only binds on this freq.
#define RF_CHANNEL          0x3C

#define FLAG_FLIP           0x0F
#define FLAG_LIGHT          0x80
#define FLAG_PICTURE        0x40
#define FLAG_VIDEO          0x20
#define FLAG_HEADLESS       0x10

// Packet ack status values
enum {
    PKT_PENDING = 0,
    PKT_ACKED,
    PKT_TIMEOUT
};

enum {
    YD717_INIT1 = 0,
    YD717_BIND2,
    YD717_BIND3,
    YD717_BIND4,
    YD717_DATA  = 0x10
};

enum {
    PROTO_OPT_YD717       = 0,
    PROTO_OPT_SKY_WALKER  = 1,
    PROTO_OPT_SYMA_X4     = 2,
    PROTO_OPT_XINXUN      = 3,
    PROTO_OPT_NI_HUI      = 4,
};

u8 RFProtocolYD717::checkStatus()
{
    u8 stat = mDev.readReg(NRF24L01_07_STATUS);

    if (stat & BV(NRF24L01_07_TX_DS)) {
        return PKT_ACKED;
    } else if (stat & BV(NRF24L01_07_MAX_RT)) {
        return PKT_TIMEOUT;
    } else {
        return PKT_PENDING;
    }
}

u8 RFProtocolYD717::getControl(u8 id)
{
    s32 ch = RFProtocol::getControl(id);

    if (ch < CHAN_MIN_VALUE) {
        ch = CHAN_MIN_VALUE;
    } else if (ch > CHAN_MAX_VALUE) {
        ch = CHAN_MAX_VALUE;
    }

    return (u8) (((ch * 0xFF / CHAN_MAX_VALUE) + 0x100) >> 1);
}

void RFProtocolYD717::getControls(u8* throttle, u8* rudder, u8* elevator, u8* aileron,
                          u8* flags, u8* rudder_trim, u8* elevator_trim, u8* aileron_trim)
{
    // Protocol is registered AETRF, that is
    // Aileron is channel 1, Elevator - 2, Throttle - 3, Rudder - 4, Flip control - 5
    u8  opt = getProtocolOpt();

    // Channel 3
    *throttle = getControl(CH_THROTTLE);

    // Channel 4
    *rudder = getControl(CH_RUDDER);
    if(opt == PROTO_OPT_XINXUN || opt == PROTO_OPT_SKY_WALKER) {
      //*rudder_trim = (0xff - *rudder) >> 1;
    } else {
      *rudder = 0xff - *rudder;
    }
    *rudder_trim = *rudder >> 1;

    // Channel 2
    *elevator = getControl(CH_ELEVATOR);
    *elevator_trim = *elevator >> 1;

    // Channel 1
    *aileron = getControl(CH_AILERON);
    if(getProtocolOpt() != PROTO_OPT_SKY_WALKER) {
        *aileron = 0xff - *aileron;
    }
    *aileron_trim = *aileron >> 1;

    // Channel 5
    if (RFProtocol::getControl(CH_AUX1) <= 0)
      *flags &= ~FLAG_FLIP;
    else
      *flags |= FLAG_FLIP;

    // Channel 6
    if (RFProtocol::getControl(CH_AUX2) <= 0)
      *flags &= ~FLAG_LIGHT;
    else
      *flags |= FLAG_LIGHT;

    // Channel 7
    if (RFProtocol::getControl(CH_AUX3) <= 0)
      *flags &= ~FLAG_PICTURE;
    else
      *flags |= FLAG_PICTURE;

    // Channel 8
    if (RFProtocol::getControl(CH_AUX4) <= 0)
      *flags &= ~FLAG_VIDEO;
    else
      *flags |= FLAG_VIDEO;

    // Channel 9
    if (RFProtocol::getControl(CH_AUX5) <= 0)
      *flags &= ~FLAG_HEADLESS;
    else
      *flags |= FLAG_HEADLESS;
}

void RFProtocolYD717::sendPacket(u8 bind)
{
    if (bind) {
        mPacketBuf[0]= mRxTxAddrBuf[0]; // send data phase address in first 4 bytes
        mPacketBuf[1]= mRxTxAddrBuf[1];
        mPacketBuf[2]= mRxTxAddrBuf[2];
        mPacketBuf[3]= mRxTxAddrBuf[3];
        mPacketBuf[4] = 0x56;
        mPacketBuf[5] = 0xAA;
        mPacketBuf[6] = (getProtocolOpt() == PROTO_OPT_NI_HUI) ? 0x00 : 0x32;
        mPacketBuf[7] = 0x00;
    } else {
        if (getProtocolOpt() == PROTO_OPT_YD717)
            getControls(&mPacketBuf[0], &mPacketBuf[1], &mPacketBuf[3], &mPacketBuf[4], &mPacketBuf[7], &mPacketBuf[6], &mPacketBuf[2], &mPacketBuf[5]);
        else
            getControls(&mPacketBuf[0], &mPacketBuf[1], &mPacketBuf[3], &mPacketBuf[4], &mPacketBuf[7], &mPacketBuf[2], &mPacketBuf[5], &mPacketBuf[6]);
    }

    // clear mPacketBuf status bits and TX FIFO
    mDev.writeReg(NRF24L01_07_STATUS, BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT));
    mDev.flushTx();

    if(getProtocolOpt() == PROTO_OPT_YD717) {
        mDev.writePayload(mPacketBuf, 8);
    } else {
        mPacketBuf[8] = mPacketBuf[0];  // checksum
        for (u8 i = 1; i < 8; i++)
            mPacketBuf[8] += mPacketBuf[i];
        mPacketBuf[8] = ~mPacketBuf[8];
        mDev.writePayload(mPacketBuf, 9);
    }
    ++mPacketCtr;

//    radio.ce(HIGH);
//    delayMicroseconds(15);
    // It saves power to turn off radio after the transmission,
    // so as long as we have pins to do so, it is wise to turn
    // it back.
//    radio.ce(LOW);

    // Check and adjust transmission power. We do this after
    // transmission to not bother with timeout after power
    // settings change -  we have plenty of time until next
    // packet
    if (isRFPowerUpdated()) {
        mDev.setRFPower(getRFPower());
        clearRFPowerUpdated();
    }
}


void RFProtocolYD717::initRxTxAddr(void)
{
    u32 lfsr = 0xb2c54a2ful; //getControllerID();

    // Pump zero bytes for LFSR to diverge more
    for (u8 i = 0; i < sizeof(lfsr); ++i)
      rand32_r(&lfsr, 0);

    mRxTxAddrBuf[0] = (lfsr >> 24) & 0xff;
    mRxTxAddrBuf[1] = (lfsr >> 16) & 0xff;
    mRxTxAddrBuf[2] = (lfsr >>  8) & 0xff;
    mRxTxAddrBuf[3] = (lfsr >>  0) & 0xff;
    mRxTxAddrBuf[4] = 0xC1;
}

static const PROGMEM u8 TBL_INIT_REGS[] = {
    BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_PWR_UP),// 00 : 2-bytes CRC
    0x3F,                                           // 01 : Auto Acknoledgement on all data pipes
    0x3F,                                           // 02 : Enable all data pipes (even though not used?)
    0x03,                                           // 03 : 5-byte RX/TX address
    0x1A,                                           // 04 : 500uS retransmit t/o, 10 tries
    RF_CHANNEL,                                     // 05 : RF channel : 3C
    0xff,                                           // 06 : skip
    0x70,                                           // 07 : Clear data ready, data sent, and retransmit
    0xff,                                           // 08 : skip
    0xff,                                           // 09 : skip
    0xff,                                           // 0A : skip
    0xff,                                           // 0B : skip
    0xC3,                                           // 0C : LSB byte of pipe 2 receive address
    0xC4,                                           // 0D :
    0xC5,                                           // 0E :
    0xC6,                                           // 0F :
    0xff,                                           // 10 : skip
    PAYLOADSIZE,                                    // 11 : bytes of data payload for pipe 0
    PAYLOADSIZE,                                    // 12 :
    PAYLOADSIZE,                                    // 13 :
    PAYLOADSIZE,                                    // 14 :
    PAYLOADSIZE,                                    // 15 :
    PAYLOADSIZE,                                    // 16 :
    0x00                                            // 17 : Just in case, no real bits to write here
};

void RFProtocolYD717::init1(void)
{
    u8 val;

    __PRINT_FUNC__;
    mDev.initialize();
    mDev.setRFMode(RF_TX);
    for (u8 i = 0; i < sizeof(TBL_INIT_REGS) ; i++) {
        if (i == NRF24L01_06_RF_SETUP) {
            mDev.setBitrate(NRF24L01_BR_1M);
            mDev.setRFPower(getRFPower());
        } else {
            val = pgm_read_byte(TBL_INIT_REGS + i);
            if (val != 0xff)
                mDev.writeReg(i, val);
        }
    }
    mDev.writeReg(NRF24L01_1C_DYNPD, 0x3F);       // Enable dynamic payload length on all pipes

    // this sequence necessary for module from stock tx
    mDev.readReg(NRF24L01_1D_FEATURE);
    mDev.activate(0x73);                          // Activate feature register
    mDev.readReg(NRF24L01_1D_FEATURE);
    mDev.writeReg(NRF24L01_1C_DYNPD, 0x3F);       // Enable dynamic payload length on all pipes
    mDev.writeReg(NRF24L01_1D_FEATURE, 0x07);     // Set feature bits on

    mDev.writeRegMulti(NRF24L01_0A_RX_ADDR_P0, mRxTxAddrBuf, 5);
    mDev.writeRegMulti(NRF24L01_10_TX_ADDR, mRxTxAddrBuf, 5);

    // Check for Beken BK2421/BK2423 chip
    // It is done by using Beken specific activate code, 0x53
    // and checking that status register changed appropriately
    // There is no harm to run it on nRF24L01 because following
    // closing activate command changes state back even if it
    // does something on nRF24L01
    mDev.activate(0x53); // magic for BK2421 bank switch
    LOG("Trying to switch banks\n");
    if (mDev.readReg(NRF24L01_07_STATUS) & 0x80) {
        LOG("BK2421 detected\n");
        // Beken registers don't have such nice names, so we just mention
        // them by their numbers
        // It's all magic, eavesdropped from real transfer and not even from the
        // data sheet - it has slightly different values
        mDev.writeRegMulti(0x00, (u8 *) "\x40\x4B\x01\xE2", 4);
        mDev.writeRegMulti(0x01, (u8 *) "\xC0\x4B\x00\x00", 4);
        mDev.writeRegMulti(0x02, (u8 *) "\xD0\xFC\x8C\x02", 4);
        mDev.writeRegMulti(0x03, (u8 *) "\x99\x00\x39\x21", 4);
        mDev.writeRegMulti(0x04, (u8 *) "\xD9\x96\x82\x1B", 4);
        mDev.writeRegMulti(0x05, (u8 *) "\x24\x06\x7F\xA6", 4);
        mDev.writeRegMulti(0x0C, (u8 *) "\x00\x12\x73\x00", 4);
        mDev.writeRegMulti(0x0D, (u8 *) "\x46\xB4\x80\x00", 4);
        mDev.writeRegMulti(0x04, (u8 *) "\xDF\x96\x82\x1B", 4);
        mDev.writeRegMulti(0x04, (u8 *) "\xD9\x96\x82\x1B", 4);

        u8 id[4];
        mDev.readRegMulti(0x08, id, 4);
        DUMP("ID", id, 4);
    } else {
        LOG("nRF24L01 detected\n");
    }
    mDev.activate(0x53); // switch bank back
}

void RFProtocolYD717::init2(void)
{
     __PRINT_FUNC__;
    // for bind packets set address to prearranged value known to receiver
    u8 bind_rx_tx_addr[5];

    if (getProtocolOpt() == PROTO_OPT_SYMA_X4) {
        memset(bind_rx_tx_addr, 0x60, 5);
    } else if (getProtocolOpt() == PROTO_OPT_NI_HUI) {
        memset(bind_rx_tx_addr, 0x64, 5);
    } else {
        memset(bind_rx_tx_addr, 0x65, 5);
    }
    mDev.writeRegMulti(NRF24L01_0A_RX_ADDR_P0, bind_rx_tx_addr, 5);
    mDev.writeRegMulti(NRF24L01_10_TX_ADDR, bind_rx_tx_addr, 5);
}

void RFProtocolYD717::init3(void)
{
    __PRINT_FUNC__;
    // set rx/tx address for data phase
    mDev.writeRegMulti(NRF24L01_0A_RX_ADDR_P0, mRxTxAddrBuf, 5);
    mDev.writeRegMulti(NRF24L01_10_TX_ADDR, mRxTxAddrBuf, 5);
}

void RFProtocolYD717::updateTelemetry(void) {
  static u8 frameloss = 0;

  frameloss += mDev.readReg(NRF24L01_08_OBSERVE_TX) >> 4;
  mDev.writeReg(NRF24L01_05_RF_CH, RF_CHANNEL);   // reset packet loss counter
}

u16 RFProtocolYD717::callState(u32 now, u32 expected)
{
    switch (mState) {
    case YD717_INIT1:
        sendPacket(0);
        mState = YD717_BIND3;
        break;

    case YD717_BIND2:
        if (mBindCtr == 0) {
            if (checkStatus() == PKT_PENDING)
                return PACKET_CHKTIME_uS;       // packet send not yet complete

            init3();                            // change to data phase rx/tx address
            sendPacket(0);
            //mState = YD717_BIND3;
            mState = YD717_BIND4;
        } else {
            if (checkStatus() == PKT_PENDING)
                return PACKET_CHKTIME_uS;       // packet send not yet complete

            sendPacket(1);
            mBindCtr--;
        }
        break;

    case YD717_BIND3:
        switch (checkStatus()) {
            case PKT_PENDING:
                return PACKET_CHKTIME_uS;           // packet send not yet complete

            case PKT_ACKED:
                mState = YD717_DATA;
                LOG("BINDED\n");
                break;

            case PKT_TIMEOUT:
                init2();                            // change to bind rx/tx address
                mBindCtr = MAX_BIND_COUNT;
                mState = YD717_BIND2;
                sendPacket(1);
                break;
        }
        break;

    case YD717_BIND4:
        switch (checkStatus()) {
            case PKT_PENDING:
                return PACKET_CHKTIME_uS;               // packet send not yet complete

            case PKT_TIMEOUT:
                LOG("force AutoACK disabled and binded\n");
                mDev.writeReg(NRF24L01_01_EN_AA, 0x00); // disable AutoACK
                // no break

            case PKT_ACKED:
                LOG("BINDED\n");
                mState = YD717_DATA;
                break;
        }
        break;

    case YD717_DATA:
        updateTelemetry();
        if (checkStatus() == PKT_PENDING)
            return PACKET_CHKTIME_uS;           // packet send not yet complete

        sendPacket(0);
        break;
    }

    return PACKET_PERIOD_uS;
}

int RFProtocolYD717::init(u8 bind)
{
    __PRINT_FUNC__;
    RFProtocol::registerCallback(this);
    mPacketCtr = 0;

    initRxTxAddr();
    init1();
    mState = YD717_INIT1;

    startState(INITIAL_WAIT_uS);
    return 0;
}

int RFProtocolYD717::close(void)
{
    mDev.initialize();
    return (mDev.reset() ? 1L : -1L);
}

int RFProtocolYD717::reset(void)
{
    return close();
}

int RFProtocolYD717::getInfo(s8 id, u8 *data)
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
                *data = RF_CHANNEL;
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
