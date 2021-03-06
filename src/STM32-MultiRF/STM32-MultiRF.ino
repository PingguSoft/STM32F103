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

#include <stdarg.h>
#include <SPI.h>
#include <Wire.h>
#include <libmaple/usart.h>

#include "common.h"
#include "utils.h"
#include "RFProtocolSyma.h"
#include "RFProtocolYD717.h"
#include "RFProtocolV2x2.h"
#include "RFProtocolHiSky.h"
#include "RFProtocolCFlie.h"
#include "RFProtocolDevo.h"
#include "RFProtocolDSM.h"
#include "RFProtocolHubsan.h"
#include "RFProtocolFlysky.h"
#include "RFProtocolMJXQ.h"
#include "RFProtocolCX10.h"
#include "RCRcvrPPM.h"
#include "RCRcvrERSkySerial.h"
#include "Telemetry.h"

#define FW_VERSION  0x0120
#define SIMUL       0

static u32          mSelProto = 0;
static u8           mBaudAckLen;
static u8           mBaudChkCtr;
static u8           mBaudAckStr[12];
static RFProtocol   *mRFProto = NULL;
static RCRcvr       *mRcvr = NULL;
static u32          mLastTS = 0;
static u8           mLed = 0;

struct Config {
    u32 dwSignature;
    u32 dwProtoID;
    u32 dwConID;
    u8  ucPower;
};


static inline __always_inline void handle_usart_irq(usart_reg_map *regs)
{
    u8 data;

    if ((regs->CR1 & USART_CR1_RXNEIE) && (regs->SR & USART_SR_RXNE)) {
        data = (u8)regs->DR;
        if (mRcvr != NULL) {
            mRcvr->handleRX(data);
        }
    }

    if ((regs->CR1 & USART_CR1_TXEIE) && (regs->SR & USART_SR_TXE)) {
        if (mRFProto != NULL) {
            u8 ret = mRFProto->getTM().handleTX(&data);
            if (ret > 0) {
                regs->DR = data;
            } else {
                regs->DR = 0;
                regs->CR1 &= ~((u32)USART_CR1_TXEIE);
            }
        }
    }
}

extern "C" void __irq_usart3(void)
{
    handle_usart_irq(USART3_BASE);
}

static u8 initProtocol(u32 id)
{
    u8  ret = 0;

    // protocol
    timer_disable_irq(Timer2.c_dev(), TIMER_CH1);
    if (mRFProto) {
        delete mRFProto;
        mRFProto = NULL;
    }

    switch (RFProtocol::getModule(id)) {
        case TX_NRF24L01: {
            ret = 1;
            switch (RFProtocol::getProtocol(id)) {
                case RFProtocol::PROTO_NRF24L01_SYMAX:
                    mRFProto = new RFProtocolSyma(id);
                    break;

                case RFProtocol::PROTO_NRF24L01_YD717:
                    mRFProto = new RFProtocolYD717(id);
                    break;

                case RFProtocol::PROTO_NRF24L01_MJXQ:
                    mRFProto = new RFProtocolMJXQ(id);
                    break;

                case RFProtocol::PROTO_NRF24L01_CX10:
                    mRFProto = new RFProtocolCX10(id);
                    break;
/*
                case RFProtocol::PROTO_NRF24L01_V2x2:
                    mRFProto = new RFProtocolV2x2(id);
                    break;

                case RFProtocol::PROTO_NRF24L01_HISKY:
                    mRFProto = new RFProtocolHiSky(id);
                    break;

                case RFProtocol::PROTO_NRF24L01_CFLIE:
                    mRFProto = new RFProtocolCFlie(id);
                    break;
*/
                default:
                    ret = 0;
                    break;
            }
        }
        break;

        case TX_CYRF6936:
            ret = 1;
            switch (RFProtocol::getProtocol(id)) {
                case RFProtocol::PROTO_CYRF6936_DEVO:
                    mRFProto = new RFProtocolDevo(id);
                    break;

                case RFProtocol::PROTO_CYRF6936_DSMX:
                    mRFProto = new RFProtocolDSM(id);
                    break;

                default:
                    ret = 0;
                    break;
            }
            break;

/*
        case TX_A7105: {
            ret = 1;
            switch (RFProtocol::getProtocol(id)) {
                case RFProtocol::PROTO_A7105_FLYSKY:
                    mRFProto = new RFProtocolFlysky(id);
                    break;

                case RFProtocol::PROTO_A7105_HUBSAN:
                    mRFProto = new RFProtocolHubsan(id);
                    break;

                default:
                    ret = 0;
                    break;
            }
        }
        break;
*/
    }

    return ret;
}

#if SIMUL
static s16 thr  = CHAN_MIN_VALUE;
static s16 ele  = CHAN_MID_VALUE;
static s16 ail  = CHAN_MAX_VALUE / 2;
static s16 rud  = CHAN_MIN_VALUE / 2;
static s16 step_thr = 10;
static s16 step_ele = 2;
static s16 step_ail = 2;
static s16 step_rud = 2;
static u8  sim = 0;

void simul_setup()
{
    mRcvr->setRC(RFProtocol::CH_THROTTLE, CHAN_MIN_VALUE);
    mRcvr->setRC(RFProtocol::CH_AILERON, 0);
    mRcvr->setRC(RFProtocol::CH_RUDDER, 0);
    mRcvr->setRC(RFProtocol::CH_ELEVATOR, 0);

    struct Config conf;

    conf.dwSignature = 0xCAFEBABE;

    conf.dwProtoID   = RFProtocol::buildID(TX_NRF24L01, RFProtocol::PROTO_NRF24L01_MJXQ, 4);
//    conf.dwProtoID   = RFProtocol::buildID(TX_CYRF6936, RFProtocol::PROTO_CYRF6936_DSMX, 0);
//    conf.dwProtoID   = RFProtocol::buildID(TX_CYRF6936, RFProtocol::PROTO_CYRF6936_DEVO, 0);
//    conf.dwProtoID   = RFProtocol::buildID(TX_NRF24L01, RFProtocol::PROTO_NRF24L01_SYMAX, 0);
//    conf.dwProtoID   = RFProtocol::buildID(TX_NRF24L01, RFProtocol::PROTO_NRF24L01_YD717, 1);

// STM32 unique device ID
//  uint16 *idBase0 =  (uint16 *) (0x1FFFF7E8);
//  uint16 *idBase1 =  (uint16 *) (0x1FFFF7E8+0x02);
//  uint32 *idBase2 =  (uint32 *) (0x1FFFF7E8+0x04);
//  uint32 *idBase3 =  (uint32 *) (0x1FFFF7E8+0x08);

    conf.dwConID     = 0x12345678;
    conf.ucPower     = TXPOWER_10mW;

    if (conf.dwSignature == 0xCAFEBABE) {
        initProtocol(conf.dwProtoID);
        if (mRFProto) {
            mRFProto->setControllerID(conf.dwConID);
            mRFProto->setRFPower(conf.ucPower);
//            mRFProto->init();
        }
    }
}

void simul_loop()
{
    u32 ts = micros();

    if (ts - mLastTS > 500000) {
        mLed = !mLed;
        digitalWrite(PC13, mLed);
        mLastTS = ts;
    }

    if (mRcvr) {
        if (Serial.available()) {
            u8 ch = Serial.read();

            switch (ch) {
                case 'b':
                    sim = 0;
                    mRFProto->init(1);
                    break;

                case 's':
                    sim = !sim;
                    LOG("SIM:%4d\n", sim);
                    break;
            }
        }

        if (sim) {
            static u32 mLastTS;
            if (ts - mLastTS > 10000) {
                if (thr <  CHAN_MIN_VALUE || thr > CHAN_MAX_VALUE)
                    step_thr = -step_thr;

                if (ele <  CHAN_MIN_VALUE || ele > CHAN_MAX_VALUE)
                    step_ele = -step_ele;

                if (ail <  CHAN_MIN_VALUE || ail > CHAN_MAX_VALUE)
                    step_ail = -step_ail;

                if (rud <  CHAN_MIN_VALUE || rud > CHAN_MAX_VALUE)
                    step_rud = -step_rud;

                thr += step_thr;
                ele += step_ele;
                ail += step_ail;
                rud += step_rud;

                mRcvr->setRC(RFProtocol::CH_THROTTLE, thr);
//                mRcvr->setRC(RFProtocol::CH_AILERON, ail);
//                mRcvr->setRC(RFProtocol::CH_RUDDER, rud);
//                mRcvr->setRC(RFProtocol::CH_ELEVATOR, ele);
                LOG("T:%4d R:%4d E:%4d A:%4d %4d %4d %4d %4d\n", mRcvr->getRC(0), mRcvr->getRC(1), mRcvr->getRC(2), mRcvr->getRC(3), mRcvr->getRC(4),
                    mRcvr->getRC(5), mRcvr->getRC(6), mRcvr->getRC(7), mRcvr->getRC(8));
                mLastTS = ts;
            }
        }
    }

    if (mRFProto) {
        mRFProto->injectControls(mRcvr->getRCs(), mRcvr->getChCnt());
        mRFProto->getTM().update();
    }
    DRAIN_LOG();
}
#endif

void setup()
{
    pinMode(PC13, OUTPUT);

    // for serial data and telemetry
    Serial3.begin(100000, SERIAL_8E2);

    // for debugging
    Serial.begin(115200);
    LOG(F("Start!!\n"));

    // make serial3 priority high
    nvic_irq_set_priority(Serial3.c_dev()->irq_num, 5);

    mRcvr = new RCRcvrERSkySerial();
    mRcvr->init();

#if SIMUL
    simul_setup();
#endif
}


void loop()
{
#if SIMUL
    simul_loop();
#else
    u32 ts = micros();

    if (ts - mLastTS > 500000) {
        mLed = !mLed;
        digitalWrite(PC13, mLed);
        mLastTS = ts;
    }

    if (mRcvr) {
        u32 proto = mRcvr->loop();

//        LOG("T:%4d R:%4d E:%4d A:%4d %4d %4d %4d %4d [%4d %4d]\n", mRcvr->getRC(0), mRcvr->getRC(1), mRcvr->getRC(2), mRcvr->getRC(3), mRcvr->getRC(4),
//            mRcvr->getRC(5), mRcvr->getRC(6), mRcvr->getRC(7), mRcvr->getRC(8), mRcvr->getRC(9), mRcvr->getRC(10));

        if (proto) {
            u8  func   = RFProtocol::getFunc(proto);
            u32 pureID = RFProtocol::getIDExceptFunc(proto);
            if (mSelProto != pureID) {
                initProtocol(pureID);
                mSelProto = pureID;
                if (mRFProto) {
                    // STM32 unique device ID
                    //  uint16 *idBase0 =  (uint16 *) (0x1FFFF7E8);
                    //  uint16 *idBase1 =  (uint16 *) (0x1FFFF7E8+0x02);
                    //  uint32 *idBase2 =  (uint32 *) (0x1FFFF7E8+0x04);
                    //  uint32 *idBase3 =  (uint32 *) (0x1FFFF7E8+0x08);
                    u32 *idBase3 = (u32*)(0x1FFFF7E8 + 0x08);
                    mRFProto->setControllerID(*idBase3);
                    mRFProto->init(func & FUNC_AUTO_BIND);
                }
            } else {
                if (mRFProto && (func & FUNC_BIND)) {
                    mRFProto->init(1);
                }
            }

            if (mRFProto) {
                u8  power = TXPOWER_10mW;
                if (func & FUNC_POWER_HI)
                    power = TXPOWER_150mW;
                mRFProto->setRFPower(power);
                LOG("RF Power : %d\n", power);
            }
        }
    }

    if (mRFProto) {
        mRFProto->injectControls(mRcvr->getRCs(), mRcvr->getChCnt());
        mRFProto->getTM().update();
    }
    DRAIN_LOG();
#endif
}
