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
#include "RFProtocol.h"
#include "RCRcvrERSkySerial.h"

typedef enum
{
    STATE_IDLE,
    STATE_BODY,
} STATE_T;

enum PROTOCOLS
{
	MODE_SERIAL = 0,		// Serial commands
	MODE_FLYSKY = 1,		// =>A7105
	MODE_HUBSAN = 2,		// =>A7105
	MODE_FRSKY = 3,			// =>CC2500
	MODE_HISKY = 4,			// =>NRF24L01
	MODE_V2X2 = 5,			// =>NRF24L01
	MODE_DSM2 = 6,			// =>CYRF6936
	MODE_DEVO =7,			// =>CYRF6936
	MODE_YD717 = 8,			// =>NRF24L01
	MODE_KN  = 9,			// =>NRF24L01
	MODE_SYMAX = 10,		// =>NRF24L01
	MODE_SLT = 11,			// =>NRF24L01
	MODE_CX10 = 12,			// =>NRF24L01
	MODE_CG023 = 13,		// =>NRF24L01
	MODE_BAYANG = 14,		// =>NRF24L01
	MODE_FRSKYX = 15,		// =>CC2500
	MODE_ESKY = 16,			// =>NRF24L01
	MODE_MT99XX=17,			// =>NRF24L01
	MODE_MJXQ=18,			// =>NRF24L01
	MODE_SHENQI=19,			// =>NRF24L01
	MODE_FY326=20,			// =>NRF24L01
	MODE_SFHSS=21			// =>CC2500
};

enum Flysky
{
	Flysky=0,
	V9X9=1,
	V6X6=2,
	V912=3
};
enum Hisky
{
	Hisky=0,
	HK310=1
};
enum DSM2{
	DSM2=0,
	DSMX=1
};
enum YD717
{
	YD717=0,
	SKYWLKR=1,
	SYMAX4=2,
	XINXUN=3,
	NIHUI=4
};
enum KN
{
	WLTOYS=0,
	FEILUN=1
};
enum SYMAX
{
	SYMAX=0,
	SYMAX5C=1
};
enum CX10
{
    CX10_GREEN = 0,
    CX10_BLUE=1,		// also compatible with CX10-A, CX12
    DM007=2,
	Q282=3,
	JC3015_1=4,
	JC3015_2=5,
	MK33041=6,
	Q242=7
};
enum CG023
{
    CG023 = 0,
    YD829 = 1,
    H8_3D = 2
};
enum MT99XX
{
	MT99	= 0,
	H7		= 1,
	YZ		= 2
};
enum MJXQ
{
	WLH08	= 0,
	X600	= 1,
	X800	= 2,
	H26D	= 3,
	E010	= 4
};

enum FRSKYX
{
	CH_16	= 0,
	CH_8	= 1,
};

struct tbl {
    u8  ersky;
    u8  chip;
    u8  proto;
};

static const PROGMEM struct tbl TBL_CNV[] = {
    { MODE_SERIAL, 0,           255 },
    { MODE_FLYSKY, TX_A7105,    RFProtocol::PROTO_A7105_FLYSKY },
    { MODE_HUBSAN, TX_A7105,    RFProtocol::PROTO_A7105_HUBSAN },
    { MODE_FRSKY,  TX_CC2500,   255   },
    { MODE_HISKY,  TX_NRF24L01, RFProtocol::PROTO_NRF24L01_HISKY },
    { MODE_V2X2,   TX_NRF24L01, RFProtocol::PROTO_NRF24L01_V2x2  },
    { MODE_DSM2,   TX_CYRF6936, RFProtocol::PROTO_CYRF6936_DSMX  },
    { MODE_DEVO,   TX_CYRF6936, RFProtocol::PROTO_CYRF6936_DEVO  },
    { MODE_YD717,  TX_NRF24L01, RFProtocol::PROTO_NRF24L01_YD717 },
    { MODE_KN,     TX_NRF24L01, 255 },
    { MODE_SYMAX,  TX_NRF24L01, RFProtocol::PROTO_NRF24L01_SYMAX },
    { MODE_SLT,    TX_NRF24L01, 255 },
    { MODE_CX10,   TX_NRF24L01, RFProtocol::PROTO_NRF24L01_CX10  },
    { MODE_CG023,  TX_NRF24L01, 255 },
    { MODE_BAYANG, TX_NRF24L01, 255 },
    { MODE_FRSKYX, TX_CC2500,   255 },
    { MODE_ESKY,   TX_CC2500,   255 },
    { MODE_MT99XX, TX_NRF24L01, 255 },
    { MODE_MJXQ,   TX_NRF24L01, RFProtocol::PROTO_NRF24L01_MJXQ  },
    { MODE_SHENQI, TX_NRF24L01, 255 },
    { MODE_FY326,  TX_NRF24L01, 255 },
    { MODE_SFHSS,  TX_CC2500,   255 },
};

u8 RCRcvrERSkySerial::getChCnt(void)
{
    return CH_CNT;
}

void RCRcvrERSkySerial::init(void)
{
    mState = STATE_IDLE;
    mProto    = 255;
    mSubProto = 255;
    mFinalProto = 255;
    mFinalSubProto = 255;
}

void RCRcvrERSkySerial::close(void)
{

}

static u8 sIdx = 0;
static u8 sRxPacket[MAX_PACKET_SIZE];
static u8 sRxPacketDone[MAX_PACKET_SIZE];
static volatile u8 sFlag = 0;

#define SET_FLAG_RX_DONE        sFlag |= BV(0)
#define CLR_FLAG_RX_DONE        sFlag &= ~BV(0)
#define IS_FLAG_RX_DONE         ((sFlag & BV(0)) != 0)

#define SET_FLAG_DO_NOT_UPDATE  sFlag |= BV(1)
#define CLR_FLAG_DO_NOT_UPDATE  sFlag |= BV(1)
#define IS_FLAG_DO_NOT_UPDATE   ((sFlag & BV(1)) != 0)

void RCRcvrERSkySerial::handleRX(u8 data)
{
   if (sIdx == 0) {
        if ((data & 0xfe) == 0x54) {
            sRxPacket[sIdx++] = data;
        }
    } else {
        sRxPacket[sIdx++] = data;
        if (sIdx >= MAX_PACKET_SIZE) {
            if (!IS_FLAG_RX_DONE) {
                memcpy(sRxPacketDone, sRxPacket, MAX_PACKET_SIZE);
                SET_FLAG_RX_DONE;
            }
            sIdx = 0;
        }
    }
}

u32 RCRcvrERSkySerial::loop(void)
{
    u32 ret = 0;

    if (IS_FLAG_RX_DONE) {
        ret = handlePacket(sRxPacketDone, MAX_PACKET_SIZE);
        CLR_FLAG_RX_DONE;
    }

    return ret;
}

#define get32(p) ( (*(p + 3) << 24) | (*(p + 2) << 16) | (*(p + 1) << 8) | *p )

u32 RCRcvrERSkySerial::handlePacket(u8 *data, u8 size)
{
    u32 ret = 0;
    u8  func = 0;
    s16 rc[CH_CNT];

    if (data[1] & 0x20) {       // check range
        func |= FUNC_RANGE;
    } else {
        func &= ~FUNC_RANGE;
    }

    if (data[1] & 0x40) {       // check autobind(0x40)
        func |= FUNC_AUTO_BIND;
    } else {
        func &= ~FUNC_AUTO_BIND;
    }

    if (data[1] & 0x80) {       // check bind(0x80)
        func |= FUNC_BIND;
    } else {
        func &= ~FUNC_BIND;
    }

    if (data[2] & 0x80) {       // low power
        func &= ~FUNC_POWER_HI;
    } else {
        func |= FUNC_POWER_HI;
    }

    // parsing channels
    u8 *p  = data + 3;
    s8 dec = -3;

    // 11 bit * 16 channel
    for (u8 i = 0; i < CH_CNT; i++) {
        dec += 3;
        if (dec >= 8) {
            dec -= 8;
            p++;
        }
        p++;

        u32 val = get32(p);
        val = (val >> dec) & 0x7ff;
        if (200 < val && val < 1850) {
            rc[i] = val;
        } else {
            return 0;
        }
    }

    for (u8 i = 0; i < CH_CNT; i++) {
        u16 val = constrain(rc[i], 200, 1845);
        sRC[i] =  map(val, 200, 1845, CHAN_MIN_VALUE, CHAN_MAX_VALUE);
    }

    // parsing protocols
    u8 proto  = data[1] & 0x1f;          // 5 bit
    u8 sub    = (data[2] >> 4) & 0x07;   // 3 bit
    u8 rxnum  = data[2] & 0x0f;          // 4 bit
    u8 option = data[3];

    if (proto != mProto || sub != mSubProto || func != mFunc) {
        mProto    = proto;
        mSubProto = sub;
        mFunc     = func;
        mProtoChCnt = 0;
    } else if (mProtoChCnt < 3) {
        mProtoChCnt++;
        if (mProtoChCnt == 3 && (proto != mFinalProto || sub != mFinalSubProto || func != mFinalFunc)) {
            mFinalProto    = proto;
            mFinalSubProto = sub;
            mFinalFunc     = func;

            struct tbl *t = NULL;
            for (u8  i = 0; i < sizeof(TBL_CNV) / sizeof(struct tbl); i++) {
                t = (struct tbl *)&TBL_CNV[i];
                if (t->ersky == proto)
                    break;
            }

            if (t == NULL) {
                LOG(F("NO PROTOCOL = %x %x %x [%x]\n"), proto, sub, func, ret);
            } else {
                if (t->proto != 255) {
                    ret = RFProtocol::buildID(t->chip, t->proto, sub, func);
                }
                LOG(F("FINAL PROTO ERSKY = %x %x %x [%x]\n"), proto, sub, func, ret);
            }
        }
    }

    return ret;
}
