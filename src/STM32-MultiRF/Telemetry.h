#ifndef _TELEMETRY_H_
#define _TELEMETRY_H_

#include "Common.h"
#include "Utils.h"

class Telemetry
{
#define MASK_VOLT       BV(0)
#define MASK_TEMP       BV(1)
#define MASK_RPM        BV(2)
#define MASK_RSSI       BV(3)
#define MASK_BARO_ALT   BV(4)
#define MASK_VELOCITY   BV(5)
#define MASK_GPS        BV(31)

public:
    Telemetry();
    ~Telemetry();

    struct gps {
        s32 latitude;
        s32 longitude;
        s32 altitude;
        u32 velocity;
        u32 time;
        u16 heading;
        u8  satcount;
    };

    inline void setVolt(u8 idx, u16 val, u16 div)   { mUpdateMask |= MASK_VOLT; mVolt[idx] = val * 100 / div; }
    inline u16  getVolt(u8 idx)                     { return mVolt[idx];                                      }

    inline void setTemp(u8 idx, u16 val)            { mUpdateMask |= MASK_TEMP;  mTemp[idx] = val; }
    inline u16  getTemp(u8 idx)                     { return mTemp[idx];}

    inline void setRPM(u8 idx, u16 val)             { mUpdateMask |= MASK_RPM;   mRPM[idx] = val;  }
    inline u16  getRPM(u8 idx)                      { return mRPM[idx]; }

    inline void setRSSI(u16 val)                    { mUpdateMask |= MASK_RSSI;  mRSSI = val;      }
    inline u16  getRSSI(void)                       { return mRSSI;     }

    inline void setBaroAlt(u16 val)                 { mUpdateMask |= MASK_BARO_ALT; mBaroAlt = val;}

    inline struct gps *getGPS(void)                 { return &mGPS;     }
    inline void  setLat(s32 lat)                    { mGPS.latitude = lat;              mUpdateMask |= MASK_GPS; }
    inline void  setLon(s32 lon)                    { mGPS.longitude = lon;             mUpdateMask |= MASK_GPS; }
    inline void  setAlt(s32 alt)                    { mGPS.altitude = alt;              mUpdateMask |= MASK_GPS; }
    inline void  setVel(s32 vel)                    { mGPS.velocity = vel;              mUpdateMask |= MASK_GPS; }
    inline void  setTime(u32 time)                  { mGPS.time = time;                 mUpdateMask |= MASK_GPS; }
    inline void  setHead(u16 head, u16 div)         { mGPS.heading = head * 10 / div;   mUpdateMask |= MASK_GPS; }
    inline void  setSatCnt(u8 sat)                  { mGPS.satcount = sat;              mUpdateMask |= MASK_GPS; }

    u8   buildRSSI(u8 *buf);
    u8   buildAltInfo(u8 *buf);
    u8   buildPowerInfo(u8 *buf);
    u8   buildGPSInfo(u8 *buf);
    u8   buildTMInfo(u8 *buf);

    void frameDSM(u8 rssi, u8 *buf, u8 size);
    void update(void);
    void clearMask(u32 mask)                        { mUpdateMask &= ~mask; }
    u8   isMasked(u32 mask)                         { return (mUpdateMask & mask) == mask; }

private:
    u32         mUpdateMask;
    u16         mVolt[3];
    u16         mTemp[4];
    u16         mRPM[3];
    u16         mRSSI;
    u16         mBaroAlt;
    u16         mBattCap;
    struct gps  mGPS;

    u8          mTeleBuf[20];
};
#endif
