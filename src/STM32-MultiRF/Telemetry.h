#ifndef _TELEMETRY_H_
#define _TELEMETRY_H_

#include "Common.h"
#include "Utils.h"

class Telemetry
{
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

    inline void setVolt(u8 idx, u16 val, u16 div)       { mVolt[idx] = val * 100 / div; }
    inline u16  getVolt(u8 idx)                         { return mVolt[idx];            }

    inline void setTemp(u8 idx, u16 val)    { mTemp[idx] = val; }
    inline u16  getTemp(u8 idx)            { return mTemp[idx];}

    inline void setRPM(u8 idx, u16 val)    { mRPM[idx] = val;  }
    inline u16  getRPM(u8 idx)             { return mRPM[idx]; }

    inline void setRSSI(u16 val)            { mRSSI = val;      }
    inline u16   getRSSI(void)              { return mRSSI;     }

    inline struct gps *getGPS(void)        { return &mGPS;     }

    inline void  setLat(s32 lat)           { mGPS.latitude = lat; }
    inline void  setLon(s32 lon)           { mGPS.longitude = lon; }
    inline void  setAlt(s32 alt)           { mGPS.altitude = alt; }
    inline void  setVel(s32 vel)           { mGPS.velocity = vel; }
    inline void  setTime(u32 time)         { mGPS.time = time; }
    inline void  setHead(u16 head)         { mGPS.heading = head; }
    inline void  setSatCnt(u8 sat)         { mGPS.satcount = sat; }

    void showInfo(void)                    {
        LOG(F("VOLT - V1:%d, V2:%d, V3:%d\n"), mVolt[0], mVolt[1], mVolt[2]);
        LOG(F("TEMP - T1:%d, T2:%d, T3:%d, T4:%d\n"), mTemp[0], mTemp[1], mTemp[2], mTemp[3]);
        LOG(F("RPM  - R1:%d, R2:%d, R3:%d\n"), mRPM[0], mRPM[1], mRPM[2]);
    }

    void frameFRSky(u8 *buf, u8 size);
    void frameDSM(u8 type, u8 *buf, u8 size);
    void update(void);
    u8   buildRSSI(u8 *buf);
    u8   buildAltInfo(u8 *buf);
    u8   buildPowerInfo(u8 *buf);

private:
    u16  mVolt[3];
    u16  mTemp[4];
    u16  mRPM[3];
    u16  mRSSI;
    u16  mBattCap;
    struct gps mGPS;

    u8  mBuffer[20];
};
#endif
