/*
* Copyright (c) Ian Millington 2003-2006. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_DEMO_TIMING_HPP
#define PEGASUS_DEMO_TIMING_HPP

struct TimingData
{
    unsigned frameNumber;
    unsigned lastFrameTimestamp;
    unsigned lastFrameDuration;
    unsigned long lastFrameClockstamp;
    unsigned long lastFrameClockTicks;
    bool isPaused;
    double averageFrameDuration;
    float fps;

    static TimingData& Get();
    static void Update();
    static void Init();
    static void Deinit();
    static unsigned GetTime();
    static unsigned long GetClock();

private:
    TimingData()
    {
    }

    TimingData(const TimingData&)
    {
    }

    TimingData& operator=(const TimingData&);
};

#endif // PEGASUS_DEMO_TIMING_HPP