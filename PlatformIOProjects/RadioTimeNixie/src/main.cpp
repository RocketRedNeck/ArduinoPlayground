#include <Arduino.h>

// See this for debug notes: https://docs.platformio.org/en/latest/plus/debug-tools/avr-stub.html
// Summary:
//      Add avr-debugger library to the project
//      Add the following to the plarform.ini
//          debug_port = SERIAL_PORT (where SERIAL_PORT is something like COM3, where arduino appears)
//          debug_tool = avr-stub
//      Add the GDB debug init to the setup() function
//          debug_init();
//      Include the following file
#include "avr8-stub.h"

// ****************************************************************************************************
//
// RadioTime
//
// Processes the Atomic Clock radio signal (WWVB) and displays time on Nixie via shift register and
// bcd/dec decoder
// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
// Copyright (c) 2021 - Michael Kessel, Rocket RedNeck
// RocketRedNeck hereby grants license for others to copy and modify this source code for
// whatever purpose other's deem worthy as long as RocketRedNeck is given credit where
// where credit is due and you leave RocketRedNeck out of it for all other nefarious purposes.
// ****************************************************************************************************

#include "MicroTimer.h"
#include <RTClib.h>
#include <time.h>

// ----------------------------------------------------------------------------------------------------
// Some constants are herein #defined or enumerated rather than instantiated as const to avoid using
// memory unless the values are actually used in the code below. Mileage depends on quality of linker
// and rather than tempt fate we just revert to old habits.
// ----------------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------------
// Pin assignments - enumeration simply to group them nicely
// NOTE: built-in and PWMs can be allocated for any purpose, and as such the comments simply keep
// thing visible to readers so we understand any potential conflicts. Read the specs to see what
// other functions overlap the
// ----------------------------------------------------------------------------------------------------
enum DigitalPins
{
    DP_RX = 0, // Arduino built in function
    DP_TX = 1, // Arduino built in function

    DP_SR_OE_LOW        = 2,    // ~OE,    Pin 13 on 74HC595
    DP_SR_RCLK          = 3,    // RCLK,   Pin 12 on 74HC595, PWM
    DP_SR_SRCLR_LOW     = 4,    // ~SRCLR, Pin 10 on 74HC595
    DP_SR_SRCLK         = 5,    // SRCLK,  Pin 11 on 74HC595, PWM
    DP_SR_DATA          = 6,    // SER,    Pin 14 on 74HC595, PWM

    DP_WWVB_INVERTED    = 7,

    DP_UNUSED8          = 8,
    DP_UNUSED9          = 9, // PWM
    DP_UNUSED10         = 10, // PWM
    DP_UNUSED11         = 11,   // PWM
    DP_UNUSED12         = 12,

    DP_BUILT_IN_LED     = 13, // Arduino built in function

    DP_MAX_NUMBER_OF_PINS,
    DP_BOGUS // A value to recognize when none of the above is indicated.
};

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
enum AnalogPins
{
    AP_RANDOM_SEED = 0, // See notations where this is used, below
    AP_UNUSED1 = 1,
    AP_UNUSED2 = 2,
    AP_UNUSED3 = 3,
    AP_UNUSED4 = 4,
    AP_UNUSED5 = 5,

    AP_MAX_NUMBER_OF_PINS,
    AP_BUGUS

};

#define EVER (;;)
#define DIM(x) (sizeof(x) / sizeof(x[0]))
#define LAST(x) (DIM(x) - 1)

// ----------------------------------------------------------------------------------------------------
// Finally! The setup routine runs once at power-on/reset:
// ----------------------------------------------------------------------------------------------------

RTC_DS1307 rtc;
bool rtcPresent = false;


// Storage for 60 seconds of data
int8_t data[60];
uint8_t index = 0;

void updateShiftRegister(byte msw, byte mid, byte lsw)
{
    digitalWrite(DP_SR_RCLK, LOW);
    shiftOut(DP_SR_DATA, DP_SR_SRCLK, MSBFIRST, msw);
    shiftOut(DP_SR_DATA, DP_SR_SRCLK, MSBFIRST, mid);
    shiftOut(DP_SR_DATA, DP_SR_SRCLK, MSBFIRST, lsw);
    digitalWrite(DP_SR_RCLK, HIGH);
}

void setup()
{
    // initialize GDB stub
    //debug_init();

    Serial.begin(115200);


    pinMode(DP_BUILT_IN_LED,    OUTPUT);

    pinMode(DP_SR_OE_LOW,       OUTPUT);
    pinMode(DP_SR_RCLK,         OUTPUT);
    pinMode(DP_SR_SRCLR_LOW,    OUTPUT);
    pinMode(DP_SR_SRCLK,        OUTPUT);
    pinMode(DP_SR_DATA,         OUTPUT);

    pinMode(DP_WWVB_INVERTED,   INPUT_PULLUP);

    digitalWrite(DP_SR_OE_LOW,      0);
    digitalWrite(DP_SR_SRCLR_LOW,   1);

    for (int i = 9; i >= 0; --i)
    {
        updateShiftRegister((i << 4) | i, (i << 4) | i, (i << 4) | i);
        delay(500);
    }

    rtcPresent = rtc.begin();
    if (!rtcPresent)
    {
        long i = 1;
        for (;;)
        {
            updateShiftRegister((i << 4) | i, (i << 4) | i, (i << 4) | i);
            delay(500);
            i ^= 1;
        }
    }

    if (rtcPresent && !rtc.isrunning())
    {
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
        rtc.adjust(DateTime(2000, 1, 1, 0, 0, 0));
    }

    for (unsigned i = 0; i < sizeof(data) / sizeof(data[0]); ++i)
    {
        data[i] = -1;
    }
}

// ----------------------------------------------------------------------------------------------------
// The loop routine runs over and over again forever:
// ----------------------------------------------------------------------------------------------------

// Pulse detector timer and states
MicroTimer pulseTimer;
enum PulseState
{
    WAITING_FOR_RISING,
    WAITING_FOR_FALLING
};
PulseState pulseState = WAITING_FOR_RISING;

// Pulse parsing states
long long pw_ms = 0;
long long tops_ms = 0; // time of pulse start
enum State
{
    WAITING_FOR_END,
    WAITING_FOR_START,
    WAITING_FOR_BIT,
    WAITING_FOR_MARK
};
State state = WAITING_FOR_END;

MicroTimer timer;
long long topOfMinute_ms = 0;
long long nextBitWindow_ms = 0;
long long nextMarkWindow_ms = 0;
long long nextMinuteWindow_ms = 0;

const unsigned long MARK_LOW_MS = 700;
const unsigned long MARK_DELTA_MS = 9000;

const unsigned long BIT_0_LOW_MS = 100;
const unsigned long BIT_0_HIGH_MS = 300;
const unsigned long BIT_1_LOW_MS = 400;
const unsigned long BIT_1_HIGH_MS = 600;

// Open windows about this much earlier
// than the precise alignment of the actual
// bit/mark time; this allows us some slop
// on detecting when the next transition should
// begin.
const unsigned long MARGIN_MS = 5;

int seconds = 0;
int minutes = 0;
int hours = 0;
int daynum = 0;
int day = 0;
int month = 0;
int year = 0;

MicroTimer displayTimer;
long long nextDisplay_ms = 1000;

bool detectPulse(void)
{
    bool pulseDetected = false;

    // Active low output from OUT (~OUT is active high)... yeah, that is backwards, live with it.
    // The reason is the radio signal power drops 17 dB to signal the bit (at 1 Hz)
    // 200 ms dropout = 0
    // 500 ms dropout = 1
    // 800 ms dropout = mark (end of 10 second interval and first mark at minute)
    // Note: The end/start of minute is two (2) 800 ms marks in a row.

    // The problem with pulseIn or pulseInLong is that they block the
    // loop() from running until the both edges or timeout occurs.
    // We we really, really want is to keep polling for the pulse while
    // doing other things
    switch (pulseState)
    {
    case WAITING_FOR_RISING:
        if (HIGH == digitalRead(DP_WWVB_INVERTED))
        {
            pulseTimer.mark();
            pulseState = WAITING_FOR_FALLING;
        }
        else if (pulseTimer.read_ms() > 2000)
        {
            pulseTimer.mark();
            pw_ms = 0;
            pulseState = WAITING_FOR_RISING;
            pulseDetected = true;
        }
        break;
    case WAITING_FOR_FALLING:
        if (LOW == digitalRead(DP_WWVB_INVERTED))
        {
            pw_ms = pulseTimer.read_ms();
            tops_ms = timer.read_ms() - pw_ms;
            pulseState = WAITING_FOR_RISING;
            pulseDetected = true;
        }
        else if (pulseTimer.read_ms() > 2000)
        {
            pulseTimer.mark();
            pw_ms = 0;
            pulseState = WAITING_FOR_RISING;
            pulseDetected = true;
        }
        break;
    default:
        pulseState = WAITING_FOR_RISING;
        break;
    };

    return pulseDetected;
}

bool parsePulses(void)
{
    bool decodeReady = false;

    switch (state)
    {
    case WAITING_FOR_END:
        if (pw_ms > MARK_LOW_MS)
        {
            state = WAITING_FOR_START;

            // Decode time and display
            ++index;
            decodeReady = true;
        }
        break;
    case WAITING_FOR_START:
        if (pw_ms > MARK_LOW_MS)
        {
            displayTimer.mark();

            topOfMinute_ms = tops_ms;
            nextMinuteWindow_ms = topOfMinute_ms + 60000 - MARGIN_MS;
            nextMarkWindow_ms = topOfMinute_ms + 9000 - MARGIN_MS;
            nextBitWindow_ms = topOfMinute_ms + 1000 - MARGIN_MS;
            state = WAITING_FOR_BIT;
            index = 0;
            seconds = 0;
            nextDisplay_ms = 0;
            data[index] = 1;
            ++index;
        }
        else
        {
            state = WAITING_FOR_END;
        }
        break;
    case WAITING_FOR_BIT:
        if (timer.read_ms() >= nextBitWindow_ms)
        {
            nextBitWindow_ms += 1000;
            if ((pw_ms > BIT_1_LOW_MS) && (pw_ms < BIT_1_HIGH_MS))
            {
                // Record 1
                data[index] = 1;
            }
            else if ((pw_ms > BIT_0_LOW_MS) && (pw_ms < BIT_0_HIGH_MS))
            {
                // Record 0
                data[index] = 0;
            }
            else
            {
                // Invalid bit
                data[index] = -1;
            }
            ++index;
            if (0 == ((index + 1) % 10))
            {
                if (59 == index)
                {
                    state = WAITING_FOR_END;
                }
                else
                {
                    state = WAITING_FOR_MARK;
                }
            }
        }
        else
        {
            // Extraneous pulse received before next pulse window
            // means previous pulse that was recorded may have
            // been chopped. Go back and invalidate it.
            data[index - 1] = -1;
        }
        break;
    case WAITING_FOR_MARK:
        if (timer.read_ms() >= nextMarkWindow_ms)
        {
            if (pw_ms > MARK_LOW_MS)
            {
                data[index] = 1;
            }
            else
            {
                // Invalid mark
                data[index] = -1;
            }
            ++index;
            nextMarkWindow_ms += 10000;
            nextBitWindow_ms += 1000;
            state = WAITING_FOR_BIT;
        }
        else
        {
            // Extraneous pulse received before next pulse window
            // means previous pulse that was recorded may have
            // been chopped. Go back and invalidate it.
            data[index - 1] = -1;
        }
        break;

    default:
        state = WAITING_FOR_END;
    };

    return decodeReady;
}

// When deciding if the RTC should be adjusted
// require at least N samples of the WWVB signal
// to match (i.e., down to the minute)
const size_t NUM_DT_SAMPLES = 3;
DateTime sampleDateTime[NUM_DT_SAMPLES];
unsigned dtIndex = 0;

unsigned timeAdjusted_count = 0;

void decodeTime(void)
{
    if (data[1] >= 0 &&
        data[2] >= 0 &&
        data[3] >= 0 &&
        data[5] >= 0 &&
        data[6] >= 0 &&
        data[7] >= 0 &&
        data[8] >= 0 &&

        data[12] >= 0 &&
        data[13] >= 0 &&
        data[15] >= 0 &&
        data[16] >= 0 &&
        data[17] >= 0 &&
        data[18] >= 0 &&

        data[22] >= 0 &&
        data[23] >= 0 &&
        data[25] >= 0 &&
        data[26] >= 0 &&
        data[27] >= 0 &&
        data[28] >= 0 &&
        data[30] >= 0 &&
        data[31] >= 0 &&
        data[32] >= 0 &&
        data[33] >= 0 &&

        data[45] >= 0 &&
        data[46] >= 0 &&
        data[47] >= 0 &&
        data[48] >= 0 &&
        data[50] >= 0 &&
        data[51] >= 0 &&
        data[52] >= 0 &&
        data[53] >= 0)
    {
        minutes = 0;
        minutes += (data[1] ? 40 : 0);
        minutes += (data[2] ? 20 : 0);
        minutes += (data[3] ? 10 : 0);
        minutes += (data[5] ? 8 : 0);
        minutes += (data[6] ? 4 : 0);
        minutes += (data[7] ? 2 : 0);
        minutes += (data[8] ? 1 : 0);

        hours = 0;
        hours += (data[12] ? 20 : 0);
        hours += (data[13] ? 10 : 0);
        hours += (data[15] ? 8 : 0);
        hours += (data[16] ? 4 : 0);
        hours += (data[17] ? 2 : 0);
        hours += (data[18] ? 1 : 0);

        daynum = 0;
        daynum += (data[22] ? 200 : 0);
        daynum += (data[23] ? 100 : 0);
        daynum += (data[25] ? 80 : 0);
        daynum += (data[26] ? 40 : 0);
        daynum += (data[27] ? 20 : 0);
        daynum += (data[28] ? 10 : 0);
        daynum += (data[30] ? 8 : 0);
        daynum += (data[31] ? 4 : 0);
        daynum += (data[32] ? 2 : 0);
        daynum += (data[33] ? 1 : 0);

        year = 2000;
        year += (data[45] ? 80 : 0);
        year += (data[46] ? 40 : 0);
        year += (data[47] ? 20 : 0);
        year += (data[48] ? 10 : 0);
        year += (data[50] ? 8 : 0);
        year += (data[51] ? 4 : 0);
        year += (data[52] ? 2 : 0);
        year += (data[53] ? 1 : 0);

        if (hours != 0 && minutes != 0 && daynum != 0 && year != 0)
        {
            hours -= 7; // Tucson offset
            if (hours < 0)
            {
                hours += 24;
                daynum -= 1;
            }
        }

        tm t;
        t.tm_sec = 0;
        t.tm_min = 0;
        t.tm_hour = 0;
        t.tm_mday = 1;
        t.tm_mon = 1;
        t.tm_year = year - 1900;
        time_t ref = mktime(&t);
        time_t dayref = ref + (daynum - 1) * 86400;
        gmtime_r(&dayref, &t);
        day = t.tm_mday;
        month = t.tm_mon;

        sampleDateTime[dtIndex] = DateTime(year, month, day, hours, minutes, 59);
        ++dtIndex;

        bool adjustTime = false;
        size_t goodSampleCount = 1;
        if (0 == dtIndex % NUM_DT_SAMPLES)
        {
            // All samples have been collected
            // Check for consecutive minutes
            for (size_t i = 1; i < NUM_DT_SAMPLES; ++i)
            {
                TimeSpan dt = sampleDateTime[i] - sampleDateTime[i-1];
                if (dt.minutes() >= 1 && dt.minutes() < 2)
                {
                    ++goodSampleCount;
                }
            }

            adjustTime = (NUM_DT_SAMPLES == goodSampleCount);

            dtIndex = 0;

        }

        if (rtcPresent && adjustTime)
        {
            // The time at the last sample is just completed.
            // To the nearest second we are at the end of the minute
            // that has just been reported
            rtc.adjust(DateTime(year, month, day, hours, minutes+1, 0));
            ++timeAdjusted_count;
            if (timeAdjusted_count == 100)
            {
                timeAdjusted_count = 0;
            }
        }
    }
    else
    {
        hours = 0;
        minutes = 0;
        daynum = 0;
        day = 0;
        month = 0;
        year = 0;
    }
}

const char* MONTH_NAME[] =
{
    "???",
    "Jan",
    "Feb",
    "Mar",
    "Apr",
    "May",
    "Jun",
    "Jul",
    "Aug",
    "Sep",
    "Oct",
    "Nov",
    "Dec"
};
void updateDisplay(void)
{
    // Display output (either LCD or Nixie)
    if (displayTimer.read_ms() >= nextDisplay_ms)
    {
        if (rtcPresent && rtc.isrunning())
        {
            DateTime now = rtc.now();
            int yr = now.year();
            int mon = now.month();
            int dy = now.day();
            int hr = now.hour();
            int min = now.minute();
            int sec = now.second();

            nextDisplay_ms += 1000;

            // Convert time to BCD
            long x = hr;
            x = 100*x + min;
            x = 100*x + sec;

            //Display date every minute for 10 seconds
            if (sec < 10)
            {
                x = yr;
                x = 100*x + mon;
                x = 100*x + dy;
            }

            long bcd = 0;
            int shift = 0;
            while (x > 0) 
            {
                bcd |= (x % 10) << (shift++ << 2);
                x /= 10;
            }
            Serial.println(bcd,HEX);
            updateShiftRegister(bcd >> 16, (bcd >> 8) & 0xFF, bcd & 0xFF);
        }
    }
}

void loop()
{
    if (detectPulse())
    {
        if (parsePulses())
        {
            decodeTime();
        }
    }

    updateDisplay();
}
