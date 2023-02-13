/* Maytag washer timer implementation.
* This sketch runs on a Sparkfun Pro Micro and matches the behavior of the Maytag mechanical
* timer LAT8504. There are 7 different relay outputs which correspond (roughly) the cams on
* the mechanical timer. There are two input pins that sense the position of two switches
* in the washer: the lid-closed switch and the tub-full switch. The mechanical switches
* retain (most of) their original functions, but this sketch also needs to know their
* positions in order to advance the cycle timer, or wait for something to happen, i.e. for
* the tub to fill, or the lid to close.
*/
// Serial is debug diagnostics on USB
// Serial1 is the gen4-Ulcd-24PT
#include <avr/wdt.h>
#include <SparkFun_Qwiic_Relay.h>
#include <genieArduino.h>
#include "MaytagCycleDefinitions.h"

// I built the first iteration with two qwiic relay boards, but had a failure in one of them
// and found it easier to replace it with discrete relays. The qwiic codes are under
// conditional compile on MECH_RELAY_TYPE
#define MECH_RELAY_TYPE 1 // 0 is qwiic, 1 is digital pin per output

// All these conditional compile directives are 0 for production program
#define INTERACTIVE_DEBUG_MODE 0
#define SIMULATE_LIDandFULL 0

#define DIM(x) (sizeof(x)/sizeof(x[0]))

class RelayPerPin {
    // This class has mostly the same methods as Sparkfun's qwiic relay
    // class for ease in switching this sketch between the two different hardware versions.
public:
    RelayPerPin(int numRelayPins, const int pinsAssigned[])
        : numRelays(numRelayPins)
        , pins(pinsAssigned)
    { }
    int begin() 
    {
        for (int i = 0; i < numRelays; i++)
        {
            digitalWrite(pins[i], LOW);
            pinMode(pins[i], OUTPUT);
        }
        return 1;
    }
    void turnAllRelaysOff()
    {
        for (int i = 0; i < numRelays; i++)
            digitalWrite(pins[i], LOW);
    }
    void turnRelayOff(uint8_t which)
    {
        digitalWrite(pins[which-1], LOW);
    }
    void turnRelayOn(uint8_t which)
    {
        digitalWrite(pins[which-1], HIGH);
    }
private:
    const int numRelays;
    const int *pins;
};

static const int HISTORY_LENGTH = 100;
class NoisyPollinPin {
public:
    NoisyPollinPin(int pin) 
        : pinAssignment(pin)
        , lastUpdate(0)
        , next(0)
        , history({})
    {
    }
    void setup()
    {
        pinMode(pinAssignment, INPUT_PULLUP);
    }
    void update(unsigned long now)
    {
        if (now == lastUpdate)
            return; // at most one sample per msec
        lastUpdate = now;
        history[next++] = digitalRead(pinAssignment) == LOW;
        if (next >= HISTORY_LENGTH)
            next = 0;
    }
    bool value() const
    {
        uint16_t ones(0);
        for (int i = 0; i < HISTORY_LENGTH; i++)
            if (history[i])  ones += 1;
        return ones >= HISTORY_LENGTH / 4;
    }
private:
    bool history[HISTORY_LENGTH];
    uint16_t next;
    unsigned long lastUpdate;
    const int pinAssignment;
};

namespace {
    const uint8_t MECH_RELAY_I2CADDR = 0x6d;
    const uint8_t SS_RELAY_IC2ADDR = 0X8;
    const uint32_t SERIAL_BAUD_RATE = 38400;

    enum SSRelayAssign_t { RLYSS_MAIN_LINE = 1,
        RLYSS_MOTOR = 2,
        RLYSS_COLD_WATER_FILL = 3,
        RLYSS_HOT_WATER_FILL = 4};
    enum MechRelayAssign_t { RLYM_DIRECTION1 = 1,
        RLYM_DIRECTION2 = 2,
        RLYM_SOAKLIGHT = 3};

    const uint8_t PIN_LIDCLOSED_SENSE = 15;        
    const uint8_t PIN_EMPTYFULL_SENSE = 16;    

    const int PIN_RLYM_DIRECTION1 = 6;
    const int PIN_RLYM_DIRECTION2 = 7;
    const int PIN_RLYM_SOAK = 8;

#if INTERACTIVE_DEBUG_MODE==0
    // The Pro Micro has a couple of LEDs that are used by the USB serial, but in this compiled mode, there is no USB
    const int PIN_LID_LED = 30;
    const int PIN_EMPTYFULL_LED = 17;
#endif

    const uint8_t PIN_WATERTEMP3 = 4;
    const uint8_t PIN_WATERTEMP4 = 5;

    const uint8_t PIN_RESETLCD = A3;

    const int MSEC_AC_SENSE_ASSUME_OFF = 100;

    enum {MAIN_KNOB_STOPPED, MAIN_KNOB_RUNNING}
        gMainKnobState = MAIN_KNOB_STOPPED;

    const uint8_t MOTOR_DIRECTION_RELAY_SETTLE_MSEC = 50;

    // The design for the genie display has 17 "settings." This enumeration must match what the genie has
    enum { NUM_GENIE_DISPLAY_CYCLE_SETTINGS = 17, GENIE_DISPLAY_CYCLE_START = 2, GENIE_LOWEST_TIME_IDX = 8 };
    // Those 17 settings correspond rougly to the labeling on the washer's front panel around its original timer.
    const uint16_t UserCycleToTime[NUM_GENIE_DISPLAY_CYCLE_SETTINGS] PROGMEM =
    {    // the genie object starts at angle 180.
        // the dryer display starts at angle 0
        2867, // spin

        3331, // SOAK
        3782, // REGULAR
        3782 + 180, // 9 min
        3782 + 2 * 180, // 6 min
        3782 + 3 * 180, // 3 min
        4766, // rinse
        4955, // spin

        // IDX = 8 = GENIE_LOWEST_TIME_IDX
        268, // PERMANENT PRESS
        268 + 180, // 9 min
        268 + 2 * 180, // 6 min
        268 + 3 * 180, // 3 min
        990, // cool down rinse
        1260, // final rinse
        1350, // spin

        1889, // DELICATES
        2702, // rinse
    };

    enum { INFINITE_SOAK_INDEX = 3, NUM_STOP_TIMES = 5 };
    const uint16_t StopTimes[NUM_STOP_TIMES] =
    {
        0,
        1621,  // Permanent press stops
        3063,    // Delicates stops
        3515, // INFINITE_SOAK_INDEX
        5316, // Regular stops (also is time zero)
    };

    void genieEvents();
    void checkAdvanceTimer(int32_t elapsedMsec, bool advance);
    void applyStateToRelays();
    void checkUpdateLEDcounter(int32_t elapsedMsec);
    void checkUpdateCycleDisplay();
    void checkUpdateMsecToIdx();

#if MECH_RELAY_TYPE==0
    Qwiic_Relay mechRelays(MECH_RELAY_I2CADDR);
#elif MECH_RELAY_TYPE==1
    const int mechRelayPins[] = 
    {   // order must match enum MechRelayAssign_t
        PIN_RLYM_DIRECTION1,
        PIN_RLYM_DIRECTION2,
        PIN_RLYM_SOAK
    };
    RelayPerPin mechRelays(DIM(mechRelayPins), mechRelayPins);
#else
#error "Unknown MECH_RELAY_TYPE"
#endif
    Qwiic_Relay ssRelays(SS_RELAY_IC2ADDR);

    uint16_t gTimerSequenceSeconds = 0; 
    CycleDefinition gCurrentCycleDef;

    Genie genie;
    const int GENIE_USER_BUTTON1_ON_FORM0 = 0;
    const int GENIE_USER_BUTTON2_ON_FORM0 = 1;

    const int GENIE_USER_BUTTON1_ON_FORM1 = 2;
    const int GENIE_USER_BUTTON2_ON_FORM1 = 3;

#if SIMULATE_LIDandFULL != 0
    bool lidClosed = false;
    bool tubFull = false;
#endif

#if INTERACTIVE_DEBUG_MODE
    bool quickTime = false;
#endif

    bool userChangedCycleWhileStopped = true;

    bool lidIsClosed;
    bool tubIsFull;

    NoisyPollinPin lidClosedPin(PIN_LIDCLOSED_SENSE);
    NoisyPollinPin tubFullPin(PIN_EMPTYFULL_SENSE);
}

void setup()
{
#if (INTERACTIVE_DEBUG_MODE)
    delay(5000);    // pro micro usb enumeration takes a long time on Windows 10. give it a chance
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println("Maytag LAT8504AAE washing machine timer");
#endif

    Wire.begin();
    auto mechStart = mechRelays.begin();
    auto ssStart = ssRelays.begin();
#if (INTERACTIVE_DEBUG_MODE)
    if (!mechStart)
        Serial.println("Mechanical relay i2c init failed!");
    if (!ssStart)
        Serial.println("Solid state relay i2c init failed!");
#endif
    ssRelays.turnAllRelaysOff();
    mechRelays.turnAllRelaysOff();

    // start up genie LCD************************
    Serial1.begin(200000);
    genie.Begin(Serial1);
    genie.AttachEventHandler(genieEvents);
    digitalWrite(PIN_RESETLCD, LOW);
    pinMode(PIN_RESETLCD, OUTPUT);
    delay(100);
    pinMode(PIN_RESETLCD, INPUT);
    delay(3500); // wait for genie
    genie.WriteContrast(10);
    genie.WriteObject(GENIE_OBJ_4DBUTTON, 0, 1);
    genie.WriteObject(GENIE_OBJ_4DBUTTON, 1, 1);
    genie.WriteObject(GENIE_OBJ_ROTARYSW, 0, GENIE_DISPLAY_CYCLE_START);
    genie.WriteObject(GENIE_OBJ_STRINGS, 0, GENIE_DISPLAY_CYCLE_START);

    pinMode(PIN_WATERTEMP3, INPUT_PULLUP);
    pinMode(PIN_WATERTEMP4, INPUT_PULLUP);

#if INTERACTIVE_DEBUG_MODE==0
    pinMode(PIN_LID_LED, OUTPUT);
    pinMode(PIN_EMPTYFULL_LED, OUTPUT);
#endif
    
    lidClosedPin.setup();
    tubFullPin.setup();

    memcpy_P(&gCurrentCycleDef, &MaytagLAT8504[0], sizeof(CycleDefinition));

    // watch dog timer. If this sketch fails to run, then reset the Arduino, which turns off the washing machine.
    wdt_enable(WDTO_8S);
}

void loop()
{
    wdt_reset();
#if (INTERACTIVE_DEBUG_MODE)
    // only when built in this special debug mode do we take input
    while (Serial.available())
    {
        char n = Serial.read();
        if (isalpha(n))
            n = toupper(n);
        switch (n)
        {
#if SIMULATE_LIDandFULL != 0
        case 'L':
            lidClosed = false;
            Serial.println("lid is open");
            break;
        case 'C':
            lidClosed = true;
            Serial.println("lid is closed");
            break;
        case 'F':
            tubFull = true;
            lidClosed = true;
            Serial.println("tub is full");
            break;
        case 'E':
            tubFull = false;
            Serial.println("tub is empty");
            break;
#endif

        case 'Q':
            quickTime = true;
            Serial.println("time is 10x");
            break;
        case 'S':
            quickTime = false;
            Serial.println("time is real time");
        }
    }
#endif

    static unsigned long prevNow;
    unsigned long now = millis();
    int32_t elapsed = now - prevNow;
    prevNow = now;

    genie.DoEvents();
    bool isEnabled = gMainKnobState != MAIN_KNOB_STOPPED;
    if (!isEnabled)
    {
        auto wasOn = ssRelays.getState(RLYSS_MAIN_LINE);
        ssRelays.turnAllRelaysOff(); 
        if (wasOn)
            delay(MOTOR_DIRECTION_RELAY_SETTLE_MSEC); // when turning off, let solid state finish first.
        mechRelays.turnAllRelaysOff(); 
        checkAdvanceTimer(elapsed, false); // keeps the tub full LED up to date.
        // the lid closed LED can't be updated with RLYSS_MAIN_LINE off (see the circuit diagram)
        return;
    }

    checkAdvanceTimer(elapsed, true);
    checkUpdateLEDcounter(elapsed);
    checkUpdateMsecToIdx();
    applyStateToRelays();
}

namespace {
    uint8_t whichCycleStep(uint16_t timerSequenceSeconds)
    {    // binary search the step number from the number of seconds into the timer cycle
        uint8_t lowest = 0;
        uint8_t highest = NUM_WASHER_TIMER_STEPS - 1;
        for (;;)
        {
            uint8_t whichStep = (highest + lowest) >> 1;
            uint16_t stepStartSeconds = pgm_read_word_near(&MaytagLAT8504[whichStep].timeStampSeconds);
            int16_t compare = stepStartSeconds - timerSequenceSeconds;
            if (compare == 0)
                return whichStep;
            else if (compare < 0)
                lowest = whichStep;
            else
                highest = whichStep;
            uint8_t diff = highest - lowest;
            if (diff <= 1)
            {   // down to only two candidates
                stepStartSeconds = pgm_read_word_near(&MaytagLAT8504[highest].timeStampSeconds);
                compare = stepStartSeconds - timerSequenceSeconds;
                if (compare > 0)
                    return lowest;
                else
                    return highest;
            }
        }
    }

    void transitionOffToOn(uint16_t genieObjValue)
    {
        if (userChangedCycleWhileStopped)
        {
            gTimerSequenceSeconds =  pgm_read_word_near(&UserCycleToTime[genieObjValue]);
#if (INTERACTIVE_DEBUG_MODE)
            Serial.print("transitionOffToOn idx=");
            Serial.print(genieObjValue);
            Serial.print(" seconds=");
            Serial.println(gTimerSequenceSeconds);
            Serial.flush();
#endif
        }
        gMainKnobState = MAIN_KNOB_RUNNING;
        ssRelays.turnRelayOn(RLYSS_MAIN_LINE);
        // With RLYSS_MAIN_LINE on, poll the lid closed sensor for 100msec to get its state right.
        auto now = millis();
        while (millis() - now < MSEC_AC_SENSE_ASSUME_OFF)
            checkAdvanceTimer(0,0);
    }

    uint8_t secTogenieIdx(uint16_t secs)
    {    
        if (secs < pgm_read_word_near(&UserCycleToTime[GENIE_LOWEST_TIME_IDX]))
            secs += TIMER_FULL_CYCLE; // modular on TIMER_FULL_CYCLE
        int i = 0;
        for (int j = 0; j < NUM_GENIE_DISPLAY_CYCLE_SETTINGS; j += 1)
        {
            i = GENIE_LOWEST_TIME_IDX - j - 1;
            if (i < 0)
                i += NUM_GENIE_DISPLAY_CYCLE_SETTINGS;
            uint16_t secondsStart = pgm_read_word_near(&UserCycleToTime[i]);
            if (secs >= secondsStart)
                break;
        }
        return i;
    }

    void transitionOnToOff()
    {
        gMainKnobState = MAIN_KNOB_STOPPED;
        userChangedCycleWhileStopped = false;
        // move rotary switch to nearest cycle corresponding to gTimerSequenceSeconds
        int i = secTogenieIdx(gTimerSequenceSeconds);
        genie.WriteObject(GENIE_OBJ_ROTARYSW, 0, i);
#if (INTERACTIVE_DEBUG_MODE)
        Serial.print("transitionOnToOff sec=");
        Serial.print(gTimerSequenceSeconds);
        Serial.print(" idx="); Serial.print(i);
        Serial.println();
        Serial.flush();
#endif
    }

    void genieEvents()
    {
        genieFrame Event;
        if (!genie.DequeueEvent(&Event))
            return;
#if (INTERACTIVE_DEBUG_MODE)
        Serial.print("Event cmd: ");
        Serial.print((int)Event.reportObject.cmd);
        Serial.print(" typ: "); Serial.print((int)Event.reportObject.object);
        Serial.print(" obj: "); Serial.print((int)Event.reportObject.index);
        Serial.print(" val: "); Serial.println((int)Event.reportObject.data_lsb | (int)(Event.reportObject.data_msb << 8));
        Serial.flush();
#endif

        if (genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_USERBUTTON, GENIE_USER_BUTTON1_ON_FORM0) ||
            genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_USERBUTTON, GENIE_USER_BUTTON2_ON_FORM0))
        {    // cycle-setting form button pressed
                 genie.ReadObject(GENIE_OBJ_STRINGS, 0);
                genie.WriteObject(GENIE_OBJ_FORM, 1, 1);
        } else if (genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_USERBUTTON, GENIE_USER_BUTTON1_ON_FORM1) ||
            genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_USERBUTTON, GENIE_USER_BUTTON2_ON_FORM1))
        {    // in-progress form button pressed
                genie.ReadObject(GENIE_OBJ_STRINGS, 1);
                genie.WriteObject(GENIE_OBJ_FORM, 0, 1);
        } 
        else if (genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_ROTARYSW, 0))
            userChangedCycleWhileStopped = true;
        else if (genie.EventIs(&Event, GENIE_REPORT_OBJ, GENIE_OBJ_STRINGS, 1))
        {
            transitionOnToOff();
            genie.WriteObject(GENIE_OBJ_STRINGS, 0, genie.GetEventData(&Event));
        } 
        else if (genie.EventIs(&Event, GENIE_REPORT_OBJ, GENIE_OBJ_STRINGS, 0))
        {
            auto newIdx = genie.GetEventData(&Event);
            transitionOffToOn(newIdx);
            genie.WriteObject(GENIE_OBJ_STRINGS, 1, newIdx);
        }
    }

    void checkAdvanceTimer(int32_t elapsedMsec, bool advance)
    {
        static unsigned long lastPollTimestamp;
        static int32_t timerMsec;
        auto now = millis();

#if SIMULATE_LIDandFULL==0
        lidClosedPin.update(now);
        tubFullPin.update(now);
        bool lidClosed = lidIsClosed = lidClosedPin.value();
        bool tubFull = tubIsFull = tubFullPin.value();
#else
        lidIsClosed = lidClosed;
        tubIsFull = tubFull;
#endif

#if (INTERACTIVE_DEBUG_MODE)
        if (quickTime)
            elapsedMsec *= 10;

        static int prevLidClosed = -1;
        if ((int)lidClosed != prevLidClosed)
        {
            Serial.println(lidClosed ? "Lid Closed" : "Lid Open");
            Serial.flush();
        }
        prevLidClosed = (int)lidClosed;

        static int prevTubFull = -1;
        if ((int)tubFull != prevTubFull)
        {
            Serial.println(tubFull ? "Tub Full" : "Tub Empty");
            Serial.flush();
        }
        prevTubFull = (int)tubFull;
#endif
#if INTERACTIVE_DEBUG_MODE==0
        digitalWrite(PIN_LID_LED, lidClosed ? LOW : HIGH);
        digitalWrite(PIN_EMPTYFULL_LED, tubFull ? LOW : HIGH);
#endif

        if (advance)
        {
            if (lidClosed)
            {
                if ((TIMER_ON_CLOSED_LID == gCurrentCycleDef.timerEnableOnClosed) ||
                    (tubFull && (TIMER_ON_FULL == gCurrentCycleDef.timerEnableOnFull)))
                {
                    timerMsec += elapsedMsec;
                    while (timerMsec >= 1000)
                    {
                        gTimerSequenceSeconds += 1;
                        while (gTimerSequenceSeconds >= TIMER_FULL_CYCLE)
                            gTimerSequenceSeconds -= TIMER_FULL_CYCLE;
                        timerMsec -= 1000;
                    }
                }
                else
                    timerMsec = 0;
            }
            else
                timerMsec = 0;
        }
    }

    void applyStateToRelays()
    {
        uint8_t step = whichCycleStep(gTimerSequenceSeconds);
#if (INTERACTIVE_DEBUG_MODE)
        static uint8_t prevStep = -1;
        if (step != prevStep)
        {
            Serial.print("Moving to step idx="); Serial.print(step);
            Serial.print(" at seconds: "); Serial.println(gTimerSequenceSeconds);
            Serial.flush();
        }
        prevStep = step;
#endif
        memcpy_P(&gCurrentCycleDef, &MaytagLAT8504[step], sizeof(CycleDefinition));
        // don't command the direction control relay while the motor relay is on.
        bool motorWasOff = ssRelays.getState(RLYSS_MOTOR) == 0;
        // motorOn is CAM6 combined with either the water level, or CAM8+CAM10
        bool motorOn = (gCurrentCycleDef.motorEnable == MOTOR_ON)  &&
            (tubIsFull || 
                (gCurrentCycleDef.timerEnableOnFull == TIMER_ON_FULL &&
                 gCurrentCycleDef.timerEnableOnClosed == TIMER_ON_CLOSED_LID));
        if (!motorOn)
        {
            ssRelays.turnRelayOff(RLYSS_MOTOR);
            if (!motorWasOff)
                delay(MOTOR_DIRECTION_RELAY_SETTLE_MSEC);
            mechRelays.turnRelayOff(RLYM_DIRECTION1);
            mechRelays.turnRelayOff(RLYM_DIRECTION2);
        }
        else
        {
            if (motorWasOff)
            {
#if (INTERACTIVE_DEBUG_MODE)
                Serial.print("Turning motor on "); 
                Serial.println(gCurrentCycleDef.dir == CAM_AGITATE ? "agitate" : "spin");
                Serial.flush();
#endif
                // the RLYM_DIRECTION relays are commanded in concert
                if (gCurrentCycleDef.dir == CAM_AGITATE)
                {
                    mechRelays.turnRelayOff(RLYM_DIRECTION1);
                    mechRelays.turnRelayOff(RLYM_DIRECTION2);
                }
                else
                {
                    mechRelays.turnRelayOn(RLYM_DIRECTION1);
                    mechRelays.turnRelayOn(RLYM_DIRECTION2);
                }
                delay(MOTOR_DIRECTION_RELAY_SETTLE_MSEC);
            }
            ssRelays.turnRelayOn(RLYSS_MOTOR);
        }

        if (gCurrentCycleDef.soakLight == SOAK_LIGHT)
            mechRelays.turnRelayOn(RLYM_SOAKLIGHT);
        else
            mechRelays.turnRelayOff(RLYM_SOAKLIGHT);

        int pinTemp3 = digitalRead(PIN_WATERTEMP3);
        int pinTemp4 = digitalRead(PIN_WATERTEMP4);

        // cold and hot equations are derived from inspection of the circuit diagram.
        bool cold = ((gCurrentCycleDef.coldWaterCam == TEMPERATURE_CAM_BOTTOM) && lidIsClosed) ||
            (gCurrentCycleDef.coldWaterCam == TEMPERATURE_CAM_OFF && 
                gCurrentCycleDef.hotWaterCam == TEMPERATURE_CAM_TOP) ||
            (gCurrentCycleDef.coldWaterCam == TEMPERATURE_CAM_TOP && pinTemp3 == LOW);
        bool hot = (pinTemp4 == LOW) && (gCurrentCycleDef.hotWaterCam == TEMPERATURE_CAM_BOTTOM);

#if (INTERACTIVE_DEBUG_MODE)
        static int prevPin3= -1;
        static int prevPin4 = -1;
        if (pinTemp3 != prevPin3 || pinTemp4 != prevPin4)
        {
            Serial.print("Temperature pin3="); Serial.print(pinTemp3);
            Serial.print(" pin4="); Serial.print(pinTemp4);
            Serial.print(" cold="); Serial.print(cold ? "on" : "off");
            Serial.print(" hot="); Serial.print(hot ? "on" : "off");
            Serial.print(" at seconds: "); Serial.println(gTimerSequenceSeconds);
            Serial.flush();
        }
        prevPin3 = pinTemp3;
        prevPin4 = pinTemp4;
#endif

        if (tubIsFull)
            hot = cold = false;

        if (cold)
            ssRelays.turnRelayOn(RLYSS_COLD_WATER_FILL);
        else
            ssRelays.turnRelayOff(RLYSS_COLD_WATER_FILL);

        if (hot)
            ssRelays.turnRelayOn(RLYSS_HOT_WATER_FILL);
        else
            ssRelays.turnRelayOff(RLYSS_HOT_WATER_FILL);
    }

    void checkUpdateLEDcounter(int32_t elapsedMsec)
    {
        static int32_t lastUpdatedLEDs = -1; // invalid state
        if (gMainKnobState != MAIN_KNOB_RUNNING)
            return;
        // line search the stop times for the first one bigger than now
        for (int i = 0; i < NUM_STOP_TIMES; i++)
        {
            uint16_t stopTime = StopTimes[i];
            int16_t diff = stopTime - gTimerSequenceSeconds;
            if (diff >= 0)
            {
                static uint32_t lastSoakUpdateMsec = -1;
                static uint16_t lastSeconds;
                static uint16_t lastMinutes;
                uint16_t seconds;
                uint16_t minutes;
                if (diff != lastUpdatedLEDs)
                {
                    seconds = diff % 60;
                    minutes = diff / 60;
                    lastUpdatedLEDs = diff;
                    lastSoakUpdateMsec = 0;
                } else if ((i == INFINITE_SOAK_INDEX) && (diff == 0))
                {
                    lastSoakUpdateMsec += elapsedMsec;
                    uint16_t elapsed = lastSoakUpdateMsec / 1000;
                    seconds = elapsed % 60;
                    minutes = elapsed / 60;
                }
                if (seconds != lastSeconds)
                    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 1, seconds);
                if (minutes != lastMinutes)
                    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, minutes);
                lastSeconds = seconds;
                lastMinutes = minutes;
                break;
            }
        }
    }

    void checkUpdateMsecToIdx()
    {
        static uint8_t lastUpdatedIdx = -1;
        int i = secTogenieIdx(gTimerSequenceSeconds);
        if (i == lastUpdatedIdx)
            return;
        lastUpdatedIdx = i;
#if (INTERACTIVE_DEBUG_MODE)
        Serial.print("checkUpdateMsecToIdx update idx="); Serial.println(i); Serial.flush();
#endif

        genie.WriteObject(GENIE_OBJ_ROTARYSW, 0, i);
        genie.WriteObject(GENIE_OBJ_STRINGS, 0, i);
        genie.WriteObject(GENIE_OBJ_STRINGS, 1, i);
    }
}
