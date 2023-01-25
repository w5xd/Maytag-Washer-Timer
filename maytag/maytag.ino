/* Maytag washer timer implementation.
* 
*/
// Serial is debug diagnostics on USB
// Serial1 is the gen4-Ulcd-24PT
#include <SparkFun_Qwiic_Relay.h>
#include <genieArduino.h>
#include "MaytagCycleDefinitions.h"

#define DISABLE_SHUTDOWN_ON_SENSE_NO_AC 1 // not useful
// All these conditional compile directives are 0 for production program
#define INTERACTIVE_DEBUG_MODE 0
#define SIMULATE_LIDandFULL 0
#define AC_TIMER_STATS 0

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

    const uint8_t PIN_LIDCLOSED_SENSE = 14;        // PCINT3
    const uint8_t PIN_CLOSEDFULL_SENSE = 15;    // PCINT1
    const uint8_t PIN_AC_SENSE = 16;            // PCINT2
#if defined  (__AVR_ATmega32U4__) // PRO MICRO specific PCINT settings to sense 60HZ on/off
    const uint8_t PIN_REGB_MASK = (1 << PCINT2) | (1 << PCINT1) | (1 << PCINT3);
#endif

    const uint8_t PIN_WATERTEMP3 = 4;
    const uint8_t PIN_WATERTEMP4 = 5;

    const uint8_t PIN_RESETLCD = A3;

    const int MSEC_AC_SENSE_ASSUME_OFF = 40;

    enum {MAIN_KNOB_STOPPED, MAIN_KNOB_RUNNING}
        gMainKnobState = MAIN_KNOB_STOPPED;

    const uint8_t MOTOR_DIRECTION_RELAY_SETTLE_MSEC = 50;

    enum { NUM_GENIE_DISPLAY_CYCLE_SETTINGS = 17, GENIE_DISPLAY_CYCLE_START = 8 };
    const uint16_t UserCycleToTime[NUM_GENIE_DISPLAY_CYCLE_SETTINGS] PROGMEM =
    {    // the genie object starts at angle 180.
        // the dryer display starts at angle 0
        2867, // spin

        3331, // SOAK
        3780, // REGULAR
        3780 + 180, // 9 min
        3780 + 2 * 180, // 6 min
        3780 + 3 * 180, // 3 min
        4766, // rinse
        4946, // spin

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
    void checkAdvanceTimer(uint16_t elapsedMsec);
    void applyStateToRelays();
    void checkUpdateLEDcounter(uint16_t elapsedMsec);
    void checkUpdateCycleDisplay();
    void checkUpdateMsecToIdx();

    Qwiic_Relay mechRelays(MECH_RELAY_I2CADDR);
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

    volatile unsigned long lastSawAConMsec; // updated on every 60Hz zero crossing
    volatile unsigned long lastSawLidClosedMsec;
    volatile bool haveSeenLidClosed;
    volatile unsigned long lastSawTubFullMsec;
    volatile bool haveSeenTubFull;
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

    pinMode(PIN_LIDCLOSED_SENSE, INPUT_PULLUP);
    pinMode(PIN_CLOSEDFULL_SENSE, INPUT_PULLUP);
    pinMode(PIN_WATERTEMP3, INPUT_PULLUP);
    pinMode(PIN_WATERTEMP4, INPUT_PULLUP);
    pinMode(PIN_AC_SENSE, INPUT_PULLUP);
    
    memcpy_P(&gCurrentCycleDef, &MaytagLAT8504[0], sizeof(CycleDefinition));
    lastSawAConMsec = millis(); // avoid power down on first call to loop()!

#if defined  (__AVR_ATmega32U4__)
    // Use pin change interrupts to sense 60Hz AC inputs
    PCMSK0 |= PIN_REGB_MASK;
    PCICR |= 1;
#endif
}

void loop()
{
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
    uint16_t elapsed = now - prevNow;
    prevNow = now;

#if (0 == DISABLE_SHUTDOWN_ON_SENSE_NO_AC)
    noInterrupts();
    auto prevACTime = lastSawAConMsec; 
    interrupts();
    int diffMsec = now - prevACTime;
#if (AC_TIMER_STATS != 0) && (INTERACTIVE_DEBUG_MODE !=0)
    {
        // this printout helped figure out how to program the sensing of 60HZ input on/off
        static uint16_t statCount = 0;
        static uint32_t delTotal = 0;
        static uint32_t delSquare = 0;
        static uint16_t maxMsec = 0;
        delTotal += diffMsec;
        statCount += 1;
        delSquare += diffMsec * diffMsec;
        if (diffMsec > maxMsec)
            maxMsec = diffMsec;
        static unsigned long lastReportedStats;
        auto msecSinceReported = now - lastReportedStats;
        if (msecSinceReported > 100)
        {
            Serial.print("stats. N=");
            Serial.print(statCount);
            Serial.print(" T=");
            Serial.print(delTotal);
            Serial.print(" T2=");
            Serial.print(delSquare);
            Serial.print(" M=");
            Serial.print(maxMsec);
            Serial.print(" D=");
            Serial.println(msecSinceReported);
            delTotal = 0;
            statCount = 0;
            delSquare = 0;
            maxMsec = 0;
            lastReportedStats = now;
        }
    }
#endif
    bool shutdown = diffMsec > MSEC_AC_SENSE_ASSUME_OFF;
    if (shutdown)
    {   // if we see AC missing for MSEC_AC_SENSE_ASSUME_OFF,
        // then try to command the solid state relays off in hopes of not sparking the 
        // mechanical relays.
        ssRelays.turnAllRelaysOff();
#if (INTERACTIVE_DEBUG_MODE !=0)
        Serial.println("power down");
#else
        for (;;); // die here. Power up reset is the only way out
#endif
    }
#endif

    genie.DoEvents();
    bool isEnabled = gMainKnobState != MAIN_KNOB_STOPPED;
    if (!isEnabled)
    {
        auto wasOn = ssRelays.getState(1);
        ssRelays.turnAllRelaysOff(); 
        if (wasOn)
            delay(MOTOR_DIRECTION_RELAY_SETTLE_MSEC); // when turning off, let solid state finish first.
        mechRelays.turnAllRelaysOff(); 
        return;
    }

    checkAdvanceTimer(elapsed);
    checkUpdateLEDcounter(elapsed);
    checkUpdateMsecToIdx();
    applyStateToRelays();

    ssRelays.turnRelayOn(RLYSS_MAIN_LINE);
}

namespace {
    uint8_t whichCycleStep(uint16_t timerSequenceSeconds)
    {    // binary search the step number from the number of seconds into the timer cycle
        uint8_t lowest = 0;
        uint8_t highest = NUM_WASHER_TIMER_STEPS - 1;
        for (;;)
        {
            uint8_t whichStep = (highest + lowest) / 2;
            uint16_t stepStartSeconds = pgm_read_word_near(&MaytagLAT8504[whichStep].timeStampSeconds);
            int compare = stepStartSeconds - timerSequenceSeconds;
            if (compare == 0)
                return whichStep;
            else if (compare < 0)
                lowest = whichStep;
            else
                highest = whichStep;
            uint8_t diff = highest - lowest;
            if (diff <= 1)
            {
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
#endif
        }
        gMainKnobState = MAIN_KNOB_RUNNING;
        // The following two deal with the fact that 16 bit rollover of msec (every 64 seconds)
        // the 60Hz sense would appear to be recent. If the the 60Hz input disappears and stays
        // gone, then this digitalRead sets the record straight
        haveSeenLidClosed = digitalRead(PIN_LIDCLOSED_SENSE) == LOW;
        haveSeenTubFull = digitalRead(PIN_CLOSEDFULL_SENSE) == LOW;
    }

    uint8_t secTogenieIdx(uint16_t secs)
    {    // OFF cycles at begining of definition map to end of previous cycle
        if (secs < pgm_read_word_near(&UserCycleToTime[GENIE_DISPLAY_CYCLE_START]))
            secs += TIMER_FULL_CYCLE; // modular on TIMER_FULL_CYCLE
        int i = 0;
        for (int j = 0; j < NUM_GENIE_DISPLAY_CYCLE_SETTINGS; j += 1)
        {
            i = GENIE_DISPLAY_CYCLE_START - j - 1;
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
        Serial.print("transitionOnToOff msec=");
        Serial.print(gTimerSequenceSeconds);
        Serial.print(" idx="); Serial.print(i);
        Serial.println();
#endif
    }

    void genieEvents()
    {
        genieFrame Event;
        genie.DequeueEvent(&Event);
#if (INTERACTIVE_DEBUG_MODE)
        Serial.print("Event cmd: ");
        Serial.print((int)Event.reportObject.cmd);
        Serial.print(" typ: "); Serial.print((int)Event.reportObject.object);
        Serial.print(" obj: "); Serial.print((int)Event.reportObject.index);
        Serial.print(" val: "); Serial.println((int)Event.reportObject.data_lsb | (int)(Event.reportObject.data_msb << 8));
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

    void checkAdvanceTimer(uint16_t elapsedMsec)
    {
        static uint16_t timerMsec;
        auto now = millis();

#if SIMULATE_LIDandFULL==0
        // reading half a 60Hz input cycle.
        noInterrupts();
        auto prevLid = lastSawLidClosedMsec;
        auto prevFull = lastSawTubFullMsec;
        auto isClosedRecently = haveSeenLidClosed;
        auto isFullRecently = haveSeenTubFull;
        interrupts();

        uint16_t diffLidMsec = now - prevLid;
        uint16_t diffFullMsec = now - prevFull;
        bool lidClosed = isClosedRecently && diffLidMsec < MSEC_AC_SENSE_ASSUME_OFF;
        bool tubFull = isFullRecently && diffFullMsec < MSEC_AC_SENSE_ASSUME_OFF;

        if (!lidClosed) // this "times out" the use of the last-seen time stamps until
            haveSeenLidClosed = false;
        if (!tubFull)
            haveSeenTubFull = false;
#else
        haveSeenTubFull = tubFull;
#endif

#if (INTERACTIVE_DEBUG_MODE)
        if (quickTime)
            elapsedMsec *= 10;

        static int prevLidClosed = -1;
        static int prevTubFull = -1;
        if ((int)lidClosed != prevLidClosed)
            Serial.println(lidClosed ? "Lid Closed" : "Lid Open");
        prevLidClosed = (int)lidClosed;

        if ((int)tubFull != prevTubFull)
            Serial.println(tubFull ? "Tub Full" : "Tub Empty");
        prevTubFull = (int)tubFull;
#endif

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
        }
        prevStep = step;
#endif
        memcpy_P(&gCurrentCycleDef, &MaytagLAT8504[step], sizeof(CycleDefinition));
        // don't command the direction control relay while the motor relay is on.
        bool motorWasOff = ssRelays.getState(RLYSS_MOTOR) == 0;
        if (motorWasOff)
        {
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
        }
        // motorOn is CAM6 combined with either the water level, or CAM8+CAM10
        bool motorOn = (gCurrentCycleDef.motorEnable == MOTOR_ON)  &&
            (haveSeenTubFull || 
                (gCurrentCycleDef.timerEnableOnFull == TIMER_ON_FULL &&
                 gCurrentCycleDef.timerEnableOnClosed == TIMER_ON_CLOSED_LID));
        if (!motorOn)
            ssRelays.turnRelayOff(RLYSS_MOTOR);
        else
        {
            if (motorWasOff)
                delay(MOTOR_DIRECTION_RELAY_SETTLE_MSEC);
            ssRelays.turnRelayOn(RLYSS_MOTOR);
        }

        if (gCurrentCycleDef.soakLight == SOAK_LIGHT)
            mechRelays.turnRelayOn(RLYM_SOAKLIGHT);
        else
            mechRelays.turnRelayOff(RLYM_SOAKLIGHT);

        int pinTemp3 = digitalRead(PIN_WATERTEMP3);
        int pinTemp4 = digitalRead(PIN_WATERTEMP4);

        // cold and hot equations are derived from inspection of the circuit diagram.
        bool cold = (gCurrentCycleDef.coldWaterCam == TEMPERATURE_CAM_BOTTOM) ||
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
        }
        prevPin3 = pinTemp3;
        prevPin4 = pinTemp4;
#endif

        if (cold)
            ssRelays.turnRelayOn(RLYSS_COLD_WATER_FILL);
        else
            ssRelays.turnRelayOff(RLYSS_COLD_WATER_FILL);

        if (hot)
            ssRelays.turnRelayOn(RLYSS_HOT_WATER_FILL);
        else
            ssRelays.turnRelayOff(RLYSS_HOT_WATER_FILL);
    }

    void checkUpdateLEDcounter(uint16_t elapsedMsec)
    {
        static uint16_t lastUpdatedLEDs = -1; // invalid state
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
                } else if ((i == INFINITE_SOAK_INDEX) && stopTime == gTimerSequenceSeconds)
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
        genie.WriteObject(GENIE_OBJ_ROTARYSW, 0, i);
        genie.WriteObject(GENIE_OBJ_STRINGS, 0, i);
        genie.WriteObject(GENIE_OBJ_STRINGS, 1, i);
    }
}

#if defined  (__AVR_ATmega32U4__)
ISR(PCINT0_vect)
{
    auto now = millis();
    uint8_t portB = PINB;
    if ((portB & PIN_REGB_MASK) != PIN_REGB_MASK)
    {    // if any of the bits is not set
        if ((portB & (1 << PCINT3)) == 0)
        {
            lastSawLidClosedMsec = now;
            haveSeenLidClosed = true;
        }
        if ((portB & (1 << PCINT1)) == 0)
        {
            lastSawTubFullMsec = now;
            haveSeenTubFull = true;
        }
    }
    lastSawAConMsec = now; // Line AC timestamp we update on any transition at all
}
#else
#error "unsupported CPU"
#endif