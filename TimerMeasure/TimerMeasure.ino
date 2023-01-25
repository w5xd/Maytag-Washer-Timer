// This sketch runs on a Mega (or something with at least 21 input pins)
// It uses pin input/output mode changes to probe whether the middle pin in a group of 3 consectutive
// pins is connected to the other two (one lower number, one higher number).
// Each group of 3 is to follow one cam on an oldstyle washing machine timer.
// Every second, it prings to Serial1 what each CAM appears to be (0 is unconnected, T is top, B is bottom and E is both)

enum CamState_t {
    CAM_OFF, CAM_T, CAM_B, CAM_ERROR, NUM_CAM_STATES
};

namespace {
    const int PinCam2T = 53;
    const int PinCam2Pole = 52;
    const int PinCam2B = 51;

    const int PinCam6B = PinCam2B - 6;
    const int PinCam4B = PinCam2B - 3;

    const int NUM_CAMS = 7;

    const char Decode[NUM_CAM_STATES] = { '0', 'T', 'B', 'E' };

    const int CamNumber[NUM_CAMS] = { 2, 4, 6, 8, 10, 12, 14 };

    CamState_t CamStates[NUM_CAMS];

    CamState_t CamLastState[NUM_CAMS];

    int TimesInLastState[NUM_CAMS];
}

void AquireCam(int which)
{
    int pinT = PinCam2T - (3 * which);
    int pinPole = pinT - 1;
    int pinB = pinPole - 1;

    // the maytag timer has internal connections for Cam2 bottom and Cam4 bottom
    // that don't appear on the connector per the pattern of the other cams.
    if (pinB == PinCam2B)
        pinB = PinCam6B;
    else if (pinB == PinCam4B)
        pinB = PinCam6B;

    pinMode(pinPole, OUTPUT);
    pinMode(pinT, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    digitalWrite(pinPole, LOW);

    CamState_t st = CAM_OFF;
    bool pinTValue = digitalRead(pinT) != LOW;
    bool pinBValue = digitalRead(pinB) != LOW;

    if (!pinTValue && !pinBValue)
        st = CAM_ERROR;
    else if (!pinTValue)
        st = CAM_T;
    else if (!pinBValue)
        st = CAM_B;

    if (CamLastState[which] == st)
    {
        TimesInLastState[which] += 1;
        if (TimesInLastState[which] > 4)
            CamStates[which] = st;
    }
    else
    {
        TimesInLastState[which] = 0;
        CamLastState[which] = st;
    }

    // restore the changed pin modes. 
    // This is crucial. Each measurement in this routine depends on the fact
    // that all the other pins not being measured are open circuit (INPUT)
    pinMode(pinPole, INPUT);
    pinMode(pinT, INPUT);
    pinMode(pinB, INPUT);
}


void setup()
{
    Serial1.begin(38400);
    // all pins are left as INPUT except for in AquireCam.
}

void loop()
{
    static long secondsAtLastReport;
    // every loop, acquire all 6 cams
    for (int i = 0; i < NUM_CAMS; i++)
        AquireCam(i);

    long nowSeconds = millis();
    nowSeconds /= 1000;

    if (nowSeconds != secondsAtLastReport)
    {
        secondsAtLastReport = nowSeconds;

        Serial1.print("time ");
        Serial1.print(nowSeconds);
        
        for (int i = 0; i < NUM_CAMS; i++)
        {
            Serial1.print(" Cam");
            Serial1.print(CamNumber[i]);
            Serial1.print(":");
            Serial1.print(Decode[static_cast<int>(CamStates[i])]);
        }
        Serial1.println();
    }


}
