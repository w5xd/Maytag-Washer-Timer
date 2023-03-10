#pragma once
// This file is generated from captured cam closing on Maytag washing machine timer
    enum { TIMER_FULL_CYCLE = 5316 };
    enum WatertempCamState_t { TEMPERATURE_CAM_TOP, TEMPERATURE_CAM_BOTTOM, TEMPERATURE_CAM_OFF };
    enum CamMotorDirection_t { CAM_AGITATE, CAM_SPIN };
	enum MotorState_t { MOTOR_OFF, MOTOR_ON };
    enum TimerCam8_t {NO_TIMER_ON_FULL, TIMER_ON_FULL};
    enum TimerCam10T_t {NO_TIMER_ON_CLOSED_LID, TIMER_ON_CLOSED_LID};
    enum TimerCam10B_t {NO_SOAK_LIGHT, SOAK_LIGHT};
    struct CycleDefinition {
        uint16_t timeStampSeconds;
        CamMotorDirection_t dir;    // cams 2 and 4
        MotorState_t motorEnable;   // cam 6
        TimerCam8_t timerEnableOnFull; // cam 8
        TimerCam10T_t timerEnableOnClosed;   // cam 10 T
        TimerCam10B_t soakLight;     // cam 10 B
        WatertempCamState_t hotWaterCam; // cam 12
        WatertempCamState_t coldWaterCam; // cam 14
    };
const CycleDefinition MaytagLAT8504[] PROGMEM = {
/*0*/	{0,	CAM_SPIN,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=0.0 /25=0.0
/*1*/	{90,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=1.0 /25=0.4
/*2*/	{179,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_TOP}, //	interval=2.0 /25=0.8
/*3*/	{267,	CAM_AGITATE,	MOTOR_ON,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_TOP}, //	interval=3.0 /25=1.3
/*4*/	{268,	CAM_AGITATE,	MOTOR_ON,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_BOTTOM,	TEMPERATURE_CAM_TOP}, //	interval=3.0 /25=1.3
/*5*/	{892,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_BOTTOM,	TEMPERATURE_CAM_TOP}, //	interval=9.9 /25=4.2
/*6*/	{900,	CAM_SPIN,	MOTOR_OFF,	NO_TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=10.0 /25=4.2
/*7*/	{902,	CAM_SPIN,	MOTOR_ON,	NO_TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=10.0 /25=4.2
/*8*/	{982,	CAM_SPIN,	MOTOR_OFF,	NO_TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=10.9 /25=4.6
/*9*/	{990,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_TOP,	TEMPERATURE_CAM_OFF}, //	interval=11.0 /25=4.7
/*10*/	{992,	CAM_AGITATE,	MOTOR_ON,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_TOP,	TEMPERATURE_CAM_OFF}, //	interval=11.0 /25=4.7
/*11*/	{1072,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_TOP,	TEMPERATURE_CAM_OFF}, //	interval=11.9 /25=5.0
/*12*/	{1080,	CAM_SPIN,	MOTOR_OFF,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=12.0 /25=5.1
/*13*/	{1082,	CAM_SPIN,	MOTOR_ON,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=12.0 /25=5.1
/*14*/	{1252,	CAM_SPIN,	MOTOR_OFF,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=13.9 /25=5.9
/*15*/	{1260,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_TOP,	TEMPERATURE_CAM_OFF}, //	interval=14.0 /25=5.9
/*16*/	{1262,	CAM_AGITATE,	MOTOR_ON,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_TOP,	TEMPERATURE_CAM_OFF}, //	interval=14.0 /25=5.9
/*17*/	{1342,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_TOP,	TEMPERATURE_CAM_OFF}, //	interval=14.9 /25=6.3
/*18*/	{1350,	CAM_SPIN,	MOTOR_OFF,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=15.0 /25=6.3
/*19*/	{1352,	CAM_SPIN,	MOTOR_ON,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=15.0 /25=6.4
/*20*/	{1410,	CAM_SPIN,	MOTOR_ON,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_BOTTOM}, //	interval=15.7 /25=6.6
/*21*/	{1425,	CAM_SPIN,	MOTOR_ON,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=15.8 /25=6.7
/*22*/	{1615,	CAM_SPIN,	MOTOR_OFF,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=17.9 /25=7.6
/*23*/	{1621,	CAM_SPIN,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=18.0 /25=7.6
/*24*/	{1713,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=19.0 /25=8.1
/*25*/	{1800,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_TOP}, //	interval=20.0 /25=8.5
/*26*/	{1888,	CAM_AGITATE,	MOTOR_ON,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_TOP}, //	interval=21.0 /25=8.9
/*27*/	{1889,	CAM_AGITATE,	MOTOR_ON,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_BOTTOM,	TEMPERATURE_CAM_TOP}, //	interval=21.0 /25=8.9
/*28*/	{1976,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_BOTTOM,	TEMPERATURE_CAM_TOP}, //	interval=22.0 /25=9.3
/*29*/	{1980,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	SOAK_LIGHT,	TEMPERATURE_CAM_BOTTOM,	TEMPERATURE_CAM_TOP}, //	interval=22.0 /25=9.3
/*30*/	{2158,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_BOTTOM,	TEMPERATURE_CAM_TOP}, //	interval=24.0 /25=10.1
/*31*/	{2160,	CAM_AGITATE,	MOTOR_ON,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_BOTTOM,	TEMPERATURE_CAM_TOP}, //	interval=24.0 /25=10.2
/*32*/	{2246,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_BOTTOM,	TEMPERATURE_CAM_TOP}, //	interval=25.0 /25=10.6
/*33*/	{2250,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	SOAK_LIGHT,	TEMPERATURE_CAM_BOTTOM,	TEMPERATURE_CAM_TOP}, //	interval=25.0 /25=10.6
/*34*/	{2429,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_BOTTOM,	TEMPERATURE_CAM_TOP}, //	interval=27.0 /25=11.4
/*35*/	{2430,	CAM_AGITATE,	MOTOR_ON,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_BOTTOM,	TEMPERATURE_CAM_TOP}, //	interval=27.0 /25=11.4
/*36*/	{2514,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_BOTTOM,	TEMPERATURE_CAM_TOP}, //	interval=27.9 /25=11.8
/*37*/	{2522,	CAM_SPIN,	MOTOR_OFF,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_TOP}, //	interval=28.0 /25=11.9
/*38*/	{2524,	CAM_SPIN,	MOTOR_ON,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=28.0 /25=11.9
/*39*/	{2671,	CAM_SPIN,	MOTOR_ON,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_BOTTOM}, //	interval=29.7 /25=12.6
/*40*/	{2687,	CAM_SPIN,	MOTOR_ON,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=29.9 /25=12.6
/*41*/	{2694,	CAM_SPIN,	MOTOR_OFF,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=29.9 /25=12.7
/*42*/	{2702,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_TOP,	TEMPERATURE_CAM_OFF}, //	interval=30.0 /25=12.7
/*43*/	{2704,	CAM_AGITATE,	MOTOR_ON,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_TOP,	TEMPERATURE_CAM_OFF}, //	interval=30.0 /25=12.7
/*44*/	{2784,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_TOP,	TEMPERATURE_CAM_OFF}, //	interval=30.9 /25=13.1
/*45*/	{2793,	CAM_SPIN,	MOTOR_OFF,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=31.0 /25=13.1
/*46*/	{2794,	CAM_SPIN,	MOTOR_ON,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=31.0 /25=13.1
/*47*/	{2851,	CAM_SPIN,	MOTOR_ON,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_BOTTOM}, //	interval=31.7 /25=13.4
/*48*/	{2867,	CAM_SPIN,	MOTOR_ON,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=31.9 /25=13.5
/*49*/	{3057,	CAM_SPIN,	MOTOR_OFF,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=34.0 /25=14.4
/*50*/	{3063,	CAM_SPIN,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=34.0 /25=14.4
/*51*/	{3153,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=35.0 /25=14.8
/*52*/	{3242,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_TOP}, //	interval=36.0 /25=15.2
/*53*/	{3330,	CAM_AGITATE,	MOTOR_ON,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_TOP}, //	interval=37.0 /25=15.7
/*54*/	{3331,	CAM_AGITATE,	MOTOR_ON,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	SOAK_LIGHT,	TEMPERATURE_CAM_BOTTOM,	TEMPERATURE_CAM_TOP}, //	interval=37.0 /25=15.7
/*55*/	{3508,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	SOAK_LIGHT,	TEMPERATURE_CAM_BOTTOM,	TEMPERATURE_CAM_TOP}, //	interval=39.0 /25=16.5
/*56*/	{3510,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_TOP}, //	interval=39.0 /25=16.5
/*57*/	{3515,	CAM_AGITATE,	MOTOR_OFF,	NO_TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_TOP}, //	interval=39.1 /25=16.5
/*58*/	{3601,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_TOP}, //	interval=40.0 /25=16.9
/*59*/	{3690,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_TOP}, //	interval=41.0 /25=17.4
/*60*/	{3780,	CAM_AGITATE,	MOTOR_ON,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_TOP}, //	interval=42.0 /25=17.8
/*61*/	{3782,	CAM_AGITATE,	MOTOR_ON,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_BOTTOM,	TEMPERATURE_CAM_TOP}, //	interval=42.0 /25=17.8
/*62*/	{4496,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_BOTTOM,	TEMPERATURE_CAM_TOP}, //	interval=50.0 /25=21.1
/*63*/	{4504,	CAM_SPIN,	MOTOR_OFF,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=50.0 /25=21.2
/*64*/	{4505,	CAM_SPIN,	MOTOR_ON,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=50.1 /25=21.2
/*65*/	{4653,	CAM_SPIN,	MOTOR_ON,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_BOTTOM}, //	interval=51.7 /25=21.9
/*66*/	{4669,	CAM_SPIN,	MOTOR_ON,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=51.9 /25=22.0
/*67*/	{4766,	CAM_SPIN,	MOTOR_OFF,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=53.0 /25=22.4
/*68*/	{4774,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_TOP,	TEMPERATURE_CAM_OFF}, //	interval=53.0 /25=22.5
/*69*/	{4776,	CAM_AGITATE,	MOTOR_ON,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_TOP,	TEMPERATURE_CAM_OFF}, //	interval=53.1 /25=22.5
/*70*/	{4946,	CAM_AGITATE,	MOTOR_OFF,	TIMER_ON_FULL,	NO_TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_TOP,	TEMPERATURE_CAM_OFF}, //	interval=55.0 /25=23.3
/*71*/	{4955,	CAM_SPIN,	MOTOR_OFF,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=55.1 /25=23.3
/*72*/	{4956,	CAM_SPIN,	MOTOR_ON,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=55.1 /25=23.3
/*73*/	{5014,	CAM_SPIN,	MOTOR_ON,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_BOTTOM}, //	interval=55.7 /25=23.6
/*74*/	{5029,	CAM_SPIN,	MOTOR_ON,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=55.9 /25=23.7
/*75*/	{5309,	CAM_SPIN,	MOTOR_OFF,	TIMER_ON_FULL,	TIMER_ON_CLOSED_LID,	NO_SOAK_LIGHT,	TEMPERATURE_CAM_OFF,	TEMPERATURE_CAM_OFF}, //	interval=59.0 /25=25.0
};
enum {NUM_WASHER_TIMER_STEPS=76};
