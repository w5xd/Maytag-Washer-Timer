using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CleanCapture
{
    class Program
    {
        static int CYCLE = 5316; // empirically determined length in seconds of the timer's full cycle.
        static void Main(string[] args)
        {
            if (args.Count() < 2)
            {
                Console.WriteLine("usage: CleanCapture <in-file> <out-file> [offset] [header]");
                return;
            }

            System.IO.StreamReader sr = new System.IO.StreamReader(args[0]);
            System.IO.StreamWriter sw = new System.IO.StreamWriter(args[1]);
            int offset = 0;
            if (args.Count() >= 3)
                offset = Int32.Parse(args[2]);

            Dictionary<int, string> headerContents = null;

            System.IO.StreamWriter header = null;
            if (args.Count() == 4)
            {
                header = new System.IO.StreamWriter(args[3]);
                header.Write(@"#pragma once
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
");
                headerContents = new Dictionary<int, string>();
            }
            Dictionary<string, string> camstrings = new Dictionary<string, string>();
            camstrings["B"] = "TEMPERATURE_CAM_BOTTOM";
            camstrings["0"] = "TEMPERATURE_CAM_OFF";
            camstrings["T"] = "TEMPERATURE_CAM_TOP";

            Dictionary<int, string> prevalues = new Dictionary<int, string>();
            string line;
            while ((line = sr.ReadLine()) != null)
            {
                Dictionary<int, string> values = new Dictionary<int, string>();
                var split = line.Split(null);
                int time = Int32.Parse(split[1]);
                time -= offset;
                if (time < 0)
                    time += CYCLE;
                foreach (string s in split)
                {
                    if (s.StartsWith("Cam"))
                    {
                        var pair = s.Split(new char[] { ':' });
                        if (pair.Count() == 2)
                        {
                            int num = Int32.Parse(pair[0].Substring(3));
                            values[num] = pair[1];
                        }
                    }
                }
                var sorted = values.OrderBy(x => x.Key);
                foreach (var p in sorted)
                {
                    if (!(prevalues.ContainsKey(p.Key) && prevalues[p.Key] == p.Value))
                    {
                        var sb = new StringBuilder();
                        time %= CYCLE;
                        sb.Append(String.Format("time {0}", time));
                        foreach (var q in values)
                            sb.Append(String.Format(" Cam{0}:{1}", q.Key, q.Value));
                        if (prevalues.Any())
                        {
                            sw.WriteLine(sb);
                            if (header != null)
                            {
                                if (values[2] != "0" && values[4] != "0")
                                {
                                    var ln = String.Format("\t{{{0},\t{1},\t{2},\t{3},\t{4},\t{5},\t{6},\t{7}}}, //\tinterval={8:0.0} /25={9:0.0}",
                                        time,
                                        values[4] == "T" ? "CAM_AGITATE" : "CAM_SPIN",
                                        values[6] == "B" ? "MOTOR_ON" : "MOTOR_OFF",
                                        values[8] == "T" ? "TIMER_ON_FULL" : "NO_TIMER_ON_FULL",
                                        values[10] == "T" ? "TIMER_ON_CLOSED_LID" : "NO_TIMER_ON_CLOSED_LID",
                                        values[10] == "B" ? "SOAK_LIGHT" : "NO_SOAK_LIGHT",
                                        camstrings[values[12]],
                                        camstrings[values[14]],
                                        time / 90.0,
                                        time * 25.0 / 5316
                                        ); ;
                                    if (!headerContents.ContainsKey(time))
                                        headerContents.Add(time, ln);
                                }
                            }
                        }
                        prevalues = values;
                        break;
                    }
                }
            }

            sr.Close();
            sw.Close();
            if (null != header)
            {
                var sortedHeader = headerContents.OrderBy(x => x.Key);
                int i = 0;
                foreach (var ln in sortedHeader)
                    header.WriteLine(String.Format("/*{0}*/{1}", i++, ln.Value));
                header.WriteLine("};");
                header.WriteLine(String.Format("enum {{NUM_WASHER_TIMER_STEPS={0}}};", headerContents.Count));
                header.Close();
            }
        }
    }
}
