using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Recorder
{

    class Program
    {
        static void Main(string[] args)
        {
            if (args.Length != 2)
            {
                Console.WriteLine("Recorder <COMPORT> <FileName>");
                return;
            }

            var r = new Recorder();
            r.start(args[0], args[1]);
            r.wait();

        }
    }

    class Recorder
    {
        System.IO.Ports.SerialPort port;
        System.IO.StreamWriter logfile;

        const int COMM_BAUD = 38400;

        public void wait()
        {
            for (; ; )
            {
                var s = Console.ReadLine();
                if (s.ToLower() == "quit")
                {
                    port.Close();
                    logfile.Close();
                    port.Dispose();
                    logfile.Dispose();
                    return;
                }
            }
        }

        public void start(string comm, string file)
        {
            port = new System.IO.Ports.SerialPort(comm, COMM_BAUD, System.IO.Ports.Parity.None, 8, System.IO.Ports.StopBits.One);
            port.Handshake = System.IO.Ports.Handshake.None;
            port.Encoding = new System.Text.ASCIIEncoding();
            port.RtsEnable = true;
            port.DtrEnable = false;
            port.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(port_DataReceived);
            port.Open();

            logfile = new System.IO.StreamWriter(file);
        }

        string incoming = "";

        object lck = new object();

        private void port_DataReceived(object sender,
              System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            for (; ;)
            {
                String s = port.ReadExisting();
                if (!s.Any())
                    return;
                lock (lck)
                {
                    incoming += s;
                    int idx;
                    char[] nl = { '\r', '\n' };
                    while (0 <= (idx = incoming.IndexOfAny(nl)))
                    {
                        string toPrint = null;
                        if (idx > 0)
                            toPrint = incoming.Substring(0, idx);
                        incoming = incoming.Substring(idx + 1);
                        if (!String.IsNullOrEmpty(toPrint))
                        {
                            Console.WriteLine(toPrint);
                            logfile.WriteLine(toPrint);
                        }
                    }
                }
            }
        }
    }
}
