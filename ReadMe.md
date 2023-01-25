Maytag LAT8504 Washing machine timer replacement

This repository documents a project that replaces the failed timer on my Maytag washing machine. Maybe there is
enough detail here to duplicate this project, and maybe the parts are even available. There is no particular
reason to use the exact parts I specify here, especially for the relays, but note that K1 and K2 must 
support full motor current, which is specified as 7.5A.

<p align='center'><img src='AsInstalled.jpg' alt='AsInstalled.jpg'/></p>

 A few parts, mostley from Sparkfun can be assembled to replace the timer in the Maytag LAT8504. All these
 parts together cost me less than half the cost of an exact replacement mechanical timer.
 <ul>
 <li> <a href='https://www.sparkfun.com/products/15795'>Qwiic Pro Micro USB-C</a></li>
 <li><a href='https://www.sparkfun.com/products/16566'>Qwiic Quad relay</a></li>
 <li><a href='https://www.sparkfun.com/products/16833'>Qwiic Quad Solid state relay kit</a></li>
 <li>Two opto isolators are also required, along with a 100K 1/4W resistor for each. A reverse diode is required
 as shown if the opto isolator is unidirectional. (The circuit diagram shows
 a third that senses line power from neutral, but the sketch doesn't use that input, and the 
 corresponding parts may be omitted.)
 <li><a href='https://4dsystems.com.au/gen4-ulcd-24pt'>4D systems GEN4-ULCD-24PT</a>
 </ul>

 Four plastic pieces are needed to mount everything. 3D printable STL files are in the stl 
 directory.

 None of the original wires in the washer are cut, nor is the connector for the timer removed. Ot
 restore the original timer, only the 
 spade lugs at the water temperature switch need to be restored to their original connections, the MOLEX connector for this
 device unplugged, the 3D printed plastic parts around the original timer hole removed, and the
 original timer installed and plugged into its original connector. The original connector is
 not plainly visible in the photos below, but its there along with all the wires connected to
 it.

 The Arduino Pro Micro is soldered to a slightly larger breadboard along with the optoisolators,
 associated resistors, and the 5 pin header for the LCD screen.


 Photos:
 <p align='center'><img src='Inside-front-panel-1.jpg' alt='Inside-front-panel-1.jpg'/></p>
<p align='center'><img src='Inside-front-panel-2.jpg' alt='Inside-front-panel-2.jpg'/></p>
<p align='center'><img src='Molex-connectors.jpg' alt='Molex-connectors.jpg'/></p>
