
Letters used for SPI:

Raspberry Pi to Arduino

Forward:                F
Backward:               B
Left:                   L
Right:                  R
Line Follow mode:       T
Object Detection mode:  O


HOW TO RUN: 
use moba xterm or another terminal to ssh into the pi by typing in:

ssh pi@(insert the ip address here)
  
You type into terminal:

cd MinibotHandoff 

python -c 'import PiArduino; PiArduino.ObjectDetection()'

NOTE: change .ObjectDetection() function to whatever function you want to run (ex. fwd(), left(),right(),stop(),back(),LineFollow() 

Arduino1.ino refers to top arduino (Closest to the raspberry pi) Arduino2.ino referes to bottom arduino (closest to battery)

When loading Arduino (.ino) programs: Connect USB to serial (USB to jumper cables) and run SPI_1.ino on the top arduino and then connect the cable to the bottom arduino and run SPI_A2.ino on the bottom one.

When cloning this Repo (If not already loaded on the Pi):

Make sure you have SPI enabled (easiest way is through the GUI in NOOBS so you can just go to configuration and enable) Otherwise on terminal do ifconfig and select the option for serial (I think option 6, will verify later)
Make sure you have SCL,SDA,CE0, and CE1 wires all connected
