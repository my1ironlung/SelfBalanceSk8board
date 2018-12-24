 import processing.serial.*;

 Serial myPort;        // The serial port
 int xPos = 1;         // horizontal position of the graph
 void setup () {
 // set the window size:
 size(1500, 1000);

 // List all the available serial ports
 println(Serial.list());
 // I know that the first port in the serial list on my mac
 // is always my  Arduino, so I open Serial.list()[0].
 // Open whatever port is the one you're using.
 myPort = new Serial(this, Serial.list()[1], 9600);
 // don't generate a serialEvent() unless you get a newline character:
 myPort.bufferUntil('\n');
 // set inital background:
 background(0);
 }
 void draw () {
 // everything happens in the serialEvent()
 }

 void serialEvent (Serial myPort) {
 // get the ASCII string:
 String inString = myPort.readStringUntil('\n');

 if (inString != null) {
 // trim off any whitespace:
 inString = trim(inString);
 
 float[] nums = float(split(inString, '|'));
 // convert to an int and map to the screen height:
 float inByte1 = nums[0];
 
 float inByte2= nums[1];
 
 inByte1 = map(inByte1, -1, 1, 0, height);
 inByte2 = map(inByte2, -1, 1, 0, height);

 // draw the point 1:
 stroke(255,0,0);
 strokeWeight(4);
 point(xPos, height - inByte1);
   
 //draw the second point:
  stroke(0,0,255);
 strokeWeight(4);
 point(xPos, height - inByte2);

 // at the edge of the screen, go back to the beginning:
 if (xPos >= width) {
 xPos = 0;
 background(0);
 }
 else {
 // increment the horizontal position:
 xPos++;
 }
 }
 }
