import processing.serial.*;
Serial myPort;

void setup()
{
  //frameRate(10);
  myPort = new Serial(this, "COM4", 9600);
   
   
}

void draw()
{
 myPort.write("0");
}
