boolean click=false;
PVector sliderLoc = new PVector(50,500);
double motorA, motorC;
boolean enabled=true;
int k=5;
int i=0;
int l = 200;
double minSpeed = 24;
float degrees = 90;
float alpha = degrees * (PI/180);
double kp=1/25,ki=1/10000,kd=1/30;
int motorDelta = 0;
float difY,difX;
int posX,posY;
double sumIY = 0;
double lastY = 0;
 String data = "";
  //String[] list = split(data, ' ');
void setup()
{
  size(800,600);
  posX=125;
  posY=250;
  
 

  // Writes the strings to a file, each on a separate line
  //saveStrings("flightRecord.txt", list);
}
void draw()
{
  
  alpha = degrees * (PI/180); //Update alpha (Radians) with new degrees
  background(200); 
  fill(255,0,0);
  text("X(Red)",100,300);
  text("Push Down Right Motor",90,485);
  text("AngleX: " + degrees, 575,50);
  text("MotorA (Right): " + motorA, 575,100);
  text("MotorC: " + motorC, 575,150);
  stroke(255,0,0);
    
  /*
  sin (alpha) * l = DifferenceY
  Y1 = posY + DifferenceY
  Y2 = posY
  
  cos (alpha) * l = DifferenceX
  X1 = posX + DifferenceX
  X2 = posX
  */
 
  difY = sin(alpha) * l;
  difX = cos(alpha) * l;
  ellipse(posX,posY,4,4);//X axis center point
  line(posX-difX/2,posY-difY/2,posX+difX/2,posY+difY/2);//Quadcopter X Axis
  fill(0,0,0);
  //Border of slider
  
  stroke(0,0,0);
  fill(100);
  rect(50,500,200,20);
  fill(0,0,0);
  
  //End of border of slider
  
  
  
  //Slider
  if(mousePressed==true)
  {
    if(click==true)
    {
      if(mouseX>49&&mouseX<30+200)
      {
      sliderLoc.x=mouseX;
      }
      else
      {
        if(mouseX>30+200)
        {
          sliderLoc.x=30+200;
        }
        else if(mouseX<49)
        {
          sliderLoc.x=50;
        }
      }
    }
    if(mouseX>sliderLoc.x-1&&mouseX<sliderLoc.x+20&&mouseY>sliderLoc.y-1&&mouseY<sliderLoc.y+20)
    {
      click=true;
    }
   
    
  }
  else
  {
    click=false;
  }
    rect(sliderLoc.x,sliderLoc.y,20,20);
  //Until Here - Slider
  
    motorDelta= (int)(sliderLoc.x-50)*2;
    
  i++; 
  pid (0.001, 0, 0, 0);
 if(i<100)
 {
  displayFlight();
 }
 else
 {
   if(enabled)
   {
     //println("random" + random(1));
     int second = second();
     int minute = minute();
     int hour = hour();
     int day= day();
     int month = month();
     int year = year();
     
     String name = "flightRecord" + hour + minute + day + month + year + ".txt";
     
     String[] list = split(data, ' ');
     saveStrings(name, list);
     enabled=false;
   }
 }
 
}

void displayFlight()
{
  data+=" a:" + motorA + "c:" + motorC + "ang:" + degrees;
}

void pid(double time, double minSpeed, double xAngle,double yAngle){
  int start = millis();
 while((millis()-start) < (time * 1000)){


double errorY = yAngle-degrees;
sumIY += errorY * 1/10000;
double pidY = errorY * 1/40 + sumIY - (degrees - lastY) *1/40;
double a = minSpeed - pidY;
double c = minSpeed + pidY;





//print("A:");
motorA= a+24;
//print(a);print("\t");
//print("C:");
motorC=c+24;
//print(c);print("\t");
degrees += (float)((c-a))*0.7;
//writeD(d);
//print(temp);print("\t");

  
//minSpeed-=(1.0/time*(minSpeed-20)/100.0);//Decreases the motors speed in relation to time and minSpeed



lastY = errorY;
/*print("minSpeed: ");
println(minSpeed);*/
delay(6);

}
}
