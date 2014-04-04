boolean click=false;
PVector sliderLoc = new PVector(50,500);
int k=5;
int l = 200;
float degrees = 0;
float alpha = degrees * (PI/180);
int kP=0,kI=0,kD=0;
int motorLeft = 0;
float difY,difX;
int posX,posY;

void setup()
{
  size(800,600);
  posX=125;
  posY=250;
}
void draw()
{
  alpha = degrees * (PI/180); //Update alpha (Radians) with new degrees
  background(200); 
  fill(255,0,0);
  text("X(Red)",100,300);
  text("Push Down Right Motor",90,485);
  text("AngleX: " + degrees, 700,50);
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
  
    motorLeft= (int)sliderLoc.x-50;
    degrees += motorLeft/15;
    
   
    degrees = degrees % 361;//Displays degrees until 361
  
 
}

