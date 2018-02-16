import processing.serial.*;

PShape rocket;
float ry;

int lf = 10;    //corresponde ao \n
int flag = 0;   //Sinalização para calibração
String myString = null;
Serial myPort;  // serial port

int pos_x,pos_y,pos_z;

float ang_x = 0,ang_y = 0,ang_z = 0;

int cont = 0;
float cont2 = 0;

void setup() {
  //size(640, 360, P3D);
  size(800, 600, P3D);
  rocket = loadShape("rocket.obj");
  // List all the available serial ports
  //printArray(Serial.list());
  // Open the port you are using at the rate you want:
  myPort = new Serial(this, Serial.list()[0], 250000);
  myPort.clear();
  // Throw out the first reading, in case we started reading 
  // in the middle of a string from the sender.
  myString = myPort.readStringUntil(lf);
  myString = null;
}

void draw() 
{
  if (myPort.available() > 0) 
  {
    myString = myPort.readStringUntil(lf);
    if (myString != null) {
      
      //println(myString);
                
        pos_x = myString.indexOf("X");
        pos_y = myString.indexOf("Y");
        pos_z = myString.indexOf("Z");
      
        if ((pos_x > 0) || (pos_y > 0) || (pos_z > 0) )
        {
            ang_x = float( myString.substring(0,pos_x        ) ); 
            ang_y = float( myString.substring(pos_x + 1,pos_y) );
            ang_z = float( myString.substring(pos_y + 1,pos_z) );
               
            println();
            println(ang_x);
            println(ang_y);
            println(ang_z); 
            println();
            
            background(0);
            lights();
            translate(width/2, height/2 + 100, -200);
            
            rotateX(PI);
            rotateY(0);
            rotateZ(-PI/2);
            rotateZ(PI/2);
            
            
            rotateY(round(ang_x)*PI/180.0);
            
            //10% OK!
            rotateX(round(ang_y)*PI/180.0);
            rotateZ(round(ang_z)*PI/180);
          
            shape(rocket);
        } //<>//
    }  
  }
}