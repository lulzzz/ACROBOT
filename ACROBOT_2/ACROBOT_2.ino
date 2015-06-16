  /*
  --------------------------------------------------------------------------------------
  **************************************************************************************
  
  ACROBOT Autonomous and Serial Operating Software
  Author: Simon Kalouche
  
  Notes: Robot code is written assuming the lower rack and pinion (LRAP) is aligned
  along the vertical (gravity) vector and the upper rack and pinion (URAP) is aligned
  along the horizontal vector. The lower winch motor is at the bottom or rear of 
  the robot.
  
  **************************************************************************************
  --------------------------------------------------------------------------------------
  */
  
  // Libraries
  #include <Pololu3pi.h>
  #include <PololuQTRSensors.h>
  #include <OrangutanMotors.h>
  #include <OrangutanAnalog.h>
  #include <OrangutanLEDs.h>
  #include <OrangutanLCD.h>
  #include <OrangutanPushbuttons.h>
  #include <OrangutanBuzzer.h>
  #include <OrangutanDigital.h>
//  #include <OrangutanSerial.h>
//  #include <orangutan.h>
  
  
  #define ENC_THRESH 550
  #define CAM_PIN 1            // M1
  #define LRAP_PIN 2           // M5
  #define URAP_PIN 3           // M4
  #define LW_PIN 4             // M3
  #define UW_PIN 5             // M2
  #define PAUSE 500  
  
  // Initialize Objects and Variables
  OrangutanMotors motors;
  OrangutanAnalog analog;
  
  unsigned long serialdata;            // value returned from serial input function
  int inbyte;
  
  int motor_num;                      
  int dir;                            
  int time;
  int deg;
  
  int climb_style;
  int fwd_bkwd;
  int steps;
  
  //*********************** SETUP ***********************************
  
  void setup()
  {
    
  Serial.begin(115200);
  analog.setMode(MODE_10_BIT);
  
  pinMode(IO_B0, OUTPUT);                // Motor 3 + 
  pinMode(IO_B1, OUTPUT);                // Motor 3 - 
  pinMode(IO_B2, OUTPUT);                // Motor 4 + 
  pinMode(IO_B4, OUTPUT);                // Motor 4 - 
  pinMode(IO_D4, OUTPUT);                // Motor 5 + 
  pinMode(IO_D7, OUTPUT);                // Motor 5 -
  pinMode(IO_C1, INPUT_PULLUP);          // S1 (Cam) Encoder
  pinMode(IO_C2, INPUT_PULLUP);          // S5 (LRAP) Encoder
  pinMode(IO_C3, INPUT_PULLUP);          // S4 (URAP) Encoder
  pinMode(IO_C4, INPUT_PULLUP);          // S3 (LW) Encoder
  pinMode(IO_C5, INPUT_PULLUP);          // S2 (UW) Encoder
  pinMode(IO_D0, INPUT);                 // Receive Port RX
  pinMode(IO_D1, OUTPUT);                // Transmit Port TX
  
    /*Alternative syntax to set pins as outputs
      OrangutanDigital::setOutput(IO_C0, HIGH);
      OrangutanDigital::setOutput(IO_C1, LOW); */   
  
  instructions();   
     
  }
  
  
  //*********************** Main Loop ************************************************
  
  
  void loop()
  {
    // Tele-operated Serial Command
    // example serial input: 't/1/1/4/' should actuate motor 1 in CCW direction for 400 ms
    //                       's/v/u/5' should instruct the robot to climb vertically up for 5 steps
    //                       '2/0/360/' should actuate motor 2 in CW direction for 1 rev (360 degrees)
    
    
    
    if (Serial.available() > 0) 
    {
      motor_num=0;
      dir=0;
      time=0;
      climb_style=0;
      fwd_bkwd=0;
      steps=0;
      
       // get first character
      getSerial();  
        switch (serialdata) {
        case 57:          // 'i' == 57 in ASCII
          instructions();
        case 67:          // 's' == 67 in ASCII
        {
          Serial.print("taking step\n");
          // get second character
          getSerial();
          climb_style=serialdata;
          Serial.println(climb_style);
          // get third character
          getSerial();
          fwd_bkwd=serialdata;
          Serial.println(fwd_bkwd);
          // get fourth character
          getSerial();
          steps=serialdata;
          Serial.println(steps);
          
          // run climb/step sequence
          climb(climb_style, fwd_bkwd, steps);
          break;
        }
        
        case 68:       // 't' == 68 in ASCII (for time based motor control)
        { 
          // get second character
          getSerial();
          motor_num=serialdata;
          Serial.println(motor_num);
          // get third character
          getSerial();
          dir=serialdata;
          Serial.println(dir);
          // get fourth character 
          getSerial();
          time=serialdata;
          Serial.println(time);
          
          // run motor controller for serial connection Tele-operation
          motor_controller(motor_num,dir,time);
          break;
        }
        
        default: 
        {
          motor_num=serialdata;
          Serial.println(motor_num);
          // get second character
          getSerial();
          dir=serialdata;
          Serial.println(dir);
          // get third character 
          getSerial();
          deg=serialdata;
          Serial.println(deg);
          
          // run motor controller for serial connection Tele-operation
          motor_controller_enc(motor_num,dir,deg);
          break;
        }
        }
    }
    
  }
  
  
  
  
  //*********************** Serial Command ****************************************
  
  long getSerial()
      {
        serialdata = 0;
        while (inbyte != '/')
        {
          inbyte = Serial.read(); 
          if (inbyte > 0 && inbyte != '/')
          {
           serialdata = serialdata * 10 + inbyte - '0';
            //serialdata = inbyte;
          }
        }
        inbyte = 0;
        return serialdata;
      }
    
    
    
    
    
  //*********************** Autonomous Climbing **************************************

  void climb(int climb_style, int fwd_bkwd, int steps)
  {
    // function parameters:
    // climb_style: robot will climb vertically or horizontally [v,h]
    // fwd_bkwd: direction the robot will travel (i.e. up, down, left, right) [u,d,l,r]
    // step: number of steps the robot should take [##]
    
    int n=0;     // step counter
    
    
    switch (climb_style)
    {
    // climb along vertical (gravity) vector-------------------------------------------------
    // 'v'==70 in ASCII
    case 70: {
      
      switch (fwd_bkwd){
        
        // climb up/forward
        // 'u'==69 in ASCII
        case 69: {
          
          for (n = 1; n <= steps; n++)
          {
          motor_controller_enc(5,0,200);       // actuate LRAP (upper pads move up)
          delay(PAUSE);
          motor_controller_enc(1,1,90);        // cam lowers lower pads
          delay(10);
          motor_controller_enc(1,1,30);        // preload lower pads
          delay(0);
          motor_controller_enc(3,0,90);        // lower winch loosens to turn lower pads ON
          delay(0);
          motor_controller_enc(2,1,90);        // upper winch tightens to turn upper pads OFF
          delay(PAUSE);
          motor_controller_enc(1,1,60);        // cam lifts upper pads
          delay(PAUSE);
          motor_controller_enc(5,1,400);       // actuate LRAP (lower pads move up)
          delay(PAUSE);
          motor_controller_enc(1,0,90);        // cam lowers upper pads
          delay(10);
          motor_controller_enc(1,0,30);        // preload upper pads
          delay(0);
          motor_controller_enc(2,0,90);        // upper winch loosens to turn upper pads ON
          delay(0);
          motor_controller_enc(3,1,120);        // lower winch tightens to turn lower pads OFF
          delay(PAUSE);
          motor_controller_enc(1,0,60);        // cam lifts lower pads up
          delay(PAUSE);
          motor_controller_enc(5,0,200);        // actuate LRAP (upper pads move up)
          delay(PAUSE);
          }
          
          break;  }
          
          // climb down/backwards 
          // 'd'==52 in ASCII
          case 52: {
          
          for (n = 1; n <= steps; n++)
          {
          motor_controller(4,1,15);       // actuate URAP (lower pads move down)
          delay(2000);
          motor_controller(3,1,5);        // lower winch tightens
          delay(500);
          motor_controller(1,1,3);        // cam lowers lower pads
          delay(500);
          motor_controller(1,1,1);        // preload lower pads
          delay(500);
          motor_controller(3,0,5);        // lower winch loosens to turn lower pads ON
          delay(500);
          motor_controller(2,1,5);        // upper winch tightens to turn upper pads OFF
          delay(500);
          motor_controller(1,1,1);        // cam lifts upper pads
          delay(500);
          motor_controller(4,0,30);       // actuate URAP (upper pads move down)
          delay(3000);
          motor_controller(2,0,5);        // upper winch tightens
          delay(500);
          motor_controller(1,0,3);        // cam lowers upper pads
          delay(500);
          motor_controller(1,0,1);        // preload upper pads
          delay(500);
          motor_controller(2,1,5);        // upper winch loosens to turn upper pads ON
          delay(500);
          motor_controller(3,1,5);        // lower winch tightens to turn lower pads OFF
          delay(500);
          motor_controller(1,0,1);        // cam lifts lower pads up
          delay(500);
          }
          
          break;  }
          
        default: {
          // do nothing
          break;  }  
      }}
      
     
      
      // Climb along horizontal vector --------------------------------------------------
      // 'h'==56 in ASCII
      case 56: {
      
      switch (fwd_bkwd){
        
        // climb right
        // 'r'==66 in ASCII
        case 66: {
          
          for (n = 1; n <= steps; n++)
          {
          motor_controller(5,1,15);       // actuate LRAP (upper pads move right)
          delay(2000);
          motor_controller(3,1,5);        // lower winch tightens
          delay(500);
          motor_controller(1,1,3);        // cam lowers lower pads
          delay(500);
          motor_controller(1,1,1);        // preload lower pads
          delay(500);
          motor_controller(3,0,5);        // lower winch loosens to turn lower pads ON
          delay(500);
          motor_controller(2,1,5);        // upper winch tightens to turn upper pads OFF
          delay(500);
          motor_controller(1,1,1);        // cam lifts upper pads
          delay(500);
          motor_controller(5,0,25);       // actuate LRAP (lower pads move right)
          delay(3000);
          motor_controller(2,0,5);        // upper winch tightens
          delay(500);
          motor_controller(1,0,3);        // cam lowers upper pads
          delay(500);
          motor_controller(1,0,1);        // preload upper pads
          delay(500);
          motor_controller(2,1,5);        // upper winch loosens to turn upper pads ON
          delay(500);
          motor_controller(3,1,5);        // lower winch tightens to turn lower pads OFF
          delay(500);
          motor_controller(1,0,1);        // cam lifts lower pads up
          delay(500);
          }
          
          break;  }
          
          // climb left
          // 'l'==60 in ASCII
          case 60: {
          
          for (n = 1; n <= steps; n++)
          {
          motor_controller(5,0,10);       // actuate LRAP (upper pads move left)
          delay(2000);
          motor_controller(3,1,5);        // lower winch tightens
          delay(500);
          motor_controller(1,1,3);        // cam lowers lower pads
          delay(500);
          motor_controller(1,1,1);        // preload lower pads
          delay(500);
          motor_controller(3,0,5);        // lower winch loosens to turn lower pads ON
          delay(500);
          motor_controller(2,1,5);        // upper winch tightens to turn upper pads OFF
          delay(500);
          motor_controller(1,1,1);        // cam lifts upper pads
          delay(500);
          motor_controller(5,1,25);       // actuate LRAP (lower pads move left)
          delay(3000);
          motor_controller(2,0,5);        // upper winch tightens
          delay(500);
          motor_controller(1,0,3);        // cam lowers upper pads
          delay(500);
          motor_controller(1,0,1);        // preload upper pads
          delay(500);
          motor_controller(2,1,5);        // upper winch loosens to turn upper pads ON
          delay(500);
          motor_controller(3,1,5);        // lower winch tightens to turn lower pads OFF
          delay(500);
          motor_controller(1,0,1);        // cam lifts lower pads up
          delay(500);
          }
          
          break;  }
          
        default: {
          // do nothing
          break;  }  
      }}
    }
  }
  
  
  //*********************** Motor Control Function (Time Based)***********************************
  
  void motor_controller(int motor_num, int dir, int time) 
  {
    // function parameters:
    // motor_num: motor to be actuated [1,2,3,4,5]
    // dir: direction to actuate the motor in [CW=0 or CCW=1]
    // time: duration of time the motor should be actuated for [ms]
    
    // eventually convert time to degrees when encoders are working 
    // deg: degrees to spin motor (360 deg=1 revolution) 
     
    // motor numbers: 
    // 1: Cam,                       motor channel 1
    // 2: Upper Winch,               motor channel 2
    // 3: Lower Winch,               IO_B0/IO_B1
    // 4: Upper Rack and Pinion,     IO_B2/IO_B4
    // 5: Lower Rack and Pinion,     IO_D4/IO_D7

    // convert mili seconds into tenths of seconds
    time=time*100;
    
    switch (motor_num)
    {
      case 1: {
      //actuate motor 1: cam
       if (dir==0)
        {
          motors.setSpeeds(255,0);
          delay(time);
          motors.setSpeeds(0,0);
          break;
        }
        else if (dir==1)
        {
          motors.setSpeeds(-255,0);
          delay(time);
          motors.setSpeeds(0,0);
          break;
        }
        else 
      break; }
      
      
      case 2: {
      //actuate motor 2: upper winch
       if (dir==0)
        {
          motors.setSpeeds(0,255);
          delay(time);
          motors.setSpeeds(0,0);
          break;
        }
        else if (dir==1)
        {
          motors.setSpeeds(0,-255);
          delay(time);
          motors.setSpeeds(0,0);
          break;
        }
        else 
      break; }
      
      case 3: {
      //actuate motor 3: lower winch
        if (dir==0)
        {
          analogWrite(IO_B0, 255);
          analogWrite(IO_B1, LOW);
          delay(time);
          analogWrite(IO_B0, 0);
          analogWrite(IO_B1, 0);
          break;
        }
        else if (dir==1)
        {
          analogWrite(IO_B0, LOW);
          analogWrite(IO_B1, 255);
          delay(time);
          analogWrite(IO_B0, 0);
          analogWrite(IO_B1, 0);
          break;
        }
        else 
      break; }
      
      case 4: {
      // actuate motor 4: upper rack and pinion
       if (dir==0)
        {
          analogWrite(IO_B2, 255);
          analogWrite(IO_B4, LOW);
          delay(time);
          analogWrite(IO_B2, 0);
          analogWrite(IO_B4, 0);
          break;
        }
        else if (dir==1)
        {
          analogWrite(IO_B2, LOW);
          analogWrite(IO_B4, 255);
          delay(time);
          analogWrite(IO_B2, 0);
          analogWrite(IO_B4, 0);
          break;
        }
        else 
      break; }
      
      case 5: {
      //actuate motor 5: lower rack and pinion
      if (dir==0)
        {
          analogWrite(IO_D4, 255);
          analogWrite(IO_D7, LOW);
          delay(time);
          analogWrite(IO_D4, 0);
          analogWrite(IO_D7, 0);
          break;
        }
        else if (dir==1)
        {
          analogWrite(IO_D4, LOW);
          analogWrite(IO_D7, 255);
          delay(time);
          analogWrite(IO_D4, 0);
          analogWrite(IO_D7, 0);
          break;
        }
        else 
      
      break; }
            
      default: {
      // do nothing
      break; }
  
    }
  }
  
  

 //*********************** Motor Control Function (Encoder Based)***********************************
  
   void motor_controller_enc(int motor_num, int dir, int deg) 
  {
    // function parameters:
    // motor_num: motor to be actuated [1,2,3,4,5]
    // dir: direction to actuate the motor in [CW=0 or CCW=1]
    // deg: degrees to spin motor (360 deg=1 revolution) 
     
    // motor numbers: 
    // 1: Cam,                       motor channel 1
    // 2: Upper Winch,               motor channel 2
    // 3: Lower Winch,               IO_B0/IO_B1
    // 4: Upper Rack and Pinion,     IO_B2/IO_B4
    // 5: Lower Rack and Pinion,     IO_D4/IO_D7

    // convert mili seconds into tenths of seconds
    
    switch (motor_num)
    {
      case 1: {
      //actuate motor 1: cam
       if (dir==0)
        {
          motors.setSpeeds(255,0);
          encoder(CAM_PIN, deg);
          motors.setSpeeds(0,0);
          break;
        }
        else if (dir==1)
        {
          motors.setSpeeds(-255,0);
          encoder(CAM_PIN, deg);
          motors.setSpeeds(0,0);
          break;
        }
        else 
      break; }
      
      
      case 2: {
      //actuate motor 2: upper winch
       if (dir==0)
        {
          motors.setSpeeds(0,255);
          encoder(UW_PIN, deg);
          motors.setSpeeds(0,0);
          break;
        }
        else if (dir==1)
        {
          motors.setSpeeds(0,-255);
          encoder(UW_PIN, deg);
          motors.setSpeeds(0,0);
          break;
        }
        else 
      break; }
      
      case 3: {
      //actuate motor 3: lower winch
        if (dir==0)
        {
          analogWrite(IO_B0, 255);
          analogWrite(IO_B1, LOW);
          encoder(LW_PIN, deg);
          analogWrite(IO_B0, 0);
          analogWrite(IO_B1, 0);
          break;
        }
        else if (dir==1)
        {
          analogWrite(IO_B0, LOW);
          analogWrite(IO_B1, 255);
          encoder(LW_PIN, deg);
          analogWrite(IO_B0, 0);
          analogWrite(IO_B1, 0);
          break;
        }
        else 
      break; }
      
      case 4: {
      // actuate motor 4: upper rack and pinion
       if (dir==0)
        {
          analogWrite(IO_B2, 255);
          analogWrite(IO_B4, LOW);
          encoder(URAP_PIN, deg);
          analogWrite(IO_B2, 0);
          analogWrite(IO_B4, 0);
          break;
        }
        else if (dir==1)
        {
          analogWrite(IO_B2, LOW);
          analogWrite(IO_B4, 255);
          encoder(URAP_PIN, deg);
          analogWrite(IO_B2, 0);
          analogWrite(IO_B4, 0);
          break;
        }
        else 
      break; }
      
      case 5: {
      //actuate motor 5: lower rack and pinion
      if (dir==0)
        {
          analogWrite(IO_D4, 255);
          analogWrite(IO_D7, LOW);
          encoder(LRAP_PIN, deg);
          analogWrite(IO_D4, 0);
          analogWrite(IO_D7, 0);
          break;
        }
        else if (dir==1)
        {
          analogWrite(IO_D4, LOW);
          analogWrite(IO_D7, 255);
          encoder(LRAP_PIN, deg);
          analogWrite(IO_D4, 0);
          analogWrite(IO_D7, 0);
          break;
        }
        else 
      
      break; }
            
      default: {
      // do nothing
      break; }
  
    }
  }
  
  
  
 //*********************** Encoder Function ***********************************
   
  void encoder(int pin_num, long deg)
  {
    // define variables
    long ticks;
    
    int n_tick=0;
    int old_tick_val=0;
    int tick_val=0;
    int sensor;
    
    // convert input degrees into encoder ticks
    // every rotation of the 3-tooth encoder wheel should generate 6 ticks
    // if statement differentiates between motors with different gearbox ratios
    if (pin_num == URAP_PIN || pin_num == LRAP_PIN || pin_num == CAM_PIN )
    {
      ticks=6*1000*deg/360;
    }
    else
    {
      ticks=6*298*deg/360;
    }
    
    while (n_tick<= ticks)
    {
      sensor=analogRead(pin_num); // 0 = IO_C0
      if (sensor>=550)
      { 
        tick_val=0;  
        if (old_tick_val==1)
        {
          n_tick++;
          old_tick_val=0;
        }
        else ;
      }
    
      if (sensor<550)
      {
        tick_val=1;
        if (old_tick_val==0)
        {
          n_tick++;
          old_tick_val=1;
        }
        else ;
      }
      
      //Serial.println(sensor);
    } 
      
    }
    
  //*********************** INSTRUCTIONS ************************************
  void instructions()
  {
   Serial.println("Tele-operated Serial Command Instructions: \n");
   Serial.println("function parameters: ");
   Serial.println("motor_num: motor to be actuated [1,2,3,4,5]");
   Serial.println("dir: direction to actuate the motor in [CW=0 or CCW=1]");
   Serial.println("deg: degrees to spin motor (360 deg=1 revolution)\n");  
   Serial.println("motor numbers: ");
   Serial.println("\t1: Cam");         
   Serial.println("\t2: Upper Winch");               
   Serial.println("\t3: Lower Winch");              
   Serial.println("\t4: Upper Rack and Pinion");
   Serial.println("\t5: Lower Rack and Pinion");
   Serial.println("(convert mili seconds into tenths of seconds)\n");
   Serial.println("example serial input: ");
   Serial.println("\t t/1/1/4/ actuate motor 1 in CCW direction for 400 ms");
   Serial.println("\t s/v/u/5 instruct the robot to climb vertically up for 5 steps");
   Serial.println("\t 2/0/360/ actuate motor 2 in CW direction for 1 rev (360 degrees)");  
   Serial.println("\t i/i/i/i/ for instructions"); 
  }
    
