#include <Servo.h>
//Includes library containing relevant servo functions

Servo jaws,base,arm,forearm;
//Creates objets for each servo
/* jaw - controls servo which moves the pinchers
 * base - controls left to right movement
 * arm - controls the arm attached directly to the base
 * forearm - controls the smaller section attached to the arm
 */

#define joy1_x 0
#define joy1_y 1
#define joy2_x 2
#define joy2_y 3
/*  Initialize constants for joystick input pin numbers
    analog pins 0-3      */
#define joy1_press 2
#define joy2_press 3
/*  Initialize constants for joystick switch input pin numbers
    digital pins 2-3     */

#define PLAY 12
#define REC 13
/*  Initialize constants for button inputs  
    digital pins 12-13   */

double jaws_out = 0;
double base_out = 90;
double arm_out = 90;
double forearm_out = 90;
// Initialize servo angle output angles

double center;
// Joystick center reference point

double max_speed = 1; 
// Maximum speed at which servo angle changes. Degree per loop.

double drift_ratio = 0.25;
// Minimum deflection of joystick required to cause movement.

String bar = "-------------------------";
// visually separates serial monitor outputs

const int arrLength = 100;
int baser[arrLength],jawsr[arrLength],armr[arrLength],forearmr[arrLength]; 
  //allocate storage for position data

int t = 0;

void setup() {
  pinMode(joy1_press, INPUT_PULLUP);
  pinMode(joy2_press, INPUT_PULLUP);
  digitalWrite(joy1_press, HIGH);
  digitalWrite(joy2_press, HIGH);

  pinMode(REC, INPUT_PULLUP);
  pinMode(PLAY, INPUT_PULLUP);

  jaws.attach(6); // Connect jaws servo to pin 6
  base.attach(9); // Connect base servo to pin 9
  arm.attach(10); // Connect arm servo to pin 10
  forearm.attach(11); // Connect forearm servo to pin 11

  jaws.write(jaws_out);
  base.write(base_out);
  arm.write(arm_out);
  forearm.write(forearm_out);
  // Start each servo in a neutral position
 
  Serial.begin(9600);
  center = 512;
  // Define center value of the joystick as 512

 for (int i = 0; i < arrLength; i++) {
  //Initialize recording array values to a default recording in neutral position
  jawsr[i] = 0;
  baser[i] = 70;
  armr[i] = 90;
  forearmr[i] = 90;
 }
}


double joy_jaws,joy_base,joy_arm,joy_forearm;
  // Initialize joystick input variables

double jaws_speed, base_speed, arm_speed, forearm_speed;

double jawsprev, baseprev, armprev, forearmprev;
  //Initialize the values of the previous loop iteration's speed for jaws, base, arm, and forearm servos, used in motionpoint

bool isRecording = false; 
  //Initialize boolean value for movement recording

bool isPlaying = false;
  //Initialize boolean value for recording playback

bool increase = true;
  //Initialize boolean value for boomerang feature

bool checkpoint;
  //Initialize boolean determining the beginnings & ends of new servo motions

void loop() {
 
    if (digitalRead(REC) == LOW) {
    Serial.println(bar);
    if (isRecording) {
      // Button is pressed after recording was started
      isRecording = false; //Stop recording
      Serial.println("Recording stopped");
    } 
    else if (isPlaying) {
      // Button was pressed after playback was started
      for (int i = 0; i < arrLength; i++) {
        //Initialize recording array values to a default recording in neutral position
        jawsr[i] = 0;
        baser[i] = 70;
        armr[i] = 90;
        forearmr[i] = 90;
       }
      isPlaying = false; //Playback stops
      isRecording = true; //Begin recording
      Serial.println("Playback stopped, now recording...");
    }
    else {
      // Button was pressed to start a recording
      for (int i = 0; i < arrLength; i++) {
        //Initialize recording array values to a default recording in neutral position
        jawsr[i] = 0;
        baser[i] = 70;
        armr[i] = 90;
        forearmr[i] = 90;
       }
      isRecording = true; //Begin recording
      Serial.println("Now recording...");
    }
    delay(1000);
    t = 0; // place recording pointer at beginning of the array
  }

  if (digitalRead(PLAY) == LOW) {
    t = 0; //Place recording pointer at beginning of array
    Serial.println(bar);
    if (isPlaying) {
      // Button was pressed after playback started
      isPlaying = false; //Stop playback
      Serial.println("Playback stopped");
    } 
    else if (isRecording) {
      //Button was pressed after recording was begun
      isRecording = false; //Stop recording
      isPlaying = true; //Play what was just recorded
      Serial.println("Recording stopped, now beginning playback...");
      Serial.println("Locating starting position...");
      goToAll(jawsr[t],baser[t],armr[t],forearmr[t]);
      delay(100);
      Serial.println("Now playing. Press PLAY to stop or RECORD to begin a new recording");
    }
    else {
      // Button was pressed to play previously stored playback
      isPlaying = true; //Begin playback
      Serial.println("Beginning Playback...");
      Serial.println("Locating starting position...");
      goToAll(jawsr[t],baser[t],armr[t],forearmr[t]);
      delay(100);
      Serial.println("Now playing. Press PLAY to stop or RECORD to begin a new recording");
    }
    delay(1000);
  }
      

  if (isPlaying) {
    if (t == 0) {
      goToAll(jawsr[t],baser[t],armr[t],forearmr[t]);
    }
    jaws_out = jawsr[t];
    base_out = baser[t];
    arm_out = armr[t];
    forearm_out = forearmr[t];
    //Set servo positions to stored values from recording arrays
    goToAll(jaws_out,base_out,arm_out,forearm_out);
    //BOOMERANG
    //Once end of recording is done, recording plays in reverse
//    if (increase) {
//      t++;
//      if(t>=arrLength) {
//        t--;
//        increase = false;
//      }
//    } else {
//      t--;
//      if (t<=0) {
//        t++;
//        increase = true;
//      }
//    }
    //REPEAT FROM BEGINNING   
    t++; // Move to the next position in the arrays
    if (t>=arrLength) {t=0;} //Continuously replay until PLAY button is pushed again
    
  } 
  else {
    joy_base = -(analogRead(joy1_x)-center); 
    //Left analog stick horizontal controls left right movement
    joy_arm = -(analogRead(joy1_y)-center);
    //Left analog stick vertical controls major arm movements
    joy_forearm = -(analogRead(joy2_y)-center);
    //Right analog stick vertical controls minor arm movements
    joy_jaws = analogRead(joy2_x)-center;
    //Right analog stick horizontal controls opening and closing of jaws
    /*  Joystick inputs are shifted by 1023/2 in order place the 
        origin of input values at the joystick center rather than 
        the NW corner  */
    base_speed = frac_map(joy_base, -center, center, -max_speed, max_speed);
    if (abs(base_speed) < max_speed*drift_ratio) {
      base_speed = 0;
    }
    arm_speed = frac_map(joy_arm, -center, center, -max_speed, max_speed);
    if (abs(arm_speed) < max_speed*drift_ratio) {
      arm_speed = 0;
    }
  
    forearm_speed = frac_map(joy_forearm, -center, center, -max_speed, max_speed);
    if (abs(forearm_speed) < max_speed*drift_ratio) {
      forearm_speed = 0;
    }
  
    jaws_speed = frac_map(joy_jaws, -center, center, -max_speed, max_speed);
    if (abs(jaws_speed) < max_speed*drift_ratio) {
      jaws_speed = 0;
    }
   
    /*  Determine the speed at which the servos will move
        and prevent movement if joystick deflection is below
        one fourth of maximum deflection to one side   */
  
    jaws_out = jaws_out + jaws_speed;
    base_out = base_out + base_speed;
    arm_out = arm_out + arm_speed;
    forearm_out = forearm_out + forearm_speed;
    // change servo position according to amount of joystick deflection
  
    if (jaws_out < 0) {jaws_out = 0;}
    if (jaws_out >180) {jaws_out = 180;}
    if (base_out < 0) {base_out = 0;}
    if (base_out >180) {base_out = 180;}
    if (arm_out < 30) {arm_out = 30;}
    if (arm_out >180) {arm_out = 180;}
    if (forearm_out < 0) {forearm_out = 0;}
    if (forearm_out >180) {forearm_out = 180;}
    // Keep output values within the range of 0 to 180
    
    if (forearm_out < 90-arm_out) {forearm_out = 90-arm_out;}
    // Prevent movement of forearm past arm such that it becomes stuck
      if (digitalRead(joy1_press) == LOW) {
      reset();
      while(digitalRead(joy1_press) == LOW){}
    }
  // Check for reset button     
    
  }


  
  jaws.write(jaws_out);
  base.write(base_out);
  arm.write(arm_out);
  forearm.write(forearm_out);
    // Output servo angles 

  checkpoint = ((base_speed!=0) && (baseprev==0)) || ((base_speed==0) && (baseprev!=0))
                || ((jaws_speed!=0) && (jawsprev==0)) || ((jaws_speed==0) && (jawsprev!=0)) 
                || ((forearm_speed!=0) && (forearmprev==0)) || ((forearm_speed==0) && (forearmprev!=0)) 
                || ((arm_speed!=0) && (armprev==0)) || ((arm_speed==0) && (armprev!=0));
 
  if (isRecording && t<arrLength && checkpoint) {
    /* If recording has started, there is memory left, and a servo has started or stopped moving, 
     * position data will be stored
     */
    jawsr[t] = jaws_out;
    baser[t] = base_out;
    armr[t] = arm_out;
    forearmr[t] = forearm_out;
    t++;
  } else if (isRecording && t>= arrLength) {
    /* If recording has started and there is no more memory
     * then the recording will end and the memory pointer is set to 0
     * so that the next recording overwrites previous data.
     */
    isRecording = false;
    Serial.println(bar);
    Serial.println("Memory full, recording stopped.");
    t = 0;
  }

  baseprev=base_speed;
  armprev=arm_speed;
  forearmprev=forearm_speed;
  jawsprev=jaws_speed;
  
  delay(10);
  // wait 10 ms
}

double frac_map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void reset() {
  Serial.println(bar);
  Serial.println("Resetting Position");
    jaws_out = goTo(jaws, 0);
    arm_out = goTo(arm, 90);
    forearm_out = goTo(forearm,90);
    base_out = goTo(base, 70);
  Serial.println("Position Reset");
  // While the left joystick is pressed, reset to normal position
}

//Smoothly move a single servo to a final position
int goTo(Servo serv, int pos) {
  String moving = "Servo moving to ";
  String output = moving + pos;
  Serial.println(output);
  int posInitial = serv.read();
  while(posInitial != pos) {
    if(posInitial<pos) {
      serv.write(posInitial++);
    }
    else {
      serv.write(posInitial--);
    }
    delay(10);
  }
  delay(100);
  String done = "Servo at ";
  output = done + pos;
  Serial.println(output);
  return pos;
}

//Move all servos at once to a specific position
void goToAll(int jawsPos, int basePos, int armPos, int forePos) {
/* Currently moves all four servos one degree at a time in 
 * the necessary direction. */
 
  int jawsInit = jaws.read();
  int baseInit = base.read();
  int armInit = arm.read();
  int foreInit = forearm.read();
  int jawsDif = jawsPos - jawsInit;
  int baseDif = basePos - baseInit;
  int armDif = armPos - armInit;
  int foreDif = forePos - foreInit;

  
  while (jawsInit != jawsPos
        || baseInit != basePos
        || armInit != armPos 
        || foreInit != forePos) {
    if (digitalRead(PLAY) == LOW || digitalRead(REC) == LOW) {
      jaws_out = jawsInit;
      base_out = baseInit;
      arm_out = armInit;
      forearm_out = foreInit;
      break;
    }
    if (jawsInit != jawsPos) {
      jawsInit = jawsInit + jawsDif/abs(jawsDif);
      jaws.write(jawsInit);
    }
    if (baseInit != basePos) {
      baseInit = baseInit + baseDif/abs(baseDif);
      base.write(baseInit);
    }
    if (armInit != armPos) {
      armInit = armInit + armDif/abs(armDif);
      arm.write(armInit);
    }
    if (foreInit != forePos) {
      foreInit = foreInit + foreDif/abs(foreDif);
      forearm.write(foreInit);
    }
    delay(10);
  }
}
