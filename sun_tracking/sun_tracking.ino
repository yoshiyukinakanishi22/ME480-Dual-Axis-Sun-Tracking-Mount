#include <Arduino.h>
#include <Stepper.h>


// defines pins
// FL Forward Left 
// AL Aft Left
// FR Forward Right
// AR Aft Right
// Coordinate System defined from the middle of the hinge 
#define ldrFLPin  27
#define ldrALPin  32
#define ldrFRPin  35
#define ldrARPin  26
#define ldrLaUPPin   25
#define ldrLaDOWNPin 33
//12 (Right)  27, 26   
//34 (Top)   25, 33    
//56 (Left)  32, 35    

// 35, 25 top two LDRs
// 32, 33 middle two LDRs
// 26, 27 bottom two LDRssx
#define rotStepPin 13
#define rotDirPin 14
#define linactENA 15
#define linactIN1 4
#define linactIN2 2



int ldrFLStatus;
int ldrALStatus;
int ldrFRStatus;
int ldrARStatus;
int ldrLaUpStatus;
int ldrLaDownStatus;
int rpm = 400;

Stepper myStepper (100, rotStepPin, rotDirPin); //set STEPS to 100 if error

int size = 3; //change to 10?
int readIndex = 0;
int readingsFL[5];
int readingsAL[5];
int readingsFR[5];
int readingsAR[5];
int readingsLaUP[5];
int readingsLaDOWN[5];
int totalFL = 0;
float averageFL = 0;
int totalAL = 0;
float averageAL = 0;
int totalFR = 0;
float averageFR = 0;
int totalAR = 0;
float averageAR = 0;
int totalLaUP = 0;
float averageLaUP = 0;
int totalLaDOWN = 0;
float averageLaDOWN = 0;

void setup() {
  // Sets the two pins as Outputs
  //Serial.begin(115200);
  Serial.begin(115200);
  pinMode(ldrFLPin, INPUT);
  pinMode(ldrALPin, INPUT);
  pinMode(ldrFRPin, INPUT);
  pinMode(ldrARPin, INPUT);
  pinMode(ldrLaUPPin, INPUT);
  pinMode(ldrLaDOWNPin, INPUT);
  pinMode(rotStepPin, OUTPUT);
  pinMode(rotDirPin, OUTPUT);
  pinMode(linactENA, OUTPUT);
  pinMode(linactIN1, OUTPUT);
  pinMode(linactIN2, OUTPUT);
  analogSetAttenuation(ADC_11db);
  //myStepper.setSpeed(rpm);
  
  // Initialize all readings to 0
  for (int i = 0; i < size; i++) {
    readingsFL[i] = 0;
    readingsAL[i] = 0;
    readingsFR[i] = 0;
    readingsAR[i] = 0;
    readingsLaUP[i] = 0;
    readingsLaDOWN[i] = 0;
  }
  digitalWrite(linactENA, HIGH);
}

void loop() {
  // read ldr value
  ldrFLStatus = analogRead(ldrFLPin);
  ldrALStatus = analogRead(ldrALPin);
  ldrFRStatus = analogRead(ldrFRPin);
  ldrARStatus = analogRead(ldrARPin);
  ldrLaUpStatus = analogRead(ldrLaUPPin);
  ldrLaDownStatus = analogRead(ldrLaDOWNPin);

  // Take the Moving Average of the last 5 readings
  totalFL = totalFL - readingsFL[readIndex];
  readingsFL[readIndex] = ldrFLStatus;
  totalFL = totalFL + readingsFL[readIndex];
  averageFL = totalFL / float(size);

  totalAL = totalAL - readingsAL[readIndex];
  readingsAL[readIndex] = ldrALStatus;
  totalAL = totalAL + readingsAL[readIndex];
  averageAL = totalAL / float(size);


  totalFR = totalFR - readingsFR[readIndex];
  readingsFR[readIndex] = ldrFRStatus;
  totalFR = totalFR + readingsFR[readIndex];
  averageFR = totalFR / float(size);

  totalAR = totalAR - readingsAR[readIndex];
  readingsAR[readIndex] = ldrARStatus;
  totalAR = totalAR + readingsAR[readIndex];
  averageAR = totalAR / float(size);

  totalLaUP = totalLaUP - readingsLaUP[readIndex];
  readingsLaUP[readIndex] = ldrLaUpStatus;
  totalLaUP = totalLaUP + readingsLaUP[readIndex];
  averageLaUP = totalLaUP / float(size);

  totalLaDOWN = totalLaDOWN - readingsLaDOWN[readIndex];
  readingsLaDOWN[readIndex] = ldrLaDownStatus;
  totalLaDOWN = totalLaDOWN + readingsLaDOWN[readIndex];
  averageLaDOWN = totalLaDOWN / float(size);
  
  delay(10);
  
  readIndex = (readIndex + 1) % size;
  int diffLeft = averageFL - averageAL;
  int diffRight = averageFR - averageAR;
  int dod = diffLeft - diffRight;
  int diffLA = averageLaUP - averageLaDOWN;

  if (dod >= 300){
     // When dod is positive, that means left side is getting more sunlight
      Serial.print("Out of Range DOD Pos");	    
      Serial.println(dod);
      Serial.print("34: "); Serial.println(averageFL);
      Serial.print("35: "); Serial.println(averageAL);
      Serial.print("32: "); Serial.println(averageFR);
      Serial.print("33: "); Serial.println(averageAR);
      myStepper.step(1);
    }
  else if (dod <= -700){
      Serial.print("Out of Range DOD Neg");	    
      Serial.println(dod);
      Serial.print("34: "); Serial.println(averageFL);
      Serial.print("35: "); Serial.println(averageAL);
      Serial.print("32: "); Serial.println(averageFR);
      Serial.print("33: "); Serial.println(averageAR);
    	myStepper.step(-1);
  }
  else{
    Serial.println("Within Range");
    Serial.print("34: "); Serial.println(averageFL);
    Serial.print("35: "); Serial.println(averageAL);
    Serial.print("32: "); Serial.println(averageFR);
    Serial.print("33: "); Serial.println(averageAR);
    
  }

  if (diffLA >= 400){
    // Extend LA
    Serial.println("Retracting");
    Serial.println(averageLaUP);
    Serial.println(averageLaDOWN);
    goBackward();
  }
  else if (diffLA <= -400){
    // Retracts LA
    Serial.println("Extending");
    Serial.println(averageLaUP);
    Serial.println(averageLaDOWN);
    goForward();
  }
  else{
    // Stop Actuator
    // digitalWrite(laSpeedPin, HIGH);
    // digitalWrite(laSpeedPin, HIGH);
    Serial.println("Stay Same");
    Serial.println(averageLaUP);
    Serial.println(averageLaDOWN);
    stay();
  }
  }


void goForward(){
  digitalWrite(linactIN1, LOW);
  digitalWrite(linactIN2, HIGH);
}

void goBackward(){
  digitalWrite(linactIN1, HIGH);
  digitalWrite(linactIN2, LOW);
}

void stay(){
  digitalWrite(linactIN1, HIGH);
  digitalWrite(linactIN2, HIGH);  
}


