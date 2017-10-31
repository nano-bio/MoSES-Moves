//------------
//define pins:
//------------
int PUL = 7; //define Pulse pin
int DIR = 6; //define Direction pin
int ENA = 5; //define Enable pin
int POWER = 4;
int TRI = 8; //define Trigger pin -> starts (or stops) measurement
int normHIGH = 9; //define normal high pin
int endSwitch_right = 12;
int start_Switch = 11;

//-----------------
//define variables:
//-----------------
int time_wait = 300; //time in µs -> defines speed of stepper motor to the left
int time_wait_initial = 500; //time in µs -> defines speed of stepper motor to the right (initial position)
int pulse_per_rev = 3200; //pulses per revolution
float mpoints = 25; //number of measurement points
long interval = 3000; // interval in ms
float total_time_min = interval * mpoints / 1000 / 60; // total measurement time - min
int status_variable = 0; // 0 = off, 1 = moving, 2 = resting, 3 = returning
unsigned long arrival_time = 0; // designates time, when a measurement position was reached
int current_mpoint = 0;

void setup() {

  Serial.begin(9600);
  pinMode (TRI, INPUT);
  pinMode (endSwitch_right, INPUT);

  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (ENA, OUTPUT);
  pinMode (POWER, OUTPUT);
  pinMode (normHIGH, OUTPUT);

  digitalWrite(normHIGH, HIGH); //fix HIGH (5V) pin
  digitalWrite(POWER, HIGH); //fix HIGH (5V) pin
  digitalWrite(endSwitch_right, HIGH); //end switch right pin high
  digitalWrite(start_Switch, HIGH); //end switch left pin high

}

// this function moves distance millimeters
void move_target(int distance) {
  // distance to move in millimeter
  int pwm_pulses = floor(distance * pulse_per_rev / 5);

  // only move if we are not currently moving or returning
  if (status_variable == 1) {
    for (int i = 0; i < pwm_pulses; i++)
    {
      if (digitalRead(TRI) == HIGH)
      {
        digitalWrite(ENA, HIGH);
        digitalWrite(DIR, LOW); //left
        digitalWrite(PUL, HIGH);
        delayMicroseconds(time_wait);
        digitalWrite(PUL, LOW);
        delayMicroseconds(time_wait);
      }
      else
      {
        digitalWrite(ENA, LOW);
      }
    }
    // set status to resting
    arrival_time = millis();
    status_variable = 2;
  }
}

// moves the target to the starting position
void move_to_starting_position() {
  if (status_variable == 3) {
    //----------------------------------------
    //drive right -> back to initial position:
    //----------------------------------------
    do
    {
      for (int i = 0; i < pulse_per_rev / 5; i++)
      {
        if (digitalRead(TRI) == HIGH && digitalRead(endSwitch_right) == LOW)
        {
          digitalWrite(ENA, HIGH);
          digitalWrite(DIR, HIGH); //right
          digitalWrite(PUL, HIGH);
          delayMicroseconds(time_wait_initial);
          digitalWrite(PUL, LOW);
          delayMicroseconds(time_wait_initial);
        }
        else
        {
          digitalWrite(ENA, LOW);
        }
      }
      Serial.println("Driving back to initial position");
    } while (digitalRead(endSwitch_right) == LOW);

    delay(1000);

    //--------------------------------------------------------------------------------------------------
    //drive left (2mm + offset for end switch hardware position) -> after end switch right is triggered:
    //--------------------------------------------------------------------------------------------------
    for (int j = 0; j < 5; j++)
    {
      for (int i = 0; i < pulse_per_rev / 10; i++)
      {
        if (digitalRead(TRI) == HIGH)
        {
          digitalWrite(ENA, HIGH);
          digitalWrite(DIR, LOW); //left
          digitalWrite(PUL, HIGH);
          delayMicroseconds(time_wait);
          digitalWrite(PUL, LOW);
          delayMicroseconds(time_wait);
        }
        else
        {
          digitalWrite(ENA, LOW);
        }
      }
    }
    Serial.print("\n");
    Serial.println("Initial position reached! Ready for new measurement.");
    Serial.print("\n");    

    status_variable = 0; // off
  }
}

void loop() {
  // if the button is pressed and the status is off, a measurement should be started
  if (digitalRead(start_Switch) == LOW) {
    if (status_variable == 0) {
      // starting new measurement

      Serial.println("-----------------------------------------------");
      Serial.println("Starting measurement!");
      Serial.println("-----------------------------------------------");
      Serial.print("Configured measurement points:");
      Serial.print("\t");
      Serial.println(mpoints);
  
      Serial.print("Configured measurement time per point:");
      Serial.print("\t");
      Serial.print(interval, DEC);
      Serial.println(" ms");
  
      Serial.print("Total measurement time:");
      Serial.print("\t");
      Serial.print(total_time_min, DEC);
      Serial.println(" min");
      Serial.println("-----------------------------------------------");
      
      status_variable = 1;
      move_target(1);
    } else if (status_variable == 1 || status_variable == 2) {
      status_variable = 3; // returning
      move_to_starting_position();
    }
  }

  // if resting, check whether the target should still wait
  if (status_variable == 2) {
    unsigned long current_time = millis();

    // waited long enough, move
    if (current_time - arrival_time > interval) {
      // two options: next measurement step or return to starting position
      if (current_mpoint < mpoints) {
        Serial.print("Measurement position:");
        Serial.print("\t");
        Serial.println(current_mpoint + 1);
        
        status_variable = 1;
        move_target(1);
        current_mpoint = current_mpoint + 1;
      } else {
        status_variable = 3;
        move_to_starting_position();
      }
    }
  }
}



