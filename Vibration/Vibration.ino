#define ACC_1
#define ACC_2
#define ZERO_D
#define SERIAL_S 115200
#define AN0 14
#define AN1 15
#define AN2 16
#define AN3 17
#define NOISE 0
void setup() {
  analogReference(EXTERNAL);  // Bridge AREF with 3v3. Check if needed to change to AR_EXTERNAL (For SAMD Boards), external is for AVR.
  Serial.begin(SERIAL_S);
}

void loop() {
  if(Serial.available() > 0){  // Waits for command.
    String response0 = Serial.readString();  // Stores command on var.
    if(response0.equals("0")){  // Passes argument to reading function depending on the char received. Sends "Wrong request: {response0}?" if no matches.
      readings(AN0);  // Sensor on A0.
    }else if(response0.equals("1")){
      readings(AN1);  // Sensor on A1.
    }else if(response0.equals("2")){
      readings(AN2);  // Sensor on A2.
    }else if(response0.equals("3")){
      readings(AN3);  // Sensor on A3.
    }else{
      Serial.print("Wrong request: ");
      Serial.print(response0);
      Serial.println("?");
    }
  }
}

void readings(int PIN){
  // Reads 500 values, one each 2000 microseconds, and then sends.
  int read[500];  // Create array to store readings.
  unsigned long t0, t;  // Create variables to store time.
  int i = 0;  // Iterator which counts how many reads have been performed.
  t0 = micros();  // Set start time
  while(i<500){  // Make 500 readings.
    t = micros();  // Current time
    if(t-t0 >= 2000){  // Perform reading if 2000 microseconds have passed.
      #if NOISE
      read[i] = (int)random(0,1024);  // Store reading on array.
      #else
      read[i] = analogRead(PIN);
      #endif
      i++;  // Increase iterator so it stores next reading on next array address.
      t0 = t;  // Set previous time to the time it performed the reading, so each reading is equidistant in time.
    }
  }
  for(int i=0; i<500; i++){
    // Print each value to serial
    Serial.print(read[i]);
    if(i!=499){
      Serial.print(",");
      // Print comma after value for all except last value. Saves a little time on data sending.
    }
  }
}
