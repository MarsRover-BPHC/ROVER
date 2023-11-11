 // Variables to store left and right velocities
float left_velocity = 0.0;
float right_velocity = 0.0;

void setup() {
  // Start the serial communication
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming serial data
      Serial.println("Data received!");
    String data = Serial.readStringUntil('\n');

    // Parse the comma-separated string
    int commaIndex = data.indexOf(',');
    if (commaIndex != -1) {
      String left_str = data.substring(0, commaIndex);
      String right_str = data.substring(commaIndex + 1);

      // Convert the velocity values to floats
      left_velocity = left_str.toFloat();
      right_velocity = right_str.toFloat();

      // Print the left and right velocities
      Serial.println(left_velocity);
      Serial.println(right_velocity);
      
    if(left_velocity==0 && right_velocity==0){
          digitalWrite(2,LOW);
          digitalWrite(3,LOW);
          digitalWrite(4,LOW);
          digitalWrite(5,LOW);
      }
          if(left_velocity>=2 && right_velocity>=2){
          digitalWrite(2,LOW);
          digitalWrite(3,HIGH);
          digitalWrite(4,LOW);
          digitalWrite(5,HIGH);
      }
          if(left_velocity>=1 && right_velocity>=1){
          digitalWrite(2,HIGH);
          digitalWrite(3,LOW);
          digitalWrite(4,LOW);
          digitalWrite(5,HIGH);
      }
          if(left_velocity>=2 && right_velocity>=1){
          digitalWrite(2,LOW);
          digitalWrite(3,HIGH);
          digitalWrite(4,LOW);
          digitalWrite(5,HIGH);
      }
          if(left_velocity>=1 && right_velocity>=2){
          digitalWrite(2,HIGH);
          digitalWrite(3,LOW);
          digitalWrite(4,HIGH);
          digitalWrite(5,LOW);
      }
      
      
    }
  }
}