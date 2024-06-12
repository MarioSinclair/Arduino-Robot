// Ultrasonic sensor pins
#define S1Trig 2 // Define S1Trig as pin 2
#define S2Trig 4 // Define S2Trig as pin 4
#define S3Trig 6 // Define S3Trig as pin 6
#define S1Echo 3 // Define S1Echoas pin 3
#define S2Echo 5 // Define S2Echo as pin 5 
#define S3Echo 7 // Define S3Echo as pin 7
// Motor control pins
#define LEFT_MOTOR_PIN1 8 // Define LEFT_MOTOR_PIN1 as pin 8
#define LEFT_MOTOR_PIN2 9 // Define LEFT_MOTRO_PIN2 as pin 9
#define RIGHT_MOTOR_PIN1 10 // Define RIGHT_MOTOR_PIN1 as pin 10
#define RIGHT_MOTOR_PIN2 11 // Define RIGHT_MOTOR_PIN2 as pin 11
// Distance thresholds for obstacle detection
#define MAX_DISTANCE 40 // Max distance set as 40
#define MIN_DISTANCE_BACK 5 // Min distance set as 5
// Maximum and minimum motor speeds
#define MAX_SPEED 150 // Define MAX_SPEED as 150
#define MIN_SPEED 75 // Define MIN_SPEED as 75
void setup() {
  // Set motor control pins as outputs
  pinMode(LEFT_MOTOR_PIN1, OUTPUT); // LEFT_MOTOR_PIN1 is set as output
  pinMode(LEFT_MOTOR_PIN2, OUTPUT); // LEFT_MOTOR_PIN2 is set as output
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT); // RIGHT_MOTOR_PIN1 is set as output
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT); // RIGHT_MOTOR_PIN2 is set as output
  //Set the Trig pins as output pins
  pinMode(S1Trig, OUTPUT); // S1Trig is set as output
  pinMode(S2Trig, OUTPUT); // S2Trig is set as output
  pinMode(S3Trig, OUTPUT); // S3Trig is set as output
  //Set the Echo pins as input pins
  pinMode(S1Echo, INPUT); // S1Echo is set as input
  pinMode(S2Echo, INPUT); // S2Echo is set as input
  pinMode(S3Echo, INPUT); // S3Echo is set as input
  // Initialize the serial communication for debugging
  Serial.begin(9600);
}
void loop() {
  int frontDistance = sensorOne(); // Distance from sensor 1 as frontDistance
  int leftDistance = sensorTwo(); // Distance from sensor 2 as leftDistance
  int rightDistance = sensorThree(); // Distance from sensor 3 as rightDistance
// Print out all the distances
  Serial.print("Front: "); // Print "Front: "
  Serial.print(frontDistance); // Print frontDistance
  Serial.print(" cm, Left: "); // Print " cm, Left: "
  Serial.print(leftDistance); // Print leftDistance
  Serial.print(" cm, Right: "); // Print " cm, Right: "
  Serial.print(rightDistance); // Print rightDistance
  Serial.println(" cm"); // Print " cm"
  // Find the sensor with the smallest distance
  if (frontDistance < MIN_DISTANCE_BACK) 
// The robot is too close to the object -> The robot goes backward
  {
    moveBackward();
    Serial.println("backward");
  } 
  else if (frontDistance < leftDistance && frontDistance < rightDistance && frontDistance < MAX_DISTANCE)
// The object is right infront of the robot -> The robot goes forward
  {
    moveForward();
    Serial.println("forward");
  }
  else if (leftDistance < rightDistance && leftDistance < MAX_DISTANCE)
 // The object is to the left of the robot -> The robot turns left
  {
    turnLeft();
    Serial.println("left");
  }
  else if (rightDistance < MAX_DISTANCE) 
// The robot is to the right of the robot -> The robot turns right
  {
    turnRight();
    Serial.println("right");   
   } 
   else
 // Object is not in sight of the sensor -> The robot stops
   {
    stop();
    Serial.println("stop");
  }
  delay(100); // Delay for stability and to avoid excessive readings
}
// Function to measure the distance using an ultrasonic sensor
int sensorOne() {
  //pulse output
  digitalWrite(S1Trig, LOW); // Set trig pin initially as LOW
  delayMicroseconds(2); // Wait for hardware to respond
  digitalWrite(S1Trig, HIGH); // Set trig pin to HIGH
  delayMicroseconds(10); // Hold trig pin in HIGH for 10 microseconds
  digitalWrite(S1Trig, LOW); // Set trig pin as LOW
  long t = pulseIn(S1Echo, HIGH);//Get the pulse
  int cm = t / 29 / 2; //Convert time to the distance
  return cm; // Return the values from the sensor
}
//Get the sensor values
int sensorTwo() {
  //pulse output
  digitalWrite(S2Trig, LOW); // Set trig pin initially as LOW
  delayMicroseconds(2); // Wait for hardware to respond
  digitalWrite(S2Trig, HIGH); // Set trig pin to HIGH
  delayMicroseconds(10); // Hold trig pin in HIGH for 10 microseconds
  digitalWrite(S2Trig, LOW); // Set trig pin as LOW
  long t = pulseIn(S2Echo, HIGH);//Get the pulse
  int cm = t / 29 / 2; //Convert time to the distance
  return cm; // Return the values from the sensor
}
//Get the sensor values
int sensorThree() {
  //pulse output
  digitalWrite(S3Trig, LOW); // Set trig pin initially as LOW
  delayMicroseconds(2); // Wait for hardware to respond
  digitalWrite(S3Trig, HIGH); // Set trig pin to HIGH
  delayMicroseconds(10); // Hold trig pin in HIGH for 10 microseconds
  digitalWrite(S3Trig, LOW); // Set trig pin as LOW
  long t = pulseIn(S3Echo, HIGH);//Get the pulse
  int cm = t / 29 / 2; //Convert time to the distance
  return cm; // Return the values from the sensor
}
  // Motor control functions
void moveForward() {
  analogWrite(LEFT_MOTOR_PIN1, MAX_SPEED); // Left motor goes forward
  analogWrite(LEFT_MOTOR_PIN2, LOW);
  analogWrite(RIGHT_MOTOR_PIN1, MAX_SPEED); // Right motor goes forward
  analogWrite(RIGHT_MOTOR_PIN2, LOW);
}
void moveBackward() {
  analogWrite(LEFT_MOTOR_PIN1, LOW);
  analogWrite(LEFT_MOTOR_PIN2, MAX_SPEED); // Left motor goes backward
  analogWrite(RIGHT_MOTOR_PIN1, LOW);
  analogWrite(RIGHT_MOTOR_PIN2, MAX_SPEED); // Right motor goes backward
}
void turnRight() {
  analogWrite(LEFT_MOTOR_PIN1, LOW);
  analogWrite(LEFT_MOTOR_PIN2, MAX_SPEED); // Left motor goes backward
  analogWrite(RIGHT_MOTOR_PIN1, MAX_SPEED); // Right motor goes forward
  analogWrite(RIGHT_MOTOR_PIN2, LOW);
}
void turnLeft() {
  analogWrite(LEFT_MOTOR_PIN1, MAX_SPEED); // Left motor goes forward
  analogWrite(LEFT_MOTOR_PIN2, LOW);
  analogWrite(RIGHT_MOTOR_PIN1, LOW);
  analogWrite(RIGHT_MOTOR_PIN2, MAX_SPEED); // Right motor goes backward
}
void stop() {
  analogWrite(LEFT_MOTOR_PIN1, LOW); // Left motor stops
  analogWrite(LEFT_MOTOR_PIN2, LOW);
  analogWrite(RIGHT_MOTOR_PIN1, LOW); // Right motor stops
  analogWrite(RIGHT_MOTOR_PIN2, LOW);
}
