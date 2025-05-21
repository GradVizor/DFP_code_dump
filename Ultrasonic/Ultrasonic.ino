// Define the pins for the ultrasonic sensors
const int trigFront = 2;
const int echoFront = 3;
const int trigLeft = 4;
const int echoLeft = 5;
const int trigRight = 6;
const int echoRight = 7;

// Detection threshold in cm
const int detectionDistance = 20;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Set up the sensor pins
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);
  
  Serial.println("Ultrasonic Sensor System Initialized");
  Serial.println("Monitoring for objects at ~20cm...");
}

void loop() {
  // Measure distances from all three sensors
  int distanceFront = getDistance(trigFront, echoFront);
  int distanceLeft = getDistance(trigLeft, echoLeft);
  int distanceRight = getDistance(trigRight, echoRight);
  
  // Check for objects within detection range
  bool objectFront = distanceFront <= detectionDistance;
  bool objectLeft = distanceLeft <= detectionDistance;
  bool objectRight = distanceRight <= detectionDistance;
  
  // Print the results
  // Serial.print("Front: ");
  // Serial.print(distanceFront);
  // Serial.print("cm | Left: ");
  // Serial.print(distanceLeft);
  // Serial.print("cm | Right: ");
  // Serial.print(distanceRight);
  // Serial.println("cm");
  
  // Print warnings if objects are detected
  if (objectFront) {
    Serial.println("WARNING: Object detected in FRONT!");
  }
  if (objectLeft) {
    Serial.println("WARNING: Object detected on LEFT!");
  }
  if (objectRight) {
    Serial.println("WARNING: Object detected on RIGHT!");
  }
  
  // Add a small delay between readings
  delay(200);
}

// Function to measure distance from a single sensor
int getDistance(int trigPin, int echoPin) {
  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Send a 10 microsecond pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Measure the pulse duration on the echo pin
  long duration = pulseIn(echoPin, HIGH);
  
  // Calculate distance in cm (speed of sound is 343 m/s or 0.0343 cm/μs)
  // Distance = (duration * speed) / 2 (round trip)
  int distance = duration * 0.0343 / 2;
  
  return distance;
}