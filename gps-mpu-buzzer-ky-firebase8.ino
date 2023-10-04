#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <FirebaseESP32.h>

#define WIFI_SSID "lalalalalalalalala"
#define WIFI_PASSWORD "123123123"
#define FIREBASE_HOST "falldetector-98d21-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "AIzaSyCluCynP5G1a_ISHYW6UXfoN81ZwGeoNb4"

#define buzzerPin 5
#define soundSensorPin 32
#define thresholdmpu 10000
#define thresholdky 500

MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial SerialGPS(1); // Gunakan Serial1 atau Serial2 tergantung port serial yang Anda gunakan

FirebaseData firebaseData;

int prevAccX = 0;
int prevAccY = 0;
int prevAccZ = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
  mpu.initialize();

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}

void loop() {
  // mpu
  int accX = mpu.getAccelerationX();
  int accY = mpu.getAccelerationY();
  int accZ = mpu.getAccelerationZ();
  int deltaX = abs(accX - prevAccX);
  int deltaY = abs(accY - prevAccY);
  int deltaZ = abs(accZ - prevAccZ);
  // hitung total delta percepatan mpu
  int a_total = sqrt(deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ); 
  //serial monitor mpu
  Serial.print("X: ");
  Serial.print(deltaX);
  Serial.print(", Y: ");
  Serial.print(deltaY);  
  Serial.print(", Z: ");
  Serial.println(deltaZ);
  Serial.println("Percepatan Total: " + String(a_total) + " m/s^2");

  // sensor ky
  int soundValue = analogRead(soundSensorPin);
  //serial monitor sound
  Serial.println("Sound Value: " + String (soundValue)); 

  // gps
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
    if (gps.location.isUpdated()) {
  //serial monitor gps
      Serial.print("Latitude: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude: ");
      Serial.print(gps.location.lng(), 6);
      Serial.println(" ");
  }
  }

  // kondisi jstuh mpu
  if (a_total >= thresholdmpu) {
    Serial.println("fall detected!");
    digitalWrite(buzzerPin, HIGH);
    delay(1000);
    digitalWrite(buzzerPin, LOW);
    Firebase.setString(firebaseData, "/Sensor/Sensor Mpu/Kondisi", "Fall Detected!");
    Firebase.setString(firebaseData, "/Sensor/Sensor Mpu/Status", "Bahaya!");
  }    
      
  // logika ky
  if (soundValue > thresholdky) {
    Firebase.setString(firebaseData, "/Sensor/Sensor Ky/Kondisi", "Scream Detected!");
    delay(3000);    
} 


    else {
    Firebase.setString(firebaseData, "/Sensor/Sensor Mpu/Kondisi", "Fall not Detected!");
    Firebase.setString(firebaseData, "/Sensor/Sensor Ky/Kondisi", "Scream not Detected!");
    Firebase.setString(firebaseData, "/Sensor/Sensor Mpu/Status", "Aman");
    }

    Firebase.setInt(firebaseData, "/Sensor/Sensor Mpu/Value", a_total);
    Firebase.setFloat(firebaseData, "/Sensor/Sensor Gps/latitude", gps.location.lat());
    Firebase.setFloat(firebaseData, "/Sensor/Sensor Gps/longitude", gps.location.lng());
    Firebase.setInt(firebaseData, "/Sensor/Sensor Ky/Value", soundValue);
    
  prevAccX = accX;
  prevAccY = accY;
  prevAccZ = accZ;

  delay(500);
}
