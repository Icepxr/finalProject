// #define HALL_SENSOR_PIN 34
// #include <Arduino.h>

// float adcOffset = 0;

// void setup() {
//     Serial.begin(115200);
//     delay(1000);

//     // Calibrate offset (ไม่มีแม่เหล็กใกล้เซนเซอร์)
//     long sum = 0;
//     for(int i = 0; i < 200; i++){
//         sum += analogRead(HALL_SENSOR_PIN);
//         delay(5);
//     }
//     adcOffset = sum / 200.0;

//     Serial.print("ADC Offset: ");
//     Serial.println(adcOffset);
// }

// void loop() {

//     int adcValue = analogRead(HALL_SENSOR_PIN);
//      int magneticStrength = adcValue - adcOffset; 


//     // แปลงเป็น Gauss
//     float B = ((adcValue - adcOffset) / 4095.0) * (3.3 / 0.0014);

//     Serial.print("ADC: ");
//     Serial.print(adcValue);
//     Serial.print(" | Magnetic Field: ");
//     Serial.print(B);
//     Serial.println(" Gauss");
//     Serial.println( "------------------------------------------" );
//     Serial.print("| Magnetic Field (Milli Tesla) |");
//     Serial.println(B / 10.0);  // แปลงจาก Gauss เป็น Tesla (1 Tesla = 10,000 Gauss)
//     if(magneticStrength > 50){
//         Serial.println("[S] South Pole Detected");
//     } else if(magneticStrength < -50){
//         Serial.println("[N] North Pole Detected");
//     } else {
//         Serial.println("No Magnetic Field Detected");
//     }
//     delay(200);
// }