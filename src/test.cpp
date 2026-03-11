// #include <Arduino.h>
// // ============================================================
// //  A3144 Digital Hall Sensor
// //  Pin 1 = VCC (5V)
// //  Pin 2 = GND  
// //  Pin 3 = Signal + 10kΩ pullup to VCC → GPIO (digital)
// //
// //  Logic: OUTPUT LOW  = มีแม่เหล็กขั้ว S อยู่ใกล้
// //         OUTPUT HIGH = ไม่มีสนามแม่เหล็ก
// // ============================================================

// #define HALL_A3144_PIN  25   // เปลี่ยนตาม pin ที่ต่อจริง

// void setup() {
//   Serial.begin(115200);
//   pinMode(HALL_A3144_PIN, INPUT);   // 10kΩ pullup อยู่ภายนอกแล้ว ไม่ต้อง INPUT_PULLUP
// }

// void loop() {
//   bool detected = (digitalRead(HALL_A3144_PIN) == LOW);   // LOW = มีแม่เหล็ก

//   if (detected) {
//     Serial.println("MAGNET DETECTED — South pole");
//   } else {
//     Serial.println("No field");
//   }

//   delay(100);
// }