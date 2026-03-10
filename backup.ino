
// -------------------------------------------------------------------
// Previous hardware-specific code (kept as comments for reference)
// -------------------------------------------------------------------
//
// #include "ICM_20948.h"
// #include <LiquidCrystal.h>
//
// #define WIRE_PORT Wire
// #define AD0_VAL 1
// #define LED_pin 23
//
// ICM_20948_I2C myICM;
//
// // LCD pins:
// // const int rs = 19, en = 18, d4 = 27, d5 = 26, d6 = 25, d7 = 33;
// // LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
//
// #define alpha 0.1f
// #define step_threshold 100
//
// bool counter_initializing = true;
// bool going_up = false;
// int steps_taken = 0;
// float ema = 0;
// float step_min = 0;
// float step_max = 0;
// int iteration = 0;
//
// unsigned long last_lcd_print = 0;
// char buff[16];
// #define millis_per_print 500
//
// void setupICM20948() {
//   WIRE_PORT.begin();
//   WIRE_PORT.setClock(400000);
//
//   bool initialized = false;
//   while (!initialized) {
//     myICM.begin(WIRE_PORT, AD0_VAL);
//
//     SERIAL_PORT.print(F("Initialization of the sensor returned: "));
//     SERIAL_PORT.println(myICM.statusString());
//
//     if (myICM.status != ICM_20948_Stat_Ok) {
//       SERIAL_PORT.println("Trying again...");
//       delay(500);
//     } else {
//       initialized = true;
//     }
//
//     myICM.swReset();
//     if (myICM.status != ICM_20948_Stat_Ok) {
//       SERIAL_PORT.print(F("Software Reset returned: "));
//       SERIAL_PORT.println(myICM.statusString());
//     }
//     delay(250);
//
//     myICM.sleep(false);
//     myICM.lowPower(false);
//
//     myICM.setSampleMode(ICM_20948_Internal_Acc, ICM_20948_Sample_Mode_Continuous);
//     if (myICM.status != ICM_20948_Stat_Ok) {
//       SERIAL_PORT.print(F("setSampleMode returned: "));
//       SERIAL_PORT.println(myICM.statusString());
//     }
//
//     ICM_20948_fss_t myFSS;
//     myFSS.a = gpm8;
//     myICM.setFullScale(ICM_20948_Internal_Acc, myFSS);
//     if (myICM.status != ICM_20948_Stat_Ok) {
//       SERIAL_PORT.print(F("setFullScale returned: "));
//       SERIAL_PORT.println(myICM.statusString());
//     }
//
//     ICM_20948_dlpcfg_t myDLPcfg;
//     myDLPcfg.a = acc_d246bw_n265bw;
//     myICM.setDLPFcfg(ICM_20948_Internal_Acc, myDLPcfg);
//     if (myICM.status != ICM_20948_Stat_Ok) {
//       SERIAL_PORT.print(F("setDLPcfg returned: "));
//       SERIAL_PORT.println(myICM.statusString());
//     }
//
//     ICM_20948_smplrt_t sample;
//     sample.a = 100;
//     myICM.setSampleRate(ICM_20948_Internal_Acc, sample);
//   }
// }
//
// float pythagorean(float x, float y, float z) {
//   return sqrt(x * x + y * y + z * z);
// }
//
// float next_ema(float ema_value, float a, float new_value) {
//   return new_value * a + ema_value * (1 - a);
// }
//
// void applyToEMA(float acc) {
//   ema = next_ema(ema, alpha, acc);
// }
//
// void begin_step() {
//   digitalWrite(LED_pin, HIGH);
//   going_up = false;
//   steps_taken++;
// }
//
// void end_step() {
//   digitalWrite(LED_pin, LOW);
//   going_up = true;
// }
//
// void init_step(float x) {
//   if (x > step_max) {
//     step_max = x;
//     if (step_max - step_min > step_threshold) {
//       counter_initializing = false;
//       begin_step();
//     }
//   }
//   if (x < step_min) {
//     step_min = x;
//     if (step_max - step_min > step_threshold) {
//       counter_initializing = false;
//       end_step();
//     }
//   }
// }
//
// void checkStep(float x) {
//   if (x > step_max) {
//     step_max = x;
//     step_min = step_max - step_threshold;
//     if (going_up) begin_step();
//   }
//   if (x < step_min) {
//     step_min = x;
//     step_max = step_min + step_threshold;
//     if (!going_up) end_step();
//   }
// }
//
// void doStepCounting(float acc) {
//   iteration++;
//   if (iteration == 1) {
//     step_min = acc;
//     step_max = acc;
//     ema = acc;
//     return;
//   }
//
//   applyToEMA(acc);
//   float x = ema;
//
//   if (counter_initializing) {
//     init_step(x);
//     return;
//   }
//
//   checkStep(x);
// }
//
// void loopICM20948() {
//   if (myICM.dataReady()) {
//     myICM.getAGMT();
//
//     float accX = myICM.accX();
//     float accY = myICM.accY();
//     float accZ = myICM.accZ();
//
//     float acc_sum = pythagorean(accX, accY, accZ);
//     doStepCounting(acc_sum);
//     delay(8);
//   } else {
//     SERIAL_PORT.println("Waiting for data");
//     delay(500);
//   }
// }
//
// void setupLCD() {
//   lcd.begin(16, 2);
//   lcd.print("hello, world!");
//   delay(1000);
//   lcd.clear();
//   delay(1000);
//   last_lcd_print = millis();
// }
//
// void loopLCD() {
//   if (millis() > last_lcd_print + millis_per_print) {
//     last_lcd_print = millis();
//     lcd.clear();
//     lcd.setCursor(0, 1);
//     sprintf(buff, "%d steps", steps_taken);
//     lcd.print(buff);
//   }
// }
//
// Original setup/loop usage:
// void setup() {
//   SERIAL_PORT.begin(115200);
//   while (!SERIAL_PORT);
//   setupICM20948();
//   setupLCD();
//   setupBLE();
//   pinMode(LED_pin, OUTPUT);
// }
//
// void loop() {
//   loopICM20948();
//   loopLCD();
//   loopBLE();

