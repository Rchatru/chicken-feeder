/* Implementation of an alarm using DS3231
 *
 * VCC and GND of RTC should be connected to some power source
 * SDA, SCL of RTC should be connected to SDA, SCL of arduino
 * SQW should be connected to CLOCK_INTERRUPT_PIN
 * CLOCK_INTERRUPT_PIN needs to work with interrupts
 * For Nano, INT0->D2, INT1->D3
 */


#include <Arduino.h>
#include <Servo.h>
#include <RTClib.h>
#include <avr/sleep.h>
#include <SPI.h>

// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 2

// Instancias de objetos
Servo myservo;  
RTC_DS3231 rtc;

// Definición de variables
int pos = 0;    
int reps = 2;
int rep = 0;
bool state = false;


// Declaracion de funciones
void abrir();
void alarm_ISR();
void enterSleep();
void onAlarm();



void setup() {
  // vincula el servo al pin digital 5
  myservo.attach(5);  

  if(rtc.lostPower()) {
        // this will adjust to the date and time at compilation
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  //we don't need the 32K Pin, so disable it
  rtc.disable32K();

  // Making it so, that the alarm will trigger an interrupt
  pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);

  // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
  // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
  // turn off alarm 1, 2 (in case it isn't off already)
  // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
  rtc.disableAlarm(1);
  rtc.disableAlarm(2);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);

  // Place SQW pin into alarm interrupt mode
  // stop oscillating signals at SQW Pin
  // otherwise setAlarm1 will fail
  rtc.writeSqwPinMode(DS3231_OFF);


  // rtc.setAlarm1(DateTime(9, 0, 0), DS3231_A1_Hour);
  // rtc.setAlarm2(DateTime(15, 0, 0), DS3231_A2_Hour);

  rtc.setAlarm1(
    rtc.now() + TimeSpan(10),
    DS3231_A1_Second // this mode triggers the alarm when the seconds match. See Doxygen for other options
  );

}

void loop() {

  delay(1000);

  enterSleep();  // Go to sleep

  


  
}



// Implementacion de funciones

void abrir(){
  //varia la posicion de 0 a 180, con esperas de 15ms
  for (pos = 0; pos <= 180; pos += 1)
  {
    myservo.write(pos);
    delay(15);
  }
  delay(1000);
  //varia la posicion de 0 a 180, con esperas de 15ms
  for (pos = 180; pos >= 0; pos -= 1)
  {
    myservo.write(pos);
    delay(15);
  }
  delay(1000);
}





void enterSleep(){
  sleep_enable();                       // Enabling sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Setting the sleep mode, in this case full sleep
  
  noInterrupts();                       // Disable interrupts
  attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), alarm_ISR, FALLING);
  
  Serial.println("Going to sleep!");    // Print message to serial monitor
  Serial.flush();                       // Ensure all characters are sent to the serial monitor
  
  interrupts();                         // Allow interrupts again
  sleep_cpu();                          // Enter sleep mode

  /* The program will continue from here when it wakes */
  
  // Disable and clear alarm
  // rtc.disableAlarm(1);
  // rtc.clearAlarm(1);
  
  Serial.println("I'm back!");          // Print message to show we're back
 }


void alarm_ISR() {
  // This runs when SQW pin is low. It will wake up the µController
  
  sleep_disable(); // Disable sleep mode
  detachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN)); // Detach the interrupt to stop it firing

  while (rep<=reps){
    abrir();
    rep++;
  }
}

