/*#########################################################################################################################
 * Codice per la calibrazione degli ESC, da fare la prima volta che si comprano degli ESC.
 * Il codice è stato preso dagli esempi della libreria ESC.h
 * La calibrazione degli ESC è una procedura standard, quindi non ho avuto bisogno di modificare il codice
 
  ESC_Calibration
    It's important to calibrate the ESC with the values (uS / microseconds) he will expect to be for Min and Max speed.
    This one require a little procedure:  
      1 - Load the sketch to your Arduino board
      2 - Connect your ESC to the Arduino board
      3 - Power your Arduino board
      4 - Once the LED (LED_PIN) is HIGH/ON connect the power to your ESC, you have 5sec to do so
      5 - Once the LED is LOW/OFF the calibration should be done
      6 - Should now be calibrated between 1000us and 2000us
      
 ##########################################################################################################################*/
#include "ESC.h"
#define LED_PIN (LED_BUILTIN)              // Pin for the LED 

ESC myESC (D6, 1000, 2000, 500);   // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Arm Value)

void setup() {
  pinMode(LED_PIN, OUTPUT);       // LED Visual Output (can be removed)
  digitalWrite(LED_PIN, HIGH);    // LED High while signal is High (can be removed)
  myESC.calib();                  // Calibration of the Max and Min value the ESC is expecting
  myESC.stop();                   // Stop the ESC to avoid damage or injuries
  digitalWrite(LED_PIN, LOW);     // LED Low when the calibration is done (can be removed)
}

void loop() {

}
