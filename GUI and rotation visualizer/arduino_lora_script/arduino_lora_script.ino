/*
 * LoRa E220
 * Write on serial to transfer a message to other device
 * by Renzo Mischianti <https://www.mischianti.org>
 * https://www.mischianti.org/category/my-libraries/lora-e220-llcc68-devices/
 *
 * E220        ----- Arduino UNO
 * M0         ----- GND
 * M1         ----- GND
 * TX         ----- PIN 2 (PullUP)
 * RX         ----- PIN 3 (PullUP & Voltage divider)
 * AUX        ----- Not connected
 * VCC        ----- 3.3v/5v
 * GND        ----- GND
 *
 */
#include "Arduino.h"
#include "LoRa_E220.h"

 
LoRa_E220 e220ttl(8, 9, 10); // Arduino RX --> e220 TX - Arduino TX --> e220 RX
String receivedString = "";


void setup() {
  Serial.begin(9600);
  delay(500);
  pinMode(13, OUTPUT); 
  
 
  // Startup all pins and UART
  e220ttl.begin();
 
  //Serial.println("Hi, I'm going to send message!");
  // Send message
  //ResponseStatus rs = e220ttl.sendMessage("Hello, world? fin");
  // Check If there is some problem of successfully send
  //Serial.println(rs.getResponseDescription());
  
}
 
void loop() {
    // If something available
  if (e220ttl.available()>1) {
      // read the String message
    ResponseContainer rc = e220ttl.receiveMessage();
    // Is something goes wrong print error
    if (rc.status.code!=1){
         Serial.println(rc.status.getResponseDescription());
    }else{
        // Print the data received
        Serial.println(rc.data);
    }
  }
  if (Serial.available()>0){
    //digitalWrite(13, HIGH); 
    //delay(500);
    //digitalWrite(13, LOW);
    receivedString = Serial.readStringUntil('\n');;
    //Serial.println(receivedString);
    e220ttl.sendMessage(receivedString);
    

  }
  


}