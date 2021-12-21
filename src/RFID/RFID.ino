/*
  https://github.com/miguelbalboa/rfid/blob/master/src/MFRC522.h
  RC522_RFID_Module_Example
*/

#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 10//SDA     Slave select
#define RST_PIN 9//RST

MFRC522 mfrc522(SS_PIN, RST_PIN);
int count1 = 0;  //keyHolder counter
int target=0, sens=0;  //initilized to 0 to check errors -> if target or sens are zero, parameters are not set
int count2 = 0;  //card      counter

int set = 0;

void setup() {
  pinMode(2, OUTPUT);
  Serial.begin(9600);
  SPI.begin();  //enable SPI
  mfrc522.PCD_Init();    // Initialize MFRC522 Hardware

}

void loop() {
  byte id1[4] = {9,133,241,194};  //key holder  TAG
  byte id2[4] = {20,110,103,43};  //card        TAG
  
  int flag1 = 0;
  int flag2 = 0;
  
  // Look for new cards
  if ( ! mfrc522.PICC_IsNewCardPresent()){
    //enter here if no card is read  
    set++;
    if(set==10){  //no card read for long time
        if(count1!=0){
          target = count1;
        }
        if(count2!=0){
          sens = count2;
        }
    }
    delay(1000);
    return;
  }

  
  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial())
  {
    return;
  }

  for (byte i = 0; i < mfrc522.uid.size; i++) {
    //Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
    //Serial.print(mfrc522.uid.uidByte[i], HEX);
    digitalWrite(2, HIGH);
    delay(300);
    digitalWrite(2, LOW);
  }

  //compare with id1 or id2
  for (byte i = 0; i < 4; i++){
     if (mfrc522.uid.uidByte[i] == id1[i]){
        flag1 = 1;
     }else{
      flag1 = 0;
     }
     if (mfrc522.uid.uidByte[i] == id2[i]){
        flag2 = 1;
     }else{
      flag2 = 0;
     }
  }
  
  if (flag1 == 1){
    if(count1 == 3){  //if reach 3, restart counting
      count1 = 0;
    }
    count1++;
  }
  if(flag2 == 1){
    if(count2 == 3){  //if reach 3, restart counting
      count2 = 0;
    }else{
      count2++;
      
    }
  }
  
  delay(1000);

}
