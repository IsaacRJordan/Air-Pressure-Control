// CAN Receive Example
//

#include <mcp_can.h>
#include <SPI.h>
#include "thingProperties.h"

int val;
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

#define CAN0_INT 4                              // Set INT to pin d2 - 4
MCP_CAN CAN0(15);                               // Set CS to pin d8 - 15
const int relay = 16;


void setup()
{
  Serial.begin(115200);

  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  pinMode(relay, OUTPUT);

  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
  
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
  
  Serial.println("MCP2515 Library Receive Example...");
}

void loop()
{

  ArduinoCloud.update();
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

    
    if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    else
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
  
    //Serial.println(msgString);
    
  
    if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for(byte i = 7; i<len; i++){
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.println(rxBuf[i]);
        val  = rxBuf[i] - '0';
        sensor = val;
        Serial.print("val: ");
        Serial.println(val);
        if( val > 200){
          Serial.println("ENTRE ON!!");
          digitalWrite(relay, LOW);
        }else{
          digitalWrite(relay, HIGH);
        }
        
        Serial.print(msgString);
      }
    }
        
    Serial.println();
  }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
