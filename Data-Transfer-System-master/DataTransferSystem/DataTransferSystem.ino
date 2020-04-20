#include <CurieBLE.h>
#include <SoftwareSerial.h>


// digital pin 0 and 2 have connected to the brake of the bike:
int leftBrakeIn = 2;
int rightBrakeIn = 0;

// analog pin A2 has connected to the hand throttle
int ThrottleIn = A2;
int mySerial = A5;

// Output Vout has connectes to the digital pin 9, which used to control the motor
int Vout = 9;

//LED attached to pin 13 that can be used for testing
int ledPin = 13;


// initial the blacbox as a peripheral device
BLEPeripheral blePeripheral;  // BLE Peripheral Device (the board you're programming)
// intinial a BLE service by given an UUID number
BLEService EBIKEService("19B10010-E8F2-537E-4F6C-D104768A1214"); // create service

// create Receive characteristic - custom 128-bit UUID, read and writable by central(smartphone)
// data from smartphone
BLECharCharacteristic RXCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWriteWithoutResponse);
// create Transceive characteristic and allow remote device (smartphone) to get notifications
// data from bike
BLECharCharacteristic TXCharacteristic("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify); 



// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // set pin mode
  pinMode(leftBrakeIn, INPUT_PULLUP);
  pinMode(rightBrakeIn, INPUT_PULLUP);
  pinMode(ThrottleIn, OUTPUT);
  pinMode(Vout, OUTPUT);
  
  // begin initialization
  BLE.begin();

  // set the local name peripheral advertises
  BLE.setLocalName("EBIKE");
  // set the UUID for the service this peripheral advertises:
  BLE.setAdvertisedService(EBIKEService);
  blePeripheral.setAdvertisedServiceUuid(EBIKEService.uuid());

// add the characteristics to the service
  EBIKEService.addCharacteristic(RXCharacteristic);
  EBIKEService.addCharacteristic(TXCharacteristic);

  // add the service
  BLE.addService(EBIKEService);

  RXCharacteristic.setValue(0);
  TXCharacteristic.setValue(0);

  // start advertising
  BLE.advertise();

  // Serial.printIn will give a \r + \n in the end
  Serial.println("Bluetooth device active, waiting for connections...");

}



// the loop routine runs over and over again forever:
void loop() {
  // read the input pin:
  int leftBrakeValue = digitalRead(leftBrakeIn);
  int rightBrakeValue = digitalRead(rightBrakeIn);
  int ThrottleValue = analogRead(ThrottleIn);

  if ((leftBrakeValue & rightBrakeValue) == 0){
    analogWrite(Vout, 0);
  }
  
  else if (ThrottleValue) {
    analogWrite(Vout, ThrottleValue);
    }
    
    else {
        // poll for BLE events
        BLE.poll();
       
        // listen for BLE peripherals to connect: true in case of connection; false otherwise 
        BLEDevice central = BLE.central();
       
        // if a central is connected to peripheral:
        if (central) {
          Serial.print("Connected to central: ");
          // print the central's MAC address:
          Serial.println(central.address());
       
          // while the central is still connected to peripheral:, the data will be sent out
          while (central.connected()) {
            
            // examing the data is corresponse to the voltage
            float Throttlevoltage = ThrottleValue * (5.0/1023.0);
            // print out the value you read:
            Serial.println(Throttlevoltage);

            char reading = mySerial.read();
            char TXValue = leftBrakeValue<<11 | rightBrakeValue<<10 | ThrottleValue;
            boolean TXChanged = (TXCharacteristic.value() != TXValue);
      
           // updata new EBIKE value to the user
            if (TXChanged) {
              //TX state changed, update characteristics
              TXCharacteristic.setValue(TXValue);
            }
      
           
            if (RXCharacteristic.written()) {
              int RXValue = RXCharacteristic.value();
          
              // A value between 0 and 1023 to mapped to a value between 0 and 255
              int MapRXValue = map(RXValue,0,1023,0,255);
              // write corresponding value for EBIKE, PWM signal
              analogWrite(Vout, MapRXValue);
            }
            
            delay(2);        // delay in between reads for stability
          }
         }
          
          Serial.print("Disconnected from central: ");
          Serial.println(central.address());
  }
}
