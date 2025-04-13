#include <CurieBLE.h>
#include <SoftwareSerial.h>


int aa = 1;
int bb = 0;
// A indicate analoge
int throttleOutPin = 9;
int bluetoothInPin = A5;
int bluetoothOutputValue=0;


//safety features - brake and throttle
int leftBrakeIn = 2;
int rightBrakeIn = 0;

int throttleIn = A2;

//initialize
int leftBrakeValue;
int rightBrakeValue;
int throttleInValue;
int throttleOutValue = 0;

int minThrottleInValue = 400;
int maxThrottleInValue = 1000;

int minBluetoothInValue = 0;
int maxBluetoothInValue = 255;

//set maximum throttle out value that should ever be sent
int throttleOutMin = 0;
int throttleOutMax = 255;

//String to store command received by bluetooth
int bluetoothReceived;
String bluetoothReceivedString="";


char dataFromSensors[100] = "";
int count =0;
bool isReadingSensor = false;

//initiates software serial, pin 11 is unused and used as a dummy TX here
SoftwareSerial mySerial(bluetoothInPin, bluetoothInPin); // RX, TX

//BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214");

// initial the blackbox as a peripheral device
BLEPeripheral blePeripheral;       // BLE Peripheral Device (the board you're programming)

// BLE Bike Service
BLEService bikeDataService("180D");

// BLE bike Measurement Characteristic
BLECharacteristic bikeChar("2A37",// standard 16-bit characteristic UUID
    BLERead | BLENotify, 80);      // remote clients will be able to get notifications if this characteristic changes
                                    // the characteristic is 100 bytes long as the fisr field needs to be "Flags" as per BLE
                                    // specifications.
// BLE motor power Characteristic        
BLEIntCharacteristic throChar("19B10002-E8F2-537E-4F6C-D104768A1214", // standard 16-bit characteristic UUID
   BLERead | BLEWriteWithoutResponse);// remote clients will be able to get notifications if this characteristic changes
                                    // the characteristic is 1 bytes long as the fisr field needs to be "Flags" as per BLE
                                    // specifications.

// the setup routine runs once when you press reset:
void setup() {
  // Open serial communications and wait for port to open:
    Serial.begin(9600);
   
    // set the data rate for the SoftwareSerial port
    mySerial.begin(9600);

    //Initialise pin 13 as the output LED pin if required for testing
    pinMode(leftBrakeIn, INPUT_PULLUP);
    pinMode(rightBrakeIn, INPUT_PULLUP);
    pinMode(throttleIn, INPUT);
    pinMode(throttleOutPin, OUTPUT);


    // begin initialization
    BLE.begin();
    
     /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet */
    // set the UUID for the service this peripheral advertises:
    BLE.setLocalName("EBIKE");
    blePeripheral.setAdvertisedServiceUuid(bikeDataService.uuid());
    blePeripheral.addAttribute(bikeDataService);
    blePeripheral.addAttribute(bikeChar);
    blePeripheral.addAttribute(throChar);
    blePeripheral.begin();

    
    /* Now activate the BLE device.  It will start continuously transmitting BLE
       advertising packets and will be visible to remote BLE central devices
       until it receives a new connection */
    blePeripheral.begin(); 
  }

void loop() {
  
  Serial.println("Loop Start");
  // read the input pin:
  
  int leftBrakeValue = digitalRead(leftBrakeIn);
  int rightBrakeValue = digitalRead(rightBrakeIn);
  int throttleInValue = analogRead(throttleIn);
  
  // listen for BLE peripherals to connect:true in case of connection; false otherwise 
  BLEDevice central = BLE.central();
  
  if (central) {
    // as long as the central is still connected: the data will be sent out
    while (central.connected()) { 
      
      //receiveSensorData();
        if (isReadingSensor) {
              return;
          }
      //read data from bike sensors which arrives to pin 
      isReadingSensor = true;

      while(mySerial.available()&& digitalRead(leftBrakeIn) && digitalRead(rightBrakeIn) && (analogRead(throttleIn) < 400) && (throChar.written()==0)){
          //Serial.print("boolean:");
          //Serial.println(mySerial.available()&& digitalRead(leftBrakeIn) && digitalRead(rightBrakeIn) && (analogRead(throttleIn) < 400) && (throChar.written()==0));
         
          char reading = mySerial.read();
    
           if(reading == '\n'){  
              count+=1;    
               
               if(count>=2){
                  Serial.print(bluetoothReceivedString);
                  Serial.println();
                  char bikeCharArray[80]= {bluetoothReceivedString[0], bluetoothReceivedString[1], bluetoothReceivedString[2], bluetoothReceivedString[3]};
                  bluetoothReceivedString.toCharArray(bikeCharArray,80);
                  
                  // updata characteristics
                  for (int i = 0; i< 4 ; i++) {
                    char splited[20];
                    for(int j =0;j<20;j++) {
                      splited[j] = bikeCharArray[j + i * 20];
                    }
                    bikeChar.setValue((unsigned char*)splited, 20);
                  }
                  Serial.println("Send data");                 
               }
               
               bluetoothReceivedString = "";
                
            } else {
                 bluetoothReceivedString += (char)reading;
           }
           //Serial.println("===============================");
        }

      Serial.println("Reading Sensor: False");  
      isReadingSensor = false;
      bluetoothReceivedString = "";
      
      if ((digitalRead(leftBrakeIn) & digitalRead(rightBrakeIn))== 0){
          Serial.println("BRAKE ENGAGED");
          throttleOutValue = 0;
          analogWrite(throttleOutPin, throttleOutValue);
          }
          
      // hand throttle status
      else {
        if (analogRead(throttleIn) > 400) {
          int throttleInValue = analogRead(throttleIn);
          Serial.print("Throttle reading is: ");
          Serial.println(analogRead(throttleIn));
          //safety check just in case last value that was sent was greater than the maximum it should be
          if (throttleInValue> 1000){
              throttleInValue = 1000;
              }
            //Note that in the mapping below we assume that if the user turns the throttle forward in any way that they want
            //to go faster than they are currently going at, this is why the fourth argument in the map function below
            //is the last bluetoothOutputValue that was sent
          throttleOutValue = map(throttleInValue, 400, 1000, 0, 255);
          Serial.print("Throttle setting is: ");
          Serial.println(throttleOutValue);
          analogWrite(throttleOutPin, throttleOutValue);    
         }

         else {//Receive from phone
                //receiveBLEData(); 
                 if (throChar.written()){
                  Serial.println("Got a command");
                  int RXValue = throChar.value();
                  // A value between 0 and 1023 to mapped to a value between 0 and 255
                  int MapRXValue = map(RXValue, minBluetoothInValue, maxBluetoothInValue, 0, 255);
                  // write corresponding value for EBIKE, PWM signal
                  Serial.println(MapRXValue);
                  analogWrite(throttleOutPin, MapRXValue);
                }
            }
         }
      }
  }
 
  else{
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    count=0; 
    bluetoothReceivedString = "";
    if (digitalRead(leftBrakeIn) && digitalRead(rightBrakeIn)== 0){
          Serial.println("BRAKE ENGAGED");
          throttleOutValue = 0;
          analogWrite(throttleOutPin, throttleOutValue);
          }
          
      // hand throttle status
   else {
        if (analogRead(throttleIn) > 400) {
          int throttleInValue = analogRead(throttleIn);
          Serial.print("Throttle reading is: ");
          Serial.println(analogRead(throttleIn));
          if (throttleInValue> 1000){
              throttleInValue = 1000;
              }
        throttleOutValue = map(throttleInValue, 400, 1000, 0, 255);
        Serial.print("Throttle setting is: ");
        Serial.println(throttleOutValue);
        analogWrite(throttleOutPin, throttleOutValue);    
         }
  }
} 

/*
void receiveSensorData() {
      if (isReadingSensor) {
          return;
      }
      //read data from bike sensors which arrives to pin 
      isReadingSensor = true;

      while(mySerial.available()&& digitalRead(leftBrakeIn) && digitalRead(rightBrakeIn) && (analogRead(throttleIn) < 400) && (throChar.written()==0)){
          Serial.print("boolean:");
          Serial.println(mySerial.available()&& digitalRead(leftBrakeIn) && digitalRead(rightBrakeIn) && (analogRead(throttleIn) < 400) && (throChar.written()==0));
         
          char reading = mySerial.read();
    
           if(reading == '\n'){  
              count+=1;    
               
               if(count>=2){
                  Serial.print(bluetoothReceivedString);
                  Serial.println();
                  char bikeCharArray[80]= {bluetoothReceivedString[0], bluetoothReceivedString[1], bluetoothReceivedString[2], bluetoothReceivedString[3]};
                  bluetoothReceivedString.toCharArray(bikeCharArray,80);
                  
                  // updata characteristics
                  for (int i = 0; i< 4 ; i++) {
                    char splited[20];
                    for(int j =0;j<20;j++) {
                      splited[j] = bikeCharArray[j + i * 20];
                    }
                    bikeChar.setValue((unsigned char*)splited, 20);
                  }
                  Serial.println("Send data");                 
               }
               
               bluetoothReceivedString = "";
                
            } else {
                 bluetoothReceivedString += (char)reading;
           }
           Serial.println("===============================");
        }

      Serial.println("Reading Sensor: False");  
      isReadingSensor = false;
      bluetoothReceivedString = "";
}

void receiveBLEData(){
      //Serial.println("Receive from phone");
      if (throChar.written()){
        Serial.println("Got a command");
        int RXValue = throChar.value();
        // A value between 0 and 1023 to mapped to a value between 0 and 255
        int MapRXValue = map(RXValue, minBluetoothInValue, maxBluetoothInValue, 0, 255);
        // write corresponding value for EBIKE, PWM signal
        Serial.println(MapRXValue);
        analogWrite(throttleOutPin, MapRXValue);
      }
}
*/
}
 
