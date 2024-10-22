#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

// GPS -> Uno
static const int RXPin = 3, TXPin = 4;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial GPS_Serial(RXPin, TXPin);

// Initialize LCD I2C
LiquidCrystal_I2C lcd(0x27, 20, 4);

// LoRa <-> Pro Mini Interface
#define LORA_RAY_NSS 10
#define LORA_RAY_RST 9
#define LORA_RAY_DIO0 2
#define LORA_RAY_DIO1 6
#define LORA_TX_POWER 20

// Stepper Motor Config
#define en 5
#define dir 6
#define pul 7

// Declare Variable
String location, dataReceived, dateTime;
float currDepth = 0;
float desiredDepth[10];  // Array to hold multiple desired depths
int depthCount = 0;      // Number of depths received
int depthIndex = 0;      // Index to track the current depth being processed
String state = "receive lora";
bool display = true;
bool transmitted = false;

struct SensorData {
  float pH;
  float temp;
  float DO;
  int turbid;
};
SensorData data;

void setup() {
  // SERIAL COMMUNICATION
  Serial.begin(9600);
  while (!Serial)
    ;

  // ===== SETUP LCD =====
  lcd.init();
  lcd.backlight();
  lcd.setCursor(4, 1);
  lcd.print("SubAquaView");
  lcd.setCursor(3, 2);
  lcd.print("Starting Up...");
  delay(3000);

  // ===== SETUP MOTOR STEPPER =====
  pinMode(dir, OUTPUT);
  pinMode(en, OUTPUT);
  pinMode(pul, OUTPUT);
  digitalWrite(en, LOW);  // Enable the stepper motor

  // ===== SETUP LORA =====
  LoRa.setPins(LORA_RAY_NSS, LORA_RAY_RST, LORA_RAY_DIO0);
  while (!LoRa.begin(920E6)) {
    Serial.print(".");
    delay(500);
  }
  LoRa.setTxPower(LORA_TX_POWER);

  // ====== SETUP GPS ======
  GPS_Serial.begin(GPSBaud);

  // Inisialisasi state
  state = "receive lora";
  lcd.clear();
  lcd.setCursor(3, 1);
  lcd.print("System Ready!");
  delay(2000);
}

void loop() {
  if (state == "receive lora") {
    // LoRa Receive Data from Gateway
    if (display == true) {
      lcd.clear();
      lcd.setCursor(2, 1);
      lcd.print("LoRa Standby...");
      display = false;
    }
    depthCount = receiveLoRa(desiredDepth);
    lcd.setCursor(9, 2);
    lcd.print("   ");
    lcd.setCursor(2, 2);
    lcd.print("Receive " + String(depthCount) + " data");

    if (depthCount > 0) {
      delay(2000);
      state = "read location";
      display = true;
      displaySuccess();
    }

  } else if (state == "read location") {
    // Read Latitude and Longitude
    if (display == true) {
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("Reading Location...");
      display = false;
    }
    location = "-6.969080,107.628008";
    dateTime = "2024-08-07T16:52:28Z";
    // location = getLocation();
    // dateTime = getDateTime();
    if (location != "INVALID" || dateTime != "INVALID") {
      state = "depth control";
      display = true;
      delay(2000);
      displaySuccess();
    }

  } else if (state == "depth control") {
    // Control Motor Stepper and Read Depth Position
    Serial.println("dalam");
    if (display == true) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Desired Depth : ");
      lcd.setCursor(0, 1);
      lcd.print(String(desiredDepth[depthIndex]) + " cm");
      lcd.setCursor(0, 2);
      lcd.print("Current Depth : ");
      display = false;
    }
    currDepth = depthControl(desiredDepth[depthIndex]);
    state = "water quality";
    display = true;
    Serial.println("air");
    delay(1000);
    displaySuccess();

  } else if (state == "water quality") {
    // Get Data from Submersible Unit
    if (display == true) {
      lcd.clear();
      lcd.setCursor(3, 1);
      lcd.print("Reading Water");
      lcd.setCursor(5, 2);
      lcd.print("Quality...");
      display = false;
    }

    dataReceived = getDataComm();
    dataReceived.replace("\n", "");
    Serial.println(dataReceived);
    SensorData sensorData = parseData(dataReceived);
    data = sensorData;
    if (dataReceived.length() > 0) {
      state = "transmit lora";
      displayQuality(sensorData, currDepth);
    }
  } else if (state == "transmit lora") {
    // LoRa Transmit Data to Gateway
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Transmitting data...");
    String dataToSend = String(data.temp) + "," + String(data.pH) + "," + String(data.DO) + "," + String(data.turbid) + "," + String(currDepth) + "," + location + "," + dateTime;
    // String dataToSend = dataReceived + "," + String(currDepth) + "," + location + "," + dateTime;
    Serial.println(dataToSend);
    transmitted = transmitLoRa(dataToSend);
    if (transmitted == true) {
      displaySuccess();
      transmitted = false;
      // Check if there are more depths to process
      if (depthIndex < depthCount - 1) {
        depthIndex++;
        state = "depth control";
        display = true;
      } else {
        depthIndex = 0;  // Reset index for the next set of depths
        state = "receive lora";
      }
    }
  } 
  // else if (state == "standby position") {
  //   lcd.clear();
  //   bool standby = depthStandby(currDepth);
  //   if(standby == true){
  //     state = "receive lora";
  //   }
  // }
}

void displaySuccess() {
  lcd.clear();
  lcd.setCursor(5, 1);
  lcd.print("Success!!!");
  delay(2000);
}

void displayQuality(SensorData data, int depth) {
  lcd.clear();
  delay(100);
  lcd.setCursor(0, 0);
  lcd.print("Depth    :" + String(depth) + "cm");
  lcd.setCursor(0, 1);
  lcd.print("DO       :" + String(data.DO) + "mg/L");
  lcd.setCursor(0, 2);
  lcd.print("Turbidity:" + String(data.turbid) + " NTU");
  lcd.setCursor(0, 3);
  lcd.print("pH :" + String(data.pH));
  lcd.setCursor(9, 3);
  lcd.print("Temp:" + String(data.temp) + "C");
  delay(7000);
}

String getLocation() {
  String location = "";
  while (GPS_Serial.available() > 0) {
    if (gps.encode(GPS_Serial.read())) {
      location = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
    }
  }
  return location;
}

String getDateTime() {
  String dateTime = "";
  if (gps.date.isValid() && gps.time.isValid()) {
    dateTime = String(gps.date.year()) + "-" + String(gps.date.month()) + "-" + String(gps.date.day()) + "T" + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) + "Z";
  }
  return dateTime;
}

SensorData parseData(String data) {
  SensorData sensorData;

  int index1 = data.indexOf(',');
  sensorData.temp = data.substring(0, index1).toFloat();

  int index2 = data.indexOf(',', index1 + 1);
  sensorData.pH = data.substring(index1 + 1, index2).toFloat();

  int index3 = data.indexOf(',', index2 + 1);
  sensorData.DO = data.substring(index2 + 1, index3).toFloat();

  sensorData.turbid = data.substring(index3 + 1).toFloat();

  return sensorData;
}

String getDataComm() {
  String data = "";  // Send the command to the submerged unit
  if (Serial.available()) {
    while (Serial.available()) {
      data = Serial.readString();
      return data;
    }
  }
  return data;
}

int receiveLoRa(float depths[]) {
  String loraData = "";
  int packetSize = LoRa.parsePacket();
  int index = 0;
  if (packetSize) {
    while (LoRa.available()) {
      loraData = LoRa.readString();
    }
    String depth = "";
    for (char c : loraData) {
      if (c == ',') {
        depths[index] = depth.toFloat();
        index++;
        depth = "";
      } else {
        depth += c;
      }
    }
    if (depth.length() > 0) {
      depths[index] = depth.toFloat();
      index++;
    }
  }
  return index;  // Return the number of depths received
}

bool transmitLoRa(String data) {
  String response = "";
  bool status = false;
  do {
    LoRa.beginPacket();
    LoRa.print(data);
    LoRa.endPacket();
    response = receiveAckLoRa();
  } while (response != "Ack");
  status = true;
  return status;
}

String receiveAckLoRa() {
  String ackData = "";
  for (int i = 0; i < 100; i++) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      while (LoRa.available()) {
        ackData = LoRa.readString();
      }
      return ackData;
    }
  }
}

float readDepth() {
  String kedalaman = "";
  while (true) {
    if (Serial.available() > 0) {
      kedalaman = Serial.readStringUntil('\n');
      if (kedalaman.toFloat() > 0) {
        return kedalaman.toFloat();
      }
    }
  }
}

void turun() {
  int n = 0;
  while (n < 70) {
    digitalWrite(dir, LOW);
    digitalWrite(pul, HIGH);
    delayMicroseconds(500);
    digitalWrite(pul, LOW);
    delayMicroseconds(500);
    n++;
  }
}

void naik() {
  int n = 0;
  while (n < 70) {
    digitalWrite(dir, HIGH);
    digitalWrite(pul, HIGH);
    delayMicroseconds(500);
    digitalWrite(pul, LOW);
    delayMicroseconds(500);
    n++;
  }
}

float depthControl(float target) {
  float depth;
  do {
    depth = readDepth();
    lcd.setCursor(0, 3);
    lcd.print("          ");
    lcd.setCursor(0, 3);
    lcd.print(String(depth) + " cm");
    // read serial comm for curr depth
    if (depth < target - 1) {
      turun();
    } else if (depth > target + 1) {
      naik();
    } else {
      return depth;
    }
  } while (depth != target);
}

bool depthStandby(float depth) {
  float target = 20;
  while (depth != target) {
    if (depth < target - 1) {
      turun();
    } else if (depth > target + 1) {
      naik();
    } else {
      return true;
    }
    depth = readDepth();
  }
}
