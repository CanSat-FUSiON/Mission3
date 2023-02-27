// Camera communication settings
#define CAMERA_START_BYTE 0x80
#define CAMERA_PACKET_SIZE 2//1+1
bool newCameraData = 0;
int heading = -999;

void setup() {
  Serial2.begin(115200);
  Serial.begin(115200);
}

void loop() {
  // Loops while the amount of data is >= the packet size
  while (Serial2.available() >= CAMERA_PACKET_SIZE) {
    uint8_t first = Serial2.read();

    // Makes sure the first byte is the start of packet indicator, otherwise keeps looping
    if (first == CAMERA_START_BYTE) {
      newCameraData = true;

      uint8_t dataBuffer[CAMERA_PACKET_SIZE - 1];

      // Put all the data into an array
      for (int i = 0; i < CAMERA_PACKET_SIZE - 1; i++) {
        dataBuffer[i] = Serial2.read();
      }

      heading = (dataBuffer[0] << 7) | dataBuffer[1];
    }
  }
  
  if (newCameraData) {
    // print the new x and y coordinates of the cone in a 240x240 window
    Serial.print("Heading: ");
    Serial.println(heading);
    // topleft = (0, 0); bottomright = (240, 240)
    newCameraData = 0;
  }
}
