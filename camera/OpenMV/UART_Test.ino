// Camera communication settings
#define CAMERA_START_BYTE 0x80
#define CAMERA_PACKET_SIZE 5 //1+2+2
bool newCameraData = 0;
int x_coordinate = -1;
int y_coordinate = -1;

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

      x_coordinate = (dataBuffer[0] << 7) | dataBuffer[1];
      y_coordinate = (dataBuffer[2] << 7) | dataBuffer[3];
    }
  }
  
  if (newCameraData) {
    // print the new x and y coordinates of the cone in a 240x240 window
    Serial.println("x: ");
    Serial.println(x_coordinate);
    Serial.println("y: ");
    Serial.println(y_coordinate);
    // topleft = (0, 0); bottomright = (240, 240)
    newCameraData = 0;
  }
}
