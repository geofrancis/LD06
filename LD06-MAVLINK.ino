
#include "mavlink/common/mavlink.h"  // Mavlink interface
#include "mavlink/common/mavlink_msg_obstacle_distance.h"
#include "mavlink/common/mavlink_msg_rc_channels.h"
#include "mavlink/common/mavlink_msg_servo_output_raw.h"


unsigned long previousMillis = 0;
const long interval = 200;

int lidarAngle = 0;
int messageAngle = 0;
uint16_t distances[72];
int target = 0;
unsigned char data_buffer[4] = { 0 };
int adjusted = 0;
int distance = 0;
int range = 0;
unsigned char CS;
uint8_t Index;
byte received;
char serial_buffer[15];



#include "ld06.h"
LD06 ld06(Serial1);  // ld06 constructor, need to specify the hardware serial you want to use with the ld06 ( You can use Serial instead of Serial1 on arduino uno )


void setup() {

  Serial.begin(115200);   // Start the Serial you want to display data
  ld06.init();            // Init Serial Lidar to 230400 Bauds and set lidar pwm pin mode if pwm pin is specified
  Serial2.begin(115200);  // 8,9
  request_datastream();
}

void loop() {


  if (ld06.readScan()) {  // Read lidar packets and return true when a new full 360° scan is available
   uint16_t n = ld06.getNbPointsInScan();  // Give the number of points in the scan, can be usefull with filtering to tell if there are abstacles around the lidar
    for (uint16_t i = 0; i < n; i++) {
      //Serial.println(String() + ld06.getPoints(i)->angle + "," + ld06.getPoints(i)->distance + ";");  // example to show how to extract data. ->x, ->y and ->intensity are also available.
      lidarAngle = ld06.getPoints(i)->angle;
      messageAngle = map(lidarAngle, 0, 360, 0, 72);
      distances[messageAngle] = (ld06.getPoints(i)->distance / 10);
      Serial.print(messageAngle);
      Serial.print(" ");
      Serial.println(distances[messageAngle]);
    }
        if (ld06.isNewScan()) {  // Even if fullScan is disabled you can know when last data chunk have a loop closure
     MAVLINK();
    }

  }
}
