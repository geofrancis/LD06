
#include "mavlink/common/mavlink.h"        // Mavlink interface
#include "mavlink/common/mavlink_msg_obstacle_distance.h"
#include "mavlink/common/mavlink_msg_rc_channels.h"
#include "mavlink/common/mavlink_msg_servo_output_raw.h"

UART Serial2(4, 5, 0, 0);

#include <NeoPixelConnect.h>
NeoPixelConnect p(16, 1, pio0, 0);

unsigned long previousMillis = 0;
const long interval = 200;

int lidarAngle = 0;
int messageAngle = 0;
uint16_t distances[72];
int target = 0;
unsigned char data_buffer[4] = {0};
int adjusted = 0;
int distance = 0;
int range = 0;
unsigned char CS;
uint8_t Index;
byte received;

char serial_buffer[15];

#include "LD06forArduino.h"
#include <algorithm>
LD06forArduino ld06;

int fcmodein = 0;
int avoidaction = 0;
int steering = 0;
int active = 0;
int mindistance = 100;

int leftavg[72];
int rightavg[72];

char j, k;
long Lresult;
long Rresult;
long Aresult;


void setup()
{

  Serial.begin(115200); // USB
  Serial2.begin(115200); // FC
    delay(2000);  
  memset(distances, UINT16_MAX, sizeof(distances)); // Filling the distances array with UINT16_MAX
  ld06.Init(19);
  p.neoPixelFill(255, 0, 0, true);
  request_datastream();
  Serial.println("STARTUP");
}
int16_t Dist = 0;    // Distance to object in centimeters

void loop()
{
   
LD06_MAP();
LD06_Mavlink();
MavLink_receive();
LD06_lane();
MavLink_RC_out();


}
  
