
void request_datastream() {
  //Request Data from Pixhawk
  uint8_t _system_id = 255;       // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2;      // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1;     // Id # of Pixhawk (should be 1)
  uint8_t _target_component = 0;  // Target component, 0 = all (seems to work with 0 or 1
  uint8_t _req_stream_id = MAV_DATA_STREAM_ALL;
  uint16_t _req_message_rate = 0x08;  //number of times per second to request the data in hex
  uint8_t _start_stop = 1;            //1 = start, 0 = stop

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

  Serial2.write(buf, len);  //Write data to serial port
}



void MAVLINK_PROX() {

  int sysid = 1;
  int compid = 196;
  uint64_t time_usec = 0;
  uint8_t sensor_type = 0;
  uint8_t increment = 5;
  uint16_t min_distance = 10;
  uint16_t max_distance = 1300;
  float increment_f = 0;
  float angle_offset = 0;
  uint8_t frame = 12;
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  uint8_t system_mode = MAV_MODE_PREFLIGHT;  ///< Booting up
  uint32_t custom_mode = 30;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY;  ///< System ready for flight

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  int type = MAV_TYPE_GROUND_ROVER;
  // Pack the message

  mavlink_msg_obstacle_distance_pack(sysid, compid, &msg, time_usec, sensor_type, distances, increment, min_distance, max_distance, increment_f, angle_offset, frame);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
}

void MAVLINK_HB() {

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
    uint8_t system_mode = MAV_MODE_PREFLIGHT;  ///< Booting up
    uint32_t custom_mode = 30;                 ///< Custom mode, can be defined by user/adopter
    uint8_t system_state = MAV_STATE_STANDBY;  ///< System ready for flight
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int type = MAV_TYPE_GROUND_ROVER;
    // Pack the message

    mavlink_msg_heartbeat_pack(1, 196, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
  }
}


void MAP_MAVLINK() {

  if (ld06.readScan()) {                    // Read lidar packets and return true when a new full 360° scan is available
    uint16_t n = ld06.getNbPointsInScan();  // Give the number of points in the scan, can be usefull with filtering to tell if there are abstacles around the lidar
    for (uint16_t i = 0; i < n; i++) {
      //Serial.println(String() + ld06.getPoints(i)->angle + "," + ld06.getPoints(i)->distance + ";");  // example to show how to extract data. ->x, ->y and ->intensity are also available.
      lidarAngle = ld06.getPoints(i)->angle;
      messageAngle = map(lidarAngle, 0, 360, 0, 72);
      distances[messageAngle] = (ld06.getPoints(i)->distance / 10);
      //Serial.print(messageAngle);
      //Serial.print(" ");
      //Serial.println(distances[messageAngle]);
    }
  }
}

