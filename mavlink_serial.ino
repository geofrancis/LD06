
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



void LD06_MAP() {
  //Serial.println("MAP");
  for (int i = 0; i < ld06.data_length; i++) {
   Serial.print(i);
    lidarAngle = ld06.angles[i];
    messageAngle = map(lidarAngle, 0, 360, 0, 72);
    distances[messageAngle] = (ld06.distances[i] / 10);
    Serial.print("lidarAngle ");
    Serial.print(lidarAngle);
    Serial.print("distances");
    Serial.println(distances[messageAngle]);

    if (messageAngle > 18 && messageAngle < 27) {
      leftavg[72] = (ld06.distances[i] / 10);
    }
    if (messageAngle > 53 && messageAngle < 64) {
      rightavg[72] = (ld06.distances[i] / 10);
    }
  }
}



void LD06_lane() {
  //Serial.println("LANE");

  Lresult = 0;
  for (j = 0; j < 72; j++) {
    Lresult = Lresult + leftavg[j];
  }
  Lresult = Lresult / 72;  // this is our averaged result


  Rresult = 0;
  for (j = 0; j < 72; j++) {
    Rresult = Rresult + rightavg[j];
  }
  Rresult = Rresult / 72;  // this is our averaged result

  steering = ((Rresult - Lresult) + 1500);
}


void LD06_Mavlink() {
  //Serial.println("MAVLINK");
  int sysid = 1;
  int compid = 196;
  uint64_t time_usec = 0; /*< Time since system boot*/
  uint8_t sensor_type = 0;
  uint8_t increment = 5;
  uint16_t min_distance = 10;  /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 650; /*< Maximum distance the sensor can measure in centimeters*/
  float increment_f = 0;
  float angle_offset = -10;
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


  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    p.neoPixelFill(0, 255, 0, true);
    mavlink_msg_heartbeat_pack(1, 196, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
  }
}


void MavLink_receive() {
  //Serial.println("MAVIN");
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial2.available()) {
    uint8_t c = Serial2.read();

    //Get new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      p.neoPixelFill(0, 0, 255, true);
      //Handle new message from autopilot
      switch (msg.msgid) {


        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:  // #35
          {
            mavlink_rc_channels_raw_t chs;
            mavlink_msg_rc_channels_raw_decode(&msg, &chs);
            Serial.print("Chanel 6 (3-Kanal Schalter): ");
            Serial.println(chs.chan6_raw);
            if (chs.chan8_raw > 1500) { active = 1; }
            if (chs.chan8_raw < 1500) { active = 0; }
          }
          break;
      }
    }
  }
}


void MavLink_RC_out() {
  {
    //Serial.println("MAVOUT");
    if (active = 1) {
      int i;

      int system_type = 250;
      int autopilot_type = MAV_COMP_ID_ALL;

      // Initialize the required buffers
      mavlink_message_t msg;
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];

      mavlink_rc_channels_override_t rcChannels;

      rcChannels.chan4_raw = steering;

      rcChannels.target_system = system_type;
      rcChannels.target_component = autopilot_type;

      mavlink_msg_rc_channels_override_encode(system_type, autopilot_type, &msg, &rcChannels);

      // Copy the message to the send buffer
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial2.write(buf[i]);
    }
  }
}