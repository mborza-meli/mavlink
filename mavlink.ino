#include <SoftwareSerial.h>
#include "mavlink.h"
#include "math.h"

// Mavlink variables
mavlink_message_t msg;
mavlink_status_t status;
mavlink_gps_global_origin_t gps_origin;
mavlink_gps_raw_int_t gps_pos;

// Serial
#define SERIAL_BAULT 57600
SoftwareSerial bluetooth(3,4);

void setup()
{
  initializeSerial();
  Serial.println("Starting antenna tracker...");
  
  
  Serial.println("Setup ready!");
}

void loop()
{
  mavlink_receive();
}

void initializeSerial()
{
  Serial.begin(SERIAL_BAULT);
  bluetooth.begin(SERIAL_BAULT);
  delay(10);
}


void mavlink_receive() {
  // Prevent bluetooth blocked.. Dragonlink?
    bluetooth.flush();
    bluetooth.write("some");
    bluetooth.flush();
  //

  while(bluetooth.available()>0) {
    uint8_t c = bluetooth.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if(msg.msgid == MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN){
        mavlink_msg_gps_global_origin_decode(&msg, &gps_origin);
        Serial.print("ORIGIN ");
        Serial.print(gps_origin.latitude); Serial.print(" ");
        Serial.print(gps_origin.longitude); Serial.print(" ");
        Serial.println(gps_origin.altitude);
      }
      if(msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT){
        mavlink_msg_gps_raw_int_decode(&msg, &gps_pos);
        Serial.print("POSITION ");
        Serial.print(gps_pos.lat); Serial.print(" ");
        Serial.print(gps_pos.lon); Serial.print(" ");
        Serial.println(gps_pos.alt);
      }
    }
  }
  
}

// Some helper functions:
// distanceInMeters gets the distance between two points
double distanceInMeters(double lat1, double lon1, double lat2, double lon2) {
  double earthRadiusMeters = 6371000;
  double dLat = (lat2-lat1) * M_PI / 180.0;
  double dLon = (lon2-lon1) * M_PI / 180.0;

  lat1 = (lat1) * M_PI / 180.0;
  lat2 = (lat2) * M_PI / 180.0;

  double a = sin(dLat/2) * sin(dLat/2) +
          sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2); 
  double c = 2 * atan2(sqrt(a), sqrt(1-a)); 
  return earthRadiusMeters * c;
}

// getAltitudeAngle gets the angle between origin (0,0) and one point (x,y)
double getAltitudeAngle(double dis, double h){
  h = h/1000;
  if(h<=15){return 0;}
  return atan2(h,dis) * 180.0/M_PI;
}

// getRadialAngle gets the angle between two points, considering north 0º
double getRadialAngle(double lat1, double lon1, double lat2, double lon2){
  float dy = lat2 - lat1;
  float dx = cosf(M_PI/180*lat1)*(lon2 - lon1);
  float angle = atan2f(dy, dx) * 180.0/M_PI;

  if(angle < 0){ 
    angle = angle * -1;
  } else if(angle > 0){
    angle = 180+(180-angle);
  }

  angle += 90;
  if(angle >= 360) angle -= 360;
  
  return angle;
}
