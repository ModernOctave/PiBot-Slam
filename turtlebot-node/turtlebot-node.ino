#define TCP false

#if TCP
  #include "ArduinoTcpHardware.h"
#else
  #include "ArduinoHardware.h"
#endif

#include "wheel.h"
#include <PID_v1.h>
#include <WiFi.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>


#if TCP
  // WiFi Variables
  const char *ssid = "Redmi 9";
  const char *password = "123456789";
  IPAddress server(192, 168, 43, 94);
  const uint16_t serverPort = 11311;
#endif


// PID variables (180,180,0)
double SetpointR = 0, SetpointL = 0, setR = 0, setL = 0;
double Kp = 60, Ki = 300, Kd = 0;
PID PIDR(&omegaR, &setR, &SetpointR, Kp, Ki, Kd, DIRECT);
PID PIDL(&omegaL, &setL, &SetpointL, Kp, Ki, Kd, DIRECT);


// ROS
float x, z;
ros::NodeHandle nh;

void velCallback(const geometry_msgs::Twist& vel)
{
  x = vel.linear.x;
  z = vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> vel_sub("/cmd_vel", &velCallback);


void setup()
{
  Serial.begin(57600);

  #if tcp
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    // Connect the ESP to the wifi AP
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // Set the connection to rosserial socket server
    nh.getHardware()->setConnection(server, serverPort);
  #else
    nh.getHardware()->setBaud(57600);
  #endif

  // Wheel driver setup
  wheelSetup();

  // PID setup
  PIDR.SetMode(AUTOMATIC);
  PIDL.SetMode(AUTOMATIC);
  PIDR.SetOutputLimits(-255, 255);
  PIDL.SetOutputLimits(-255, 255);

  // ROS node setup
  nh.initNode();
  nh.subscribe(vel_sub);
}

void loop()
{
  // SetpointR = (x - z * AXIL_LENGTH / 2) / (WHEELDIAMETER / 2);
  // SetpointL = (x + z * AXIL_LENGTH / 2) / (WHEELDIAMETER / 2);
  SetpointR = 2;
  Serial.println(SetpointR - omegaR);

  PIDR.Compute();
  PIDL.Compute();
  motorWrite(RIGHT, setR);
  motorWrite(LEFT, setL);

  nh.spinOnce();
}
