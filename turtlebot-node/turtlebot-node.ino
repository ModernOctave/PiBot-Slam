#define TCP true
#define BAUD 115200

#if TCP
  #include "ArduinoTcpHardware.h"
#else
  #include "ArduinoHardware.h"
#endif

#include "env.h"
#include "wheel.h"
#include <PID_v1.h>
#include <WiFi.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>


#if TCP
  // WiFi Variables
  const char *ssid = SSID;
  const char *password = PASSWORD;
  IPAddress server(192, 168, 43, 94);
  const uint16_t serverPort = 11411;
#endif

long currentMillis = 0, previousMillis = 0;

// PID variables (0,300,0)
double SetpointR = 0, SetpointL = 0, setR = 0, setL = 0;
double Kp = 50, Ki = 300, Kd = 0;
PID PIDR(&omegaR, &setR, &SetpointR, Kp, Ki, Kd, DIRECT);
PID PIDL(&omegaL, &setL, &SetpointL, Kp, Ki, Kd, DIRECT);


// ROS
ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);
tf::TransformBroadcaster broadcaster;

float x, z;

void velCallback(const geometry_msgs::Twist& vel)
{
  x = vel.linear.x;   // In meter per second
  z = vel.angular.z;  // In radian per second
}

ros::Subscriber<geometry_msgs::Twist> vel_sub("/cmd_vel", &velCallback);


void setup()
{
  Serial.begin(BAUD);

  #if TCP
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
    nh.getHardware()->setBaud(BAUD);
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
  broadcaster.init(nh);
}

void loop()
{
  // Calculate Setpoints in revolutions per second
  // We get value in radians per second by kinematics equations
  // 2*pi radians per revolution
  // Hence divide by 2*pi to get revolutions per second
  SetpointR = (x * 100 + z * AXIL_LENGTH / 2) / (WHEELDIAMETER / 2) / (2 * PI);
  SetpointL = (x * 100 - z * AXIL_LENGTH / 2) / (WHEELDIAMETER / 2) / (2 * PI);
  // Serial.println(SetpointR - omegaR);

  PIDR.Compute();
  PIDL.Compute();
  motorWrite(RIGHT, setR);
  motorWrite(LEFT, setL);

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10)
  {
    previousMillis = currentMillis;
    // Broadcast the transform
    t.header.stamp = nh.now();
    t.header.frame_id = "/odom";
    t.child_frame_id = "/base_link";
    t.transform.translation.x = pose_x / 100;
    t.transform.translation.y = pose_y / 100;
    t.transform.translation.z = 0;
    t.transform.rotation = tf::createQuaternionFromYaw(pose_theta);
    broadcaster.sendTransform(t);


    // Publish the odometry
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = "/odom";
    odom_msg.pose.pose.position.x = pose_x / 100;
    odom_msg.pose.pose.position.y = pose_y / 100; 
    odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(pose_theta);
    odom_msg.twist.twist.linear.x = (velocityR + velocityL) / 2;
    odom_msg.twist.twist.angular.z = (velocityR - velocityL) / AXIL_LENGTH;
    // odom_pub.publish(&odom_msg);
  }


  // ROS
  nh.spinOnce();
}
