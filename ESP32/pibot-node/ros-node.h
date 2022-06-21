#define BAUD 115200
#define TCP false
#include "env.h"

#if TCP
#include "ArduinoTcpHardware.h"
#include <WiFi.h>
#else
#include "ArduinoHardware.h"
#endif

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

#if TCP
// WiFi Variables
IPAddress server(192, 168, 43, 94);
const uint16_t serverPort = 11411;
#endif

ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);
tf::TransformBroadcaster broadcaster;

float x, z;
long currentMillis = 0, previousMillis = 0;

void velCallback(const geometry_msgs::Twist &vel)
{
    x = vel.linear.x;  // In meter per second
    z = vel.angular.z; // In radian per second
}

ros::Subscriber<geometry_msgs::Twist> vel_sub("/cmd_vel", &velCallback);

void rosSetup()
{
#if TCP
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(mySSID);

    // Connect the ESP to the wifi AP
    WiFi.begin(mySSID, myPASSWORD);
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

    nh.initNode();
    nh.subscribe(vel_sub);
    nh.advertise(odom_pub);
    broadcaster.init(nh);
}

void rosLoop()
{
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
        odom_msg.header.stamp = nh.now();
        odom_msg.header.frame_id = "/odom";
        odom_msg.pose.pose.position.x = pose_x / 100;
        odom_msg.pose.pose.position.y = pose_y / 100;
        odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(pose_theta);
        odom_msg.twist.twist.linear.x = (velocityR + velocityL) / 2;
        odom_msg.twist.twist.angular.z = (velocityR - velocityL) / AXIL_LENGTH;
        odom_pub.publish(&odom_msg);
    }

    // ROS
    nh.spinOnce();
}