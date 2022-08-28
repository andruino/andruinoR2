#include <ros/ros.h>

//ROS libraries
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//ROS messages
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>

//Para el movimiento
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <string>

using namespace std;

//Globals
double collisionDistance = 2; //meters the ultrasonic detectors will flag obstacles
string publishedName;
char host[128];

int obstacle_Mode = 0;

float heartbeat_publish_interval = 2;

//Publishers
ros::Publisher obstaclePublish;
ros::Publisher heartbeatPublisher;

//Timers
ros::Timer publish_heartbeat_timer;

//Callback handlers
void sonarHandler(const sensor_msgs::Range::ConstPtr& sensorLeft, const sensor_msgs::Range::ConstPtr& sensorCenter, const sensor_msgs::Range::ConstPtr& sensorRight);
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);

int main(int argc, char** argv) {
    gethostname(host, sizeof (host));
    string hostname(host);
    
    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "! Obstacle module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No name selected. Default is: " << publishedName << endl;
    }

    ros::init(argc, argv, (publishedName + "_OBSTACLE"));
    ros::NodeHandle oNH;
    
    obstaclePublish = oNH.advertise<std_msgs::UInt8>((publishedName + "/obstacle"), 10);
    heartbeatPublisher = oNH.advertise<std_msgs::String>((publishedName + "/obstacle/heartbeat"), 1, true);
    
    //string cmd_vel_topic_name = robot_name;
    //cmd_vel_topic_name += "/cmd_vel";

    ros::Publisher cmd_vel_pub = oNH.advertise<geometry_msgs::Twist>((publishedName + "/cmd_vel"), 10);

    message_filters::Subscriber<sensor_msgs::Range> sensorLeftSubscriber(oNH, (publishedName + "/hc_sr04_1"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sensorCenterSubscriber(oNH, (publishedName + "/hc_sr04_2"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sensorRightSubscriber(oNH, (publishedName + "/hc_sr04_3"), 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;

    message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sensorLeftSubscriber, sensorCenterSubscriber, sensorRightSubscriber);
    sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

    publish_heartbeat_timer = oNH.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);

    //ros::spin();
    ros::spinOnce();   

    geometry_msgs::Twist moveForwardCommand;
    moveForwardCommand.linear.x = 0.5;

    geometry_msgs::Twist moveBackCommand;
    moveBackCommand.linear.x = -1;

    geometry_msgs::Twist turnRightCommand;
    turnRightCommand.angular.z = 1;
    
    geometry_msgs::Twist turnLeftCommand;
    turnLeftCommand.angular.z = -1;

    ros::Rate loop_rate(10);

    while (ros::ok()) {

        if (obstacle_Mode==1) {
            ROS_INFO("Turning right");
            cmd_vel_pub.publish(turnRightCommand);
        } else if (obstacle_Mode==2) {
            ROS_INFO("Turning left");
            cmd_vel_pub.publish(turnLeftCommand);
        }else if (obstacle_Mode==3) {
            ROS_INFO("Moving back");
            //cmd_vel_pub.publish(moveBackCommand); No puede ser para atrás porque luego anda hacia adelante y entra en bucle palante y patrás
	    cmd_vel_pub.publish(turnLeftCommand);
        }else {
            ROS_INFO("Moving forward");
            cmd_vel_pub.publish(moveForwardCommand);
        }

        ros::spinOnce(); // let ROS process incoming messages
        
        loop_rate.sleep();

    }

    //ros::spin();

   return EXIT_SUCCESS;
    //return 0;
}

void sonarHandler(const sensor_msgs::Range::ConstPtr& sensorLeft, const sensor_msgs::Range::ConstPtr& sensorCenter, const sensor_msgs::Range::ConstPtr& sensorRight) {
	std_msgs::UInt8 obstacleMode;
	

	if ((sensorLeft->range > collisionDistance) && (sensorCenter->range > collisionDistance) && (sensorRight->range > collisionDistance)) {
		obstacleMode.data = 0; //no collision
	}
	else if ((sensorLeft->range > collisionDistance) && (sensorRight->range < collisionDistance)) {
		obstacleMode.data = 1; //collision on right side
	}
	else {
		obstacleMode.data = 2; //collision in front or on left side
	}
	if (sensorCenter->range < collisionDistance) //block in front of center unltrasound.
	{
		obstacleMode.data = 3;
	}
	
        obstaclePublish.publish(obstacleMode);
        ROS_INFO("Info Obstaclemode: [%d]", obstacleMode.data);
	obstacle_Mode = obstacleMode.data;
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);
     ROS_INFO("yes");
}
