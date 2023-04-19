#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <kobuki_msgs/WheelDropEvent.h>
#include <kobuki_msgs/BumperEvent.h>
#include <cstdlib>
#include <sensor_msgs/LaserScan.h>
#include <thread>

#define DEG2RAD(deg) ((deg) * M_PI / 180.);

using namespace std;
geometry_msgs::Twist follow_cmd;


//Different Emotional States
int worldState = 0;
int followerState = 0;
int sadnessState = 0;
int angerState = 0;
int fearState = 0;
int infatuationState = 0;

//for wheel drop callback function
const uint8_t N_WHEELS = 2;
uint8_t wheel[N_WHEELS] = {kobuki_msgs::WheelDropEvent::RAISED, kobuki_msgs::WheelDropEvent::RAISED};
bool wheelDrop = false;

//for bumper callback function
const uint8_t N_BUMPER = 3;
uint8_t bumper[N_BUMPER] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED };
bool anyBumperPressed = false;
uint8_t lastBumperPressed = 0;

//for laser callback function
float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers=0, desiredNLasers=0, desiredAngle=5;
float avgLaserDist = 0;
const float personThreshold = 2.5;



//follower callback function
void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

//bumper callback function
void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg){
	bumper[msg->bumper] = msg->state;

	if(msg->state == kobuki_msgs::BumperEvent::RELEASED){
		ROS_INFO("%d released", msg->bumper);
	}
	else if(msg->state == kobuki_msgs::BumperEvent::PRESSED){
		anyBumperPressed = true;
		ROS_INFO("%d bumper impact detected", msg->bumper);

		//change state if hit
		worldState = angerState;
	}
}

//wheel drop callback function
void wheelDropCB(const kobuki_msgs::WheelDropEvent::ConstPtr& msg){
    const int wheelIndex = msg->wheel;
    const int state = msg->state;
    
    if (wheelIndex < 0 || wheelIndex > 1){
        ROS_INFO("Invalid wheel number: %d", wheelIndex);
        return;
    }
    
    wheel[wheelIndex] = state;
    
    //if both wheels are dropped, set to infatuation
    if (wheel[0] == kobuki_msgs::WheelDropEvent::DROPPED && wheel[1] == kobuki_msgs::WheelDropEvent::DROPPED) {
        worldState = infatuationState;
    }
    //if only one wheel is dropped set to fear
    else if (wheel[0] == kobuki_msgs::WheelDropEvent::RAISED && wheel[1] == kobuki_msgs::WheelDropEvent::DROPPED) {
        worldState = fearState;
    }
    else if (wheel[0] == kobuki_msgs::WheelDropEvent::DROPPED && wheel[1] == kobuki_msgs::WheelDropEvent::RAISED) {
        worldState = fearState;
    }
}


//this callback function returns the minimum laser distance
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    minLaserDist = std::numeric_limits<float>::infinity();
    avgLaserDist = 0;

    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);
    //ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);

    //calculate beginning and end of for loop based on desired angle
    int32_t beginIndex = 0, endIndex = nLasers;
    //ROS_INFO("endIndex: %i", endIndex);

    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        beginIndex = nLasers / 2 - desiredNLasers;
        endIndex = nLasers / 2 + desiredNLasers;
    }

    float min_dist = std::numeric_limits<float>::infinity();
    float sum_dist = 0;
    int count = 0;
    //loop through and find minimum and non zero values to get an average
    for(int i = beginIndex; i < endIndex; i++){
        float reading = msg->ranges[i];
        // Check if the reading is within a certain range
        if (reading >= msg->range_min && reading <= msg->range_max) {
            min_dist = std::min(min_dist, reading);
            sum_dist += reading;
            count++;
            //ROS_INFO("MIN_DIST Value: %.2fm", min_dist);
        }
    }

    //set minLaserDist based on min_dist
    minLaserDist = min_dist;
    
    //check if the distance is greater than the threshold, if it is
    //the robot has lost the person, therefore set state to sadness State
    if (minLaserDist > personThreshold){
	    worldState = sadnessState;
    }
	
    //if(minLaserDist == std::numeric_limits<float>::infinity()) minLaserDist = 0;
    ROS_INFO("Min Laser Distance: %.2fm, Avg Laser Distance: %.2fm", minLaserDist, avgLaserDist);
}

//Motion Functions\
//function to stop all movement for a set amount of seconds
void stopMovement(ros::Publisher vel_pub, geometry_msgs::Twist velocity, double stop){
	velocity.linear.x = 0;
	velocity.linear.z = 0;
	vel_pub.publish(velocity);
	ros::Duration(stop).sleep();
}

//crying motion
void cryingSweep(ros::Publisher vel_pub, geometry_msgs::Twist velocity, double angularVelocity, double sadTime) {
    ros::Time startTime = ros::Time::now();

    while ((ros::Time::now() - startTime).toSec() < sadTime) {
        // Rotate to the left
        velocity.angular.z = angularVelocity;
        velocity.linear.x = 0;
        vel_pub.publish(velocity);
        ros::Duration(0.1).sleep(); // Wait for the robot to turn left

        // Rotate to the right
        velocity.angular.z = -angularVelocity;
        velocity.linear.x = 0;
        vel_pub.publish(velocity);
        ros::Duration(0.1).sleep(); // Wait for the robot to turn right
    }

	ROS_INFO("Sad Motion Complete");
    // Stop the robot
    velocity.angular.z = 0;
    velocity.linear.x = 0;
    vel_pub.publish(velocity);
}


//motion for anger
void angerRotate(ros::Publisher vel_pub, geometry_msgs::Twist velocity, double rotationSpeed, double rotationTime) {
    double fullCircle = DEG2RAD(360);
    double numRotations = rotationTime / (fullCircle / rotationSpeed);
    velocity.linear.x = 0.0;

    //rotate the robot
    for (int i = 0; i < numRotations; i++) {
        velocity.angular.z = rotationSpeed;
        vel_pub.publish(velocity);
        ros::Duration(fullCircle / rotationSpeed).sleep();
    }

	ROS_INFO("Angry Motion Complete");
    //stop rotating
    velocity.angular.z = 0.0;
    vel_pub.publish(velocity);
}

//function to reverse the robot
void reverse(ros::Publisher vel_pub, geometry_msgs::Twist velocity, double distance, double speed) {
    double time = distance / std::abs(speed); // Time needed to travel the distance
    double start_time = ros::Time::now().toSec(); // Get the current time

    // Reverse the robot
    velocity.linear.x = speed;
    vel_pub.publish(velocity);

    // Keep reversing until the desired distance is reached
    while (ros::Time::now().toSec() - start_time < time) {
        ros::Duration(0.01).sleep();
    }

    // Stop the robot
	ROS_INFO("Reversing Complete");
    velocity.linear.x = 0;
    vel_pub.publish(velocity);
}

// motion for fear for a certain length of time
void fearMotion(ros::Publisher vel_pub, geometry_msgs::Twist velocity, double distance, double speed, double duration) {
    double startTime = ros::Time::now().toSec();

    while ((ros::Time::now().toSec() - startTime) < duration) {
        // move forward
        velocity.linear.x = speed;
        velocity.angular.z = 0.0;
        vel_pub.publish(velocity);
        ros::Duration(0.1).sleep();

        // stop the robot
        velocity.linear.x = 0.0;
        vel_pub.publish(velocity);
        ros::Duration(0.05).sleep();

        // move backward
        velocity.linear.x = -speed;
        velocity.angular.z = 0.0;
        vel_pub.publish(velocity);
        ros::Duration(0.1).sleep();

        // stop the robot
        velocity.linear.x = 0.0;
        vel_pub.publish(velocity);
        ros::Duration(0.05).sleep();
    }
	ROS_INFO("Fear Motion Complete");
}


//motion for infatuation
void infatuationCircles(ros::Publisher vel_pub, geometry_msgs::Twist velocity, double radius, double linearSpeed, double duration){
    double circumference = 2 * M_PI * radius;
    double distance = circumference;

    //calculate angular velocity
    double angularSpeed = linearSpeed / radius;
    double startTime = ros::Time::now().toSec();

    while((ros::Time::now().toSec() - startTime) < duration){
        velocity.linear.x = linearSpeed;
        velocity.angular.z = angularSpeed;
        vel_pub.publish(velocity);
        ros::Duration(0.01).sleep();
    }

	ROS_INFO("Infatuation Motion Complete");
    //stop the robot
    velocity.linear.x = 0;
    velocity.angular.z = 0;
    vel_pub.publish(velocity);
}

//this functions allows videos to be displayed
void displayVideo(const std::string& filePath){
    //open the video file using video capture
    cv::VideoCapture cap(filePath);

    //check if gif file was successfully opened
    if(!cap.isOpened()){
        ROS_INFO("Could not open file");
        return;
    }

    //create a full-screen window to display the gif
    cv::namedWindow("VIDEO", cv::WINDOW_FULLSCREEN);

    //read and display each frame of the gif file multiple times
    cv::Mat frame;
    while (cap.read(frame)){
        cv::imshow("VIDEO", frame);
        cv::waitKey(16.68333);   //wait for 16.68333ms
    }
    cap.set(cv::CAP_PROP_POS_FRAMES, 0); //set the video capture to the beginning of the file for the next loop

    //release the videocapture and destroy the window
    cap.release();
    cv::destroyWindow("VIDEO");
	ROS_INFO("Done Playing Video");
}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber wheel = nh.subscribe("mobile_base/events/wheel_drop", 10, &wheelDropCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);


	// contest count down timer
	ros::Rate loop_rate(10);
	std::chrono::time_point<std::chrono::system_clock> start;
	start = std::chrono::system_clock::now();
	uint64_t secondsElapsed = 0;

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	//define the states
	followerState = 0;
	sadnessState = 1;
	angerState = 2;
	fearState = 3;
	infatuationState = 4;

	//set the state to follow initially
	worldState = followerState;

	geometry_msgs::Twist vel;
	
	//time to stop robot at the beginning of each state
	double stopTime = 3.0;
	
	//for reversing
	double reverseDist = 0.2;
	double reverseSpeed = 0.5;
	
	//for sad state
	double sadVel = 0.5; 
	double sadTime = 30;
	
	//for anger state
	double angerSpeed = 2.0;
	double angerTime = 30;
	
	//for fear state
	double fearDist = 0.5;
	double fearSpeed = 1.0;
	double fearTime = 30;
	
	//for infatuation state
	double infatuationRadius = 0.5;
	double infatuationSpeed = 0.5;
	double infatuationTime = 40;
	
	//velocity variables
	double angular = 0;
	double linear = 2.0; 
	vel.angular.z = angular;
	vel.linear.x = linear;

	//Files when running on Kevon Computer
	//list of file paths for sadness
	std::string sadVideoPath = "/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/Sadness.mp4";
	string sadSoundPath = "/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/Sadness.wav";

	//list of file paths for anger
	std::string angerVideoPath = "/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/Anger.mp4";
	string angerSoundPath = "/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/Anger.wav";
	
	//list of file paths for fear
	std::string fearVideoPath = "/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/Fear.mp4";
	string fearSoundPath = "/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/Fear.wav";
	
	//list of file paths for infatuation
	std::string infatuationVideoPath = "/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/Infatuation.mp4";
	string infatuationSoundPath = "/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/Infatuation.wav";
	
	
	while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();
		
		//default state of the robot; follow state
		if (worldState == followerState){
			ROS_INFO("Follower State");
			//vel_pub.publish(follow_cmd);
			vel_pub.publish(vel);
		}
		
		//robot loses track of person it is following --> sadness state
		else if (worldState == sadnessState){
			ROS_INFO("Entering Sad State");
			
			//stop robot movement
			ROS_INFO("Stopping Robot");
			stopMovement(vel_pub, vel, stopTime);
			
			//play the video
			ROS_INFO("Beginning Video");
			std::thread sadVideoThread(displayVideo, sadVideoPath);

			//play the sound
			ROS_INFO("Beginning Sound");
			std::thread sadSoundThread([&sc, sadSoundPath](){sc.playWave(sadSoundPath);});

			//execute sad movement
			ROS_INFO("Beginning Sad Movement");
			cryingSweep(vel_pub, vel, sadVel, sadTime);
			
			sadVideoThread.join();
			sadSoundThread.join();

			//reset state
			ROS_INFO("Resetting to Follower State");
			worldState = followerState;
		}
		
		//bumper trigger --> anger state
		else if (worldState == angerState){
			ROS_INFO("Entering Anger State");
			
			//stop robot movement
			ROS_INFO("Stopping Robot");
			stopMovement(vel_pub, vel, stopTime);
			
			//reversing robot
			ROS_INFO("Reversing Robot");
			reverse(vel_pub, vel, reverseDist, reverseSpeed)

			//play the video
			ROS_INFO("Beginning Video");
			std::thread angerVideoThread(displayVideo, angerVideoPath);

			//play the sound
			ROS_INFO("Beginning Sound");
			std::thread angerSoundThread([&sc, angerSoundPath](){sc.playWave(angerSoundPath);});

			//anger movmement
			ROS_INFO("Beginning Angry Movement");
			angerRotate(vel_pub, vel, angerSpeed, angerTime);
			
			angerVideoThread.join();
			angerSoundThread.join();

			//reset state
			ROS_INFO("Resetting to Follower State");
			worldState = followerState;
		}
		
		//one wheel drop sensor triggered --> fear state
		else if (worldState == fearState){
			ROS_INFO("Entering Fear State");
			
			//stop robot movement
			ROS_INFO("Stopping Robot");
			stopMovement(vel_pub, vel, stopTime);

			//play the video
			ROS_INFO("Beginning Video");
			std::thread fearVideoThread(displayVideo, fearVideoPath);
			
			//play the sound
			ROS_INFO("Beginning Sound");
			std::thread fearSoundThread([&sc, fearSoundPath](){sc.playWave(fearSoundPath);});

			//execute fear motion
			ROS_INFO("Beginning Fear Movement");
			fearMotion(vel_pub, vel, fearDist, fearSpeed, fearTime);
			
			fearVideoThread.join();
			fearSoundThread.join();

			//reset state
			ROS_INFO("Resetting to Follower State");
			worldState = followerState;
		}
		
		//both wheel drop sensors --> infatuation state
		else if (worldState == infatuationState){
			ROS_INFO("Entering Infautation State");
			
			//stop robot movement
			ROS_INFO("Stopping Robot");
			stopMovement(vel_pub, vel, stopTime);

			//play the video
			ROS_INFO("Beginning Video");
			std::thread infatuationVideoThread(displayVideo, infatuationVideoPath);
			
			//play the sound
			ROS_INFO("Beginning Sound");
			std::thread infatuationSoundThread([&sc, infatuationSoundPath](){sc.playWave(infatuationSoundPath);});

			//infatuation motion
			ROS_INFO("Beginning Infatuation Movement");
			infatuationCircles(vel_pub, vel, infatuationRadius, infatuationSpeed, infatuationTime);
			
			infatuationVideoThread.join();
			infatuationSoundThread.join();

			//reset state
			ROS_INFO("Resetting to Follower State");
			worldState = followerState;
		}
		
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}
	return 0;
}
