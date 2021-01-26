#include <ros/ros.h>
#include <location_monitor/LandmarkDistance.h>


#include <string>

using namespace std;

class closestLandmark {
    public:
        closestLandmark(){};
    
        void subscribeCallback(const location_monitor::LandmarkDistance::ConstPtr& msg){
            string name = msg->name;
            double distance = msg->distance;
            ROS_INFO("The closest landmark is: %s", name.c_str());
            ROS_INFO("And the current distance to it is : %f", distance);
        };

};




int main(int argc, char** argv){
    ros::init(argc, argv, "test_subscriber");
    ros::NodeHandle nh;

    closestLandmark subscriber;

    //define a subscriber
    ros::Subscriber sub = nh.subscribe("closest_landmark", 
    10,
    &closestLandmark::subscribeCallback,
    &subscriber);
    
    ros::spin();
    return 0;
}