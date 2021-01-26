#include <ros/ros.h>
#include  <nav_msgs/Odometry.h>
#include <location_monitor/LandmarkDistance.h>
#include <location_monitor/GetClosest.h>
#include <location_monitor/GetDistance.h>

#include <string>
#include <vector>
#include <math.h>

using namespace std;

//build a class to save landmark data
class Landmark{
    public:
        Landmark(string name, double x, double y):
        name(name), x(x), y(y){};
        string name;
        double x;
        double y;
};

//build a class to be monitor for giving back the information
//about landmark
class LandmarkMonitor{
    public:
        LandmarkMonitor(const ros::Publisher& landmark_pub): 
        landmark_() , landmark_pub_(landmark_pub){
            InitLandmark();
        }
        
        //using the service to share information
        //========================================
        //using class member landmark_ and both x_, y_ as current
        //postion of robot to detect the closest landmark
        bool GetClosestCallback(
                location_monitor::GetClosestRequest &request,
                location_monitor::GetClosestResponse &response){
                    ROS_INFO("GetClosest called");
                    location_monitor::LandmarkDistance closestLandmark = FindClosest(x_, y_);
                    response.LandmarkName = closestLandmark.name;
                    return true;
        }
        
        bool GetDistanceCallback(
            location_monitor::GetDistanceRequest &request,
            location_monitor::GetDistanceResponse &response){
            ROS_INFO("GetDistance called for : %s", 
            request.LandmarkName.c_str());
            string name = request.LandmarkName;
            double x_landmark  = 0;
            double y_landmark =  0;
            bool flag = false;
            for (int i = 0 ; i < landmark_.size() ; ++i){
                if(landmark_[i].name == name){
                    x_landmark = landmark_[i].x;
                    y_landmark = landmark_[i].y;
                    flag = true;
                }
            }
            if (flag == false){
                return false;
            }
            else{
                double dx = x_ - x_landmark;
                double dy = y_ - y_landmark;
                response.DistanceToRobot = sqrt(dx*dx + dy*dy);
                return true;
            };
        }
        
        //========================================



        //using the publishing way for sharing information
        //=========================================
        //make the odometry data monitor to be part of 
        //landmark monitor
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
            location_monitor::LandmarkDistance closestLandmark;
            x_ = msg->pose.pose.position.x;
            y_ = msg->pose.pose.position.y;
            closestLandmark = FindClosest(x_, y_);

            ROS_INFO("x: %f, y: %f, z: %f ", 
            msg->pose.pose.position.x, 
            msg->pose.pose.position.y, 
            msg->pose.pose.position.z);
            ROS_INFO("The closest landmark at this moment is : %s", closestLandmark.name.c_str());
            ROS_INFO("The distance to that landmark is : %f", closestLandmark.distance);
            landmark_pub_.publish(closestLandmark);
        }
        //=========================================


    private:
        vector<Landmark> landmark_;
        ros::Publisher landmark_pub_;
        double x_;
        double y_;

        location_monitor::LandmarkDistance FindClosest (double x, double y){
            location_monitor::LandmarkDistance result;
            result.distance = -1;
            double distance = 0;
            double dy = 0;
            double dx = 0;
            for (int i =  0; i < 9 ; i++){
                dy = y - landmark_[i].y;
                dx = x - landmark_[i].x;
                distance = sqrt(dx*dx + dy*dy);
                if ((result.distance < 0)||(distance < result.distance)){
                    result.name = landmark_[i].name;
                    result.distance = distance;
                };
            }
            return result;
        }
        void InitLandmark(){
            landmark_.push_back(Landmark("cylinder1", -1.1, -1.1));
            landmark_.push_back(Landmark("cylinder2", -1.1,   0    ));
            landmark_.push_back(Landmark("cylinder3", -1.1,   1.1 ));
            landmark_.push_back(Landmark("cylinder4",   0    ,  1.1));
            landmark_.push_back(Landmark("cylinder5",   0    ,   0   ));  
            landmark_.push_back(Landmark("cylinder6",   0    , -1.1));
            landmark_.push_back(Landmark("cylinder7",  1.1 , -1.1));
            landmark_.push_back(Landmark("cylinder8",  1.1,    0   ));
            landmark_.push_back(Landmark("cylinder9",  1.1, 1.1));                                                
        }
    
};

int main(int argc, char** argv){
    ros::init(argc, argv, "location_monitor");
    ros::NodeHandle nh;
    

    //pushing the information about landmarks in the way
    //of publishing
    //================================
    ros::Publisher landmark_pub = nh.advertise<location_monitor::LandmarkDistance>(
        "closest_landmark", 10);
    LandmarkMonitor monitor(landmark_pub);
    ros::Subscriber sub = nh.subscribe("odom",
     10,
      &LandmarkMonitor::OdomCallback,
      &monitor);
      //===============================

      //pushing the information about landmarks in the way
      // of sevice
      //==============================
      ros::ServiceServer get_closest = nh.advertiseService("get_closest", 
      &LandmarkMonitor::GetClosestCallback,  &monitor);
      ros::ServiceServer get_distance = nh.advertiseService("get_distance",
      &LandmarkMonitor::GetDistanceCallback, &monitor);
    //==============================

    ros::spin();
    return 0;
}





