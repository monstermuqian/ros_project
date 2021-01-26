#include <ros/ros.h>
#include <location_monitor/GetClosest.h>
#include <location_monitor/GetDistance.h>
#include <vector>
#include <string>
#include <iostream>

using namespace std;


int main (int argc, char** argv){
    ros::init(argc, argv, "monitor_client");
    ros::NodeHandle nh;

    ros::ServiceClient get_closest = nh.serviceClient<location_monitor::GetClosest>("get_closest");
    ros::ServiceClient get_distance = nh.serviceClient<location_monitor::GetDistance>("get_distance");

    location_monitor::GetClosestRequest req;
    location_monitor::GetClosestResponse res;

    // Because the service "get_closest" doestn't need any
    // input. So I can just use it without collecting input data
    // in this node.
    get_closest.call(req, res);

    vector<string> landmark  = {"cylinder1","cylinder2","cylinder3","cylinder4","cylinder5",
                                                              "cylinder6","cylinder7","cylinder8","cylinder9",};

    for(const string &name : landmark){
        location_monitor::GetDistanceRequest d_req;
        // here input the values as "requests" to use the service
        d_req.LandmarkName = name;
        location_monitor::GetDistanceResponse d_res;
        get_distance.call(d_req, d_res);
        cout << "name :" << d_req.LandmarkName << "distance :" << d_res.DistanceToRobot << endl;
    };

    ros::spinOnce();
    return 0;
}