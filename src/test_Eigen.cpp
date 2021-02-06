#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <location_monitor/MatrixResult.h>

using namespace  std;
//using namespace Eigen::Eigen;


int main(int argc, char** argv){
    ros::init(argc, argv, "test_Eigen");
    ros::NodeHandle nh;

    Eigen::MatrixXd A(2, 1);
    A << 1 , 2 ;
    Eigen::MatrixXd B(1, 2);
    B << 1, 1;
    cout<< A <<endl;
    cout<< B <<endl;
    Eigen::MatrixXd c(1,1) ;
    c =  B * A;

    location_monitor::MatrixResult result;
    result.result =  c(0,0);

    ros::Publisher matrix_pub = nh.advertise<location_monitor::MatrixResult>("testEigen", 10);

 while(true){   
    matrix_pub.publish(result);


    ros::spin();
 }
    return 0;

    
}