#include <ros/ros.h>
#include <std_msgs/Float64.h>

int count = 0;
double data = 0;
double mean, pre_mean = 0;
double variance = 0;

void callback(const std_msgs::Float64::ConstPtr &msg){
    data = msg->data; //アロー演算子使ってアクセスする
    if(count == 0){   //平均と分散を逐次更新
        mean = data;
    }else{
        pre_mean = mean;
        mean = (count * mean + data) / (count + 1);
        variance = (count * (variance + pre_mean * pre_mean) + data * data) 
                 / (count + 1) - mean * mean;
    }
    count++;
    ROS_INFO("[%d]: [%+8.7lf]\t mean: [%+8.7lf]\t variance: [%+8.7lf]\n", count, data, mean, variance);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "subscriber_cpp");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("rand", 10, callback);

    ros::spin();
}