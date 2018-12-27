#include <ros/ros.h>
#include <assignment/Capitalize.h>
#include <iostream>

int main(int argc, char **argv){
    ros::init(argc, argv, "capitalize_client");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<assignment::Capitalize>("capitalize");

    assignment::Capitalize srv;
    srv.request.words = argv[1];
    //srv.request.words = "hello";
    if(client.call(srv)){
        std::cerr << "Successfully Capitalized: " << srv.response.capitalized_words << std::endl;
    }else{
        std::cerr << "Failure" << std::endl;
    }

    return 0;
}

