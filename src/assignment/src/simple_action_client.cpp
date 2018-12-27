#include <ros/ros.h>
#include <assignment/DoDishesAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib ::SimpleActionClient<assignment::DoDishesAction> Client;

int main(int argc, char **argv){
    ros::init(argc, argv, "do_dishes_client");
    Client client("do_dishes", true);
    client.waitForServer();
    assignment::DoDishesGoal goal;
    client.sendGoal(goal);
    client.waitForResult(ros::Duration(5.0));
    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        printf("Yay! The dishes are now clean");
    }
    printf("Current State: %s\n", client.getState().toString().c_str()) ;
    return 0;
}