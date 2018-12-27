#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(ros::roseus "talker")
(ros::advertise "chatter" std_msgs::string 1)
(ros::rate 10)
(while (ros::ok)
  (setq msg (instance std_msgs::string :init))
  (send msg :data (format nil "hello world ~a" (send (ros::time-now) :sec-nsec)))
  (ros::ros-info "msg [~A]" (send msg :data))
  (ros::publish "chatter" msg)
  (ros::sleep))
(exit)
