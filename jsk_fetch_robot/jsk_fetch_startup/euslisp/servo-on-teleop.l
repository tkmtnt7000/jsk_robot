#!/usr/bin/env roseus

(ros::roseus-add-msgs "sensor_msgs")

(load "package://fetcheus/fetch-interface.l")


(defclass servo-on-teleop
    :slots ())
(defmethod servo-on-teleop
  (:init ()
   "create class for servo-on-teleop"
   (ros::roseus "servo-on-teleop")
   (let* ((msg (one-shot-subscribe "/joint_states"
                                   sensor_msgs::JointState
                                   :timeout 5000)))
     (unless msg
       (ros::ros-error "Failed to initialize servo-on-teleop node")
       (unix::exit 1))
   (ros::ros-info "init")
   (fetch-init)
   (ros::subscribe "/joy" sensor_msgs::Joy #'send self :cb)
   (ros::ros-info "start servo-on-teleop")))
  (:cb (msg)
   ;; use floor to convert float to integer
   (when
     (and
      (> (elt (send msg :buttons) (floor (ros::get-param "/teleop/base/button_deadman" 10))) 0)
      (> (elt (send msg :buttons) (floor (ros::get-param "/teleop/button_servo_on" 4))) 0)
      (= (elt (send msg :buttons) (floor (ros::get-param "/tuck_arm/tuck_button"))) 0))
       ;; because the dualshock cross button is associated with an adjacent button
     (send self :servo-on)))
  (:servo-on ()
   (send *ri* :servo-on) ;; arm, head, gripper servo on
   (ros::ros-info "Enable motor position holding")))


(setq m (instance servo-on-teleop :init))
(ros::rate 1)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))
