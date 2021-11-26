#!/usr/bin/env roseus

(ros::roseus "tuck_arm")
(ros::roseus-add-srvs "std_srvs")
;; (load "package://spotkinova/spotkinova-interface.l")
;; (spotkinova-init nil)

;; (defun tuck-arm ()
;;   (send *ri* :state :potentio-vector)
;;   (send *ri* :angle-vector (send *spotkinova* :kinova-rest-pose) :fast))

;; (defun reset-pose ()
;;   (send *ri* :state :potentio-vector)
;;   (send *ri* :angle-vector (send *spotkinova* :reset-pose) :fast))

;; (defun tuck-arm-server (req)
;;   (let ((hoge nil))
;;     (when req
;;       (tuck-arm))))

;; (ros::roseus "tuck_arm_server")
;; (ros::advertise-service "tuck_arm" roseus::hoge #'tuck-arm-server)

;; header: 
;;   seq: 6704127
;;   stamp: 
;;     secs: 1637734668
;;     nsecs: 533706832
;;   frame_id: "/dev/input/js_dualsense"
;; axes: [0.0, 0.0, -0.0, 1.0, 1.0, 0.0, -0.0, -0.0]
;; buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

(defclass tuck-arm-teleop-class
    :slots ((dualsense-deadman (ros::get-param
                                       "/teleop_twist_joy_dualsense/enable_button" 4))
            (dualsense-tuck (ros::get-param
                                    "/teleop_twist_joy_dualsense/tuck_button" 6))
            (R1-deadman (ros::get-param
                                "/teleop_twist_joy_dualsense/enable_button" nil))))
(defmethod tuck-arm-teleop-class
  (:init ()
   (load "package://spotkinovaeus/spotkinova-interface.l")
   (spotkinova-init)
   (ros::subscribe "/joy_dualsense" sensor_msgs::Joy #'send self :dualsense-cb)
   (ros::subscribe "/joy_R1" sensor_msgs::Joy #'send self :r1-cb))
  (:dualsense-cb (msg)
   ;; (ros::ros-info "deadman:~A~%" (elt (send msg :buttons) (floor (read-from-string dualsense-deadman))))
   ;; (ros::ros-info "tuck:~A~%" (elt (send msg :axes) 6));;(floor dualsense-tuck)))
   (when (and (> (elt (send msg :buttons) 4) 0)
              (< (elt (send msg :axes) 6) 0))
     (send self :tuck-arm)))
  (:tuck-arm ()
   (send *ri* :state :potentio-vector)
   (send *ri* :angle-vector (send *spotkinova* :kinova-rest-pose) 10000))
   (send *ri* :wait-interpolation)
  )

(defclass kinova-arm-rest-server
  :slots ())
(defmethod kinova-arm-rest-server
  (:init ()
   (load "package://spotkinovaeus/spotkinova-interface.l")
   (spotkinova-init)
   (ros::advertise-service "/arm_gen3/kinova_rest" std_srvs::Trigger #'send self :rest-arm))
  (:rest-arm (req)
    (let ((m (send req :response)))
      ;;(send *ri* :state :potentio-vector)
;;      (send *ri* :wait-interpolation)
      (send m :success
            (progn
              (send *ri* :state :potentio-vector)
              (send *ri* :angle-vector (send *spotkinova* :kinova-rest-pose) 10000)
              (send *ri* :wait-interpolation)))
      m)))
(setq a (instance kinova-arm-rest-server :init))
(ros::rate 1)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))
