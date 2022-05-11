#!/usr/bin/env roseus

;; load simple model for fast-boot
;; (require :go1 "package://unitreeeus/go1-simple.l")

;; copied from https://github.com/jsk-ros-pkg/jsk_demos/blob/0a82540da013b8e21307354bb1c6a552185e36a2/jsk_spot_watch_dog/scripts/watch-dog.l
(load "package://unitreeeus/unitree-interface.l")
(load "package://pr2eus/speak.l")

(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "nav_msgs")


(defclass watchdog-interface
  :super propertied-object
  :slots (luminance ypr-euler luminance-threshold stand-start-time yaw-offset battery-temp temp-threshold human-rects rest-time-per-hour human-width-threshold human-height-threshold max-tracking-yaw paranoma-width paranoma-height lookup-angle change-body-duration bark-sound))
(defmethod watchdog-interface
  (:init
   (&rest args)
   (let ()
     (setq luminance-threshold (ros::get-param "~luminance_threshold" 0))
     (setq human-height-threshold (ros::get-param "~human_height_threshold" 0.0))
     (setq rest-time-per-hour (ros::get-param "~rest_time_per_hour" 0))
     (setq human-width-threshold (ros::get-param "~human_width_threshold" 2.0))
     (setq max-tracking-yaw (ros::get-param "~max_tracking_yaw" 0.0))
     (setq lookup-angle  (ros::get-param "~lookup_angle" -0.4)) ;; radian
     (setq change-body-duration (ros::get-param "~change_body_duration" 1.0))
#|
     (setq temp-threshold (ros::get-param "~temp_threshold" 50))
|#
     (setq bark-sound (pathname (ros::get-param "~bark_sound" "/home/pi/Music/bark.wav")))
#|
     (let ((ros-img (one-shot-subscribe "~panorama_image" sensor_msgs::Image)))
       (setq paranoma-width (send ros-img :width))
       (setq paranoma-height (send ros-img :height)))
     (ros::subscribe "~input" sensor_msgs::Image #'send self :image-cb)
|#
     (ros::subscribe "/people_pose" jsk_recognition_msgs::PeoplePoseArray #'send self :detect-cb)
#|
     (ros::subscribe "/spot/odometry" nav_msgs::Odometry #'send self :odom-cb)
|#
     (setq stand-start-time (ros::time-now))
#|
     (setq yaw-offset 0)
     (setq luminance 0)
     (setq battery-temp 0)
|#
     (setq ypr-euler '(0 -0.3 0))
     ))
#|
  (:spot-status-battery-states-callback  (msg)   ;; overwrite
                                         (send-super :spot-status-battery-states-callback msg)
                                         (setq battery-temp (apply #'max (coerce (send (elt (send msg :battery_states) 0) :temperatures) cons)))
                                         (ros::ros-debug "battery temp: ~A" battery-temp)
                                         )
  (:image-cb (msg)
             (let ((img (instance grayscale-image :init
                                  (send msg :width)
                                  (send msg :height)
                                  (copy-object (send msg :data)))))
               (dotimes (w (send img :width))
                 (dotimes (h (send img :height))
                   (setq luminance (+ luminance (send img :pixel w h)))))
               (setq luminance (/ luminance (* (send img :width) (send img :height))))
               (ros::ros-debug "receive ~A, luminance is ~A" (send (send msg :header) :frame_id) luminance)
               t)
             )
|#
  (:detect-cb (msg)
              (let (bbox largest-box)
                (ros::ros-info (format nil "detect callback ~A~%" msg))
                (setq bbox (mapcar #'(lambda (p)
                                       (geo::make-bounding-box (mapcar #'(lambda (x)
                                                                           (float-vector (send x :position :x) (send x :position :y)))
                                                                       (send p :poses)))) (send msg :poses)))
                (setq bbox (sort bbox #'> #'(lambda (x) (* (elt (send x :diagonal) 0) (elt (send x :diagonal) 1)))))
                (setq human-rects (instance jsk_recognition_msgs::RectArray :init
                                            :rects
                                            (mapcar #'(lambda (x) (instance jsk_recognition_msgs::Rect :init
                                                                            :x (elt (send x :center) 1)
                                                                            :y (elt (send x :center) 0)
                                                                            :width (elt (send x :diagonal) 1)
                                                                            :height (elt (send x :diagonal) 0))) bbox)))
                (setq human-rects (send human-rects :rects))))
#|
  (:odom-cb (msg)
            (let ()
              (setq q (float-vector
                      (send msg :pose :pose :orientation :w)
                      (send msg :pose :pose :orientation :x)
                      (send msg :pose :pose :orientation :y)
                      (send msg :pose :pose :orientation :z)))
              (setq ypr-euler (elt (rpy-angle (quaternion2matrix q)) 0))
              (if (and
                   (< (abs (elt (send *ri* :get-val 'ypr-euler) 2)) 0.05)
                   (< (abs (elt (send *ri* :get-val 'ypr-euler) 1)) 0.05))
                  (send *ri* :set-val 'yaw-offset (elt (send *ri* :get-val 'ypr-euler) 0)))
              (ros::ros-debug  "body quaternion is ~A, YPR euler is ~A ~%" q ypr-euler)
              ))
  (:bright-surrounding () (if (> luminance luminance-threshold) t nil))
  ;;(:high-battery-temp () (if (> battery-temp temp-threshold) t nil))
  (:high-battery-temp () (if (> battery-temp temp-threshold)
                             (progn
                               (ros::ros-info "bat temp: ~A; temp thresh: ~A" battery-temp temp-threshold)
                             t)
		   nil))
|#
  )

(defun watchdog-init (&optional (create-viewer))
  (unless (ros::ok) (ros::roseus "robot_watchdog"))
  (setq *wi* (instance watchdog-interface :init))
  (ros::spin-once)
  (when create-viewer (objects (list *robot*)))
  (send *wi* :get-val 'bark-sound))


(watchdog-init nil)


(do-until-key
    (dolist (rect (send *wi* :get-val 'human-rects))
      (ros::ros-info "received rect center: ~A ~A, size: ~A ~A" (send rect :x) (send rect :y) (send rect :width) (send rect :height))
      (speak (send *wi* :get-val 'bark-sound) :topic-name "sound_play" :wait t)
      (unix:usleep 10000000)
      )
  (ros::spin-once)
  (ros::sleep)
  )
