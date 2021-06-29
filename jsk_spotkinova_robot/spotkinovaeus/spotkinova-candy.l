(defun candy ()
  (send *ri* :start-grasp)
  ;; (send *ri* :angle-vector-raw (send *spot-kinova* :angle-vector #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 -90.075 -17.0383 51.0735 -90.482 -108.916 -0.540145)) 10000)
  ;; (send *ri* :wait-interpolation)
  ;; (send *ri* :angle-vector-raw (send *spot-kinova* :angle-vector #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 -149.075 -17.0383 51.0735 -90.482 -108.916 -0.540145)) 10000)
  ;; (send *ri* :wait-interpolation)

  ;; angle-vector-rawではjoint_speedが早すぎるという計算結果が出ている
  (send *ri* :angle-vector (send *spot-kinova* :angle-vector #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 -120.075 -17.0383 51.0735 -90.482 -108.916 -0.540145)) 5000)
  (send *ri* :wait-interpolation)
  ;; caution! this plannninng cannot be solved by Moveit because of *co*
  (send *ri* :angle-vector-raw (send *spot-kinova* :angle-vector #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 -149.075 -17.0383 51.0735 -90.482 -108.916 -0.540145)) 10000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector-raw (send *spot-kinova* :head :move-end-pos #f(90 0 0)))
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp)
  (unix::sleep 1)
  (send *ri* :angle-vector-raw (send *spot-kinova* :head :move-end-pos #f(50 0 0)))
  (send *ri* :wait-interpolation)
  ;;(unix::sleep 1)
  (send *ri* :start-grasp)
  (unix::sleep 1)
  (send *ri* :angle-vector-raw (send *spot-kinova* :head :move-end-pos #f(-170 0 0)))
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector-raw (send *spot-kinova* :angle-vector #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 -70.075 -17.0383 51.0735 -90.482 -108.916 -0.540145)) 10000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (send *spot-kinova* :reset-pose) 5000)
  (send *ri* :wait-interpolation)
  ;;(unix::sleep 1)
  (send *ri* :stop-grasp)
  )

;; kinova-rest-poseの候補
(defun rest-pose ()
  ;;;;(send *ri* :angle-vector (send *spot-kinova* :angle-vector #f(40 60 149.9 0 40 0)) 7000)
  ;;;;(send *ri* :angle-vector (send *spot-kinova* :angle-vector #f(40 60 149.9 0 -30 0)) 7000)
  ;;(send *ri* :angle-vector (send *spot-kinova* :reset-pose) 7000)
  ;;(send *ri* :wait-interpolation)
  ;;;(send *ri* :angle-vector (send *spot-kinova* :angle-vector #f(90 140 149.9 -90 -55 90)) 7000) ;; good position but may collide with spot legs
  (send *ri* :angle-vector (send *spot-kinova* :angle-vector #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0 -149.9 -150.0 -90 55 90)) 7000) ;; this is also good position
  )

;;(send *ri* :angle-vector-raw (send *spot-kinova* :angle-vector #f(-140.075 -50 -30 -30 90 0)) 5000)
;;(send *ri* :angle-vector (send *spot-kinova* :angle-vector #f(16 50 -30 20 -90 60)))

(defun stair-pose ()
  ;; assume that the arm is in reset-pose
  (send *ri* :start-grasp)
  (send *ri* :angle-vector (send *spot-kinova* :angle-vector #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 30 65 -20 0 0 0)))
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (send *spot-kinova* :angle-vector #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 13 65 -20 0 -70 0)))
  (send *ri* :wait-interpolation)
  )

(defun stair-pose-reset ()
  (send *ri* :angle-vector (send *spot-kinova* :angle-vector #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 30 65 -20 0 0 0)))
  (send *ri* :wait-interpolation)
  (rest-pose))
