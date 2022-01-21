;; Auto-generated. Do not edit!


(when (boundp 'bobert_control::bobertTelemetry)
  (if (not (find-package "BOBERT_CONTROL"))
    (make-package "BOBERT_CONTROL"))
  (shadow 'bobertTelemetry (find-package "BOBERT_CONTROL")))
(unless (find-package "BOBERT_CONTROL::BOBERTTELEMETRY")
  (make-package "BOBERT_CONTROL::BOBERTTELEMETRY"))

(in-package "ROS")
;;//! \htmlinclude bobertTelemetry.msg.html


(defclass bobert_control::bobertTelemetry
  :super ros::object
  :slots (_angle ))

(defmethod bobert_control::bobertTelemetry
  (:init
   (&key
    ((:angle __angle) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _angle __angle)
   self)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float32[6] _angle
    (* 4    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[6] _angle
     (dotimes (i 6)
       (sys::poke (elt _angle i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[6] _angle
   (dotimes (i (length _angle))
     (setf (elt _angle i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get bobert_control::bobertTelemetry :md5sum-) "d5646d2d9986672237331f3ea363f45f")
(setf (get bobert_control::bobertTelemetry :datatype-) "bobert_control/bobertTelemetry")
(setf (get bobert_control::bobertTelemetry :definition-)
      "float32[6] angle # degrees
")



(provide :bobert_control/bobertTelemetry "d5646d2d9986672237331f3ea363f45f")


