; Auto-generated. Do not edit!


(cl:in-package bobert_control-msg)


;//! \htmlinclude armCmd.msg.html

(cl:defclass <armCmd> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass armCmd (<armCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <armCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'armCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bobert_control-msg:<armCmd> is deprecated: use bobert_control-msg:armCmd instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <armCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bobert_control-msg:angle-val is deprecated.  Use bobert_control-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <armCmd>) ostream)
  "Serializes a message object of type '<armCmd>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'angle))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <armCmd>) istream)
  "Deserializes a message object of type '<armCmd>"
  (cl:setf (cl:slot-value msg 'angle) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'angle)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<armCmd>)))
  "Returns string type for a message object of type '<armCmd>"
  "bobert_control/armCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'armCmd)))
  "Returns string type for a message object of type 'armCmd"
  "bobert_control/armCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<armCmd>)))
  "Returns md5sum for a message object of type '<armCmd>"
  "d5646d2d9986672237331f3ea363f45f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'armCmd)))
  "Returns md5sum for a message object of type 'armCmd"
  "d5646d2d9986672237331f3ea363f45f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<armCmd>)))
  "Returns full string definition for message of type '<armCmd>"
  (cl:format cl:nil "float32[6] angle # degrees~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'armCmd)))
  "Returns full string definition for message of type 'armCmd"
  (cl:format cl:nil "float32[6] angle # degrees~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <armCmd>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'angle) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <armCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'armCmd
    (cl:cons ':angle (angle msg))
))
