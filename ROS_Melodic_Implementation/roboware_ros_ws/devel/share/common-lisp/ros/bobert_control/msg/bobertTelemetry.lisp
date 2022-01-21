; Auto-generated. Do not edit!


(cl:in-package bobert_control-msg)


;//! \htmlinclude bobertTelemetry.msg.html

(cl:defclass <bobertTelemetry> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass bobertTelemetry (<bobertTelemetry>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <bobertTelemetry>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'bobertTelemetry)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bobert_control-msg:<bobertTelemetry> is deprecated: use bobert_control-msg:bobertTelemetry instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <bobertTelemetry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bobert_control-msg:angle-val is deprecated.  Use bobert_control-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <bobertTelemetry>) ostream)
  "Serializes a message object of type '<bobertTelemetry>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'angle))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <bobertTelemetry>) istream)
  "Deserializes a message object of type '<bobertTelemetry>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<bobertTelemetry>)))
  "Returns string type for a message object of type '<bobertTelemetry>"
  "bobert_control/bobertTelemetry")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bobertTelemetry)))
  "Returns string type for a message object of type 'bobertTelemetry"
  "bobert_control/bobertTelemetry")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<bobertTelemetry>)))
  "Returns md5sum for a message object of type '<bobertTelemetry>"
  "d5646d2d9986672237331f3ea363f45f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'bobertTelemetry)))
  "Returns md5sum for a message object of type 'bobertTelemetry"
  "d5646d2d9986672237331f3ea363f45f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<bobertTelemetry>)))
  "Returns full string definition for message of type '<bobertTelemetry>"
  (cl:format cl:nil "float32[6] angle # degrees~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'bobertTelemetry)))
  "Returns full string definition for message of type 'bobertTelemetry"
  (cl:format cl:nil "float32[6] angle # degrees~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <bobertTelemetry>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'angle) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <bobertTelemetry>))
  "Converts a ROS message object to a list"
  (cl:list 'bobertTelemetry
    (cl:cons ':angle (angle msg))
))
