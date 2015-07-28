; Auto-generated. Do not edit!


(cl:in-package ArmController-srv)


;//! \htmlinclude MoveAbsoluteMotor-request.msg.html

(cl:defclass <MoveAbsoluteMotor-request> (roslisp-msg-protocol:ros-message)
  ((motorID
    :reader motorID
    :initarg :motorID
    :type cl:fixnum
    :initform 0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass MoveAbsoluteMotor-request (<MoveAbsoluteMotor-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveAbsoluteMotor-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveAbsoluteMotor-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ArmController-srv:<MoveAbsoluteMotor-request> is deprecated: use ArmController-srv:MoveAbsoluteMotor-request instead.")))

(cl:ensure-generic-function 'motorID-val :lambda-list '(m))
(cl:defmethod motorID-val ((m <MoveAbsoluteMotor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ArmController-srv:motorID-val is deprecated.  Use ArmController-srv:motorID instead.")
  (motorID m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <MoveAbsoluteMotor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ArmController-srv:angle-val is deprecated.  Use ArmController-srv:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveAbsoluteMotor-request>) ostream)
  "Serializes a message object of type '<MoveAbsoluteMotor-request>"
  (cl:let* ((signed (cl:slot-value msg 'motorID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveAbsoluteMotor-request>) istream)
  "Deserializes a message object of type '<MoveAbsoluteMotor-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motorID) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveAbsoluteMotor-request>)))
  "Returns string type for a service object of type '<MoveAbsoluteMotor-request>"
  "ArmController/MoveAbsoluteMotorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveAbsoluteMotor-request)))
  "Returns string type for a service object of type 'MoveAbsoluteMotor-request"
  "ArmController/MoveAbsoluteMotorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveAbsoluteMotor-request>)))
  "Returns md5sum for a message object of type '<MoveAbsoluteMotor-request>"
  "52ffe69d55b4b8197d25cc69b97f4c61")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveAbsoluteMotor-request)))
  "Returns md5sum for a message object of type 'MoveAbsoluteMotor-request"
  "52ffe69d55b4b8197d25cc69b97f4c61")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveAbsoluteMotor-request>)))
  "Returns full string definition for message of type '<MoveAbsoluteMotor-request>"
  (cl:format cl:nil "int8 motorID~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveAbsoluteMotor-request)))
  "Returns full string definition for message of type 'MoveAbsoluteMotor-request"
  (cl:format cl:nil "int8 motorID~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveAbsoluteMotor-request>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveAbsoluteMotor-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveAbsoluteMotor-request
    (cl:cons ':motorID (motorID msg))
    (cl:cons ':angle (angle msg))
))
;//! \htmlinclude MoveAbsoluteMotor-response.msg.html

(cl:defclass <MoveAbsoluteMotor-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MoveAbsoluteMotor-response (<MoveAbsoluteMotor-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveAbsoluteMotor-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveAbsoluteMotor-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ArmController-srv:<MoveAbsoluteMotor-response> is deprecated: use ArmController-srv:MoveAbsoluteMotor-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveAbsoluteMotor-response>) ostream)
  "Serializes a message object of type '<MoveAbsoluteMotor-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveAbsoluteMotor-response>) istream)
  "Deserializes a message object of type '<MoveAbsoluteMotor-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveAbsoluteMotor-response>)))
  "Returns string type for a service object of type '<MoveAbsoluteMotor-response>"
  "ArmController/MoveAbsoluteMotorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveAbsoluteMotor-response)))
  "Returns string type for a service object of type 'MoveAbsoluteMotor-response"
  "ArmController/MoveAbsoluteMotorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveAbsoluteMotor-response>)))
  "Returns md5sum for a message object of type '<MoveAbsoluteMotor-response>"
  "52ffe69d55b4b8197d25cc69b97f4c61")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveAbsoluteMotor-response)))
  "Returns md5sum for a message object of type 'MoveAbsoluteMotor-response"
  "52ffe69d55b4b8197d25cc69b97f4c61")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveAbsoluteMotor-response>)))
  "Returns full string definition for message of type '<MoveAbsoluteMotor-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveAbsoluteMotor-response)))
  "Returns full string definition for message of type 'MoveAbsoluteMotor-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveAbsoluteMotor-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveAbsoluteMotor-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveAbsoluteMotor-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveAbsoluteMotor)))
  'MoveAbsoluteMotor-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveAbsoluteMotor)))
  'MoveAbsoluteMotor-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveAbsoluteMotor)))
  "Returns string type for a service object of type '<MoveAbsoluteMotor>"
  "ArmController/MoveAbsoluteMotor")