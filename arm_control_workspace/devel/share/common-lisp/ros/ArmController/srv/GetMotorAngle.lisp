; Auto-generated. Do not edit!


(cl:in-package ArmController-srv)


;//! \htmlinclude GetMotorAngle-request.msg.html

(cl:defclass <GetMotorAngle-request> (roslisp-msg-protocol:ros-message)
  ((motorID
    :reader motorID
    :initarg :motorID
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetMotorAngle-request (<GetMotorAngle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMotorAngle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMotorAngle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ArmController-srv:<GetMotorAngle-request> is deprecated: use ArmController-srv:GetMotorAngle-request instead.")))

(cl:ensure-generic-function 'motorID-val :lambda-list '(m))
(cl:defmethod motorID-val ((m <GetMotorAngle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ArmController-srv:motorID-val is deprecated.  Use ArmController-srv:motorID instead.")
  (motorID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMotorAngle-request>) ostream)
  "Serializes a message object of type '<GetMotorAngle-request>"
  (cl:let* ((signed (cl:slot-value msg 'motorID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMotorAngle-request>) istream)
  "Deserializes a message object of type '<GetMotorAngle-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motorID) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMotorAngle-request>)))
  "Returns string type for a service object of type '<GetMotorAngle-request>"
  "ArmController/GetMotorAngleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMotorAngle-request)))
  "Returns string type for a service object of type 'GetMotorAngle-request"
  "ArmController/GetMotorAngleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMotorAngle-request>)))
  "Returns md5sum for a message object of type '<GetMotorAngle-request>"
  "fb28cd0b4dec57104328493f86235b1b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMotorAngle-request)))
  "Returns md5sum for a message object of type 'GetMotorAngle-request"
  "fb28cd0b4dec57104328493f86235b1b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMotorAngle-request>)))
  "Returns full string definition for message of type '<GetMotorAngle-request>"
  (cl:format cl:nil "int8 motorID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMotorAngle-request)))
  "Returns full string definition for message of type 'GetMotorAngle-request"
  (cl:format cl:nil "int8 motorID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMotorAngle-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMotorAngle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMotorAngle-request
    (cl:cons ':motorID (motorID msg))
))
;//! \htmlinclude GetMotorAngle-response.msg.html

(cl:defclass <GetMotorAngle-response> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetMotorAngle-response (<GetMotorAngle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMotorAngle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMotorAngle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ArmController-srv:<GetMotorAngle-response> is deprecated: use ArmController-srv:GetMotorAngle-response instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <GetMotorAngle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ArmController-srv:angle-val is deprecated.  Use ArmController-srv:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMotorAngle-response>) ostream)
  "Serializes a message object of type '<GetMotorAngle-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMotorAngle-response>) istream)
  "Deserializes a message object of type '<GetMotorAngle-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMotorAngle-response>)))
  "Returns string type for a service object of type '<GetMotorAngle-response>"
  "ArmController/GetMotorAngleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMotorAngle-response)))
  "Returns string type for a service object of type 'GetMotorAngle-response"
  "ArmController/GetMotorAngleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMotorAngle-response>)))
  "Returns md5sum for a message object of type '<GetMotorAngle-response>"
  "fb28cd0b4dec57104328493f86235b1b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMotorAngle-response)))
  "Returns md5sum for a message object of type 'GetMotorAngle-response"
  "fb28cd0b4dec57104328493f86235b1b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMotorAngle-response>)))
  "Returns full string definition for message of type '<GetMotorAngle-response>"
  (cl:format cl:nil "float32 angle~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMotorAngle-response)))
  "Returns full string definition for message of type 'GetMotorAngle-response"
  (cl:format cl:nil "float32 angle~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMotorAngle-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMotorAngle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMotorAngle-response
    (cl:cons ':angle (angle msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetMotorAngle)))
  'GetMotorAngle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetMotorAngle)))
  'GetMotorAngle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMotorAngle)))
  "Returns string type for a service object of type '<GetMotorAngle>"
  "ArmController/GetMotorAngle")