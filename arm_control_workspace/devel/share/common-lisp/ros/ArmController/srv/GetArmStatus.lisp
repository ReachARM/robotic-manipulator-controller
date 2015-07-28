; Auto-generated. Do not edit!


(cl:in-package ArmController-srv)


;//! \htmlinclude GetArmStatus-request.msg.html

(cl:defclass <GetArmStatus-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetArmStatus-request (<GetArmStatus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetArmStatus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetArmStatus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ArmController-srv:<GetArmStatus-request> is deprecated: use ArmController-srv:GetArmStatus-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetArmStatus-request>) ostream)
  "Serializes a message object of type '<GetArmStatus-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetArmStatus-request>) istream)
  "Deserializes a message object of type '<GetArmStatus-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetArmStatus-request>)))
  "Returns string type for a service object of type '<GetArmStatus-request>"
  "ArmController/GetArmStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetArmStatus-request)))
  "Returns string type for a service object of type 'GetArmStatus-request"
  "ArmController/GetArmStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetArmStatus-request>)))
  "Returns md5sum for a message object of type '<GetArmStatus-request>"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetArmStatus-request)))
  "Returns md5sum for a message object of type 'GetArmStatus-request"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetArmStatus-request>)))
  "Returns full string definition for message of type '<GetArmStatus-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetArmStatus-request)))
  "Returns full string definition for message of type 'GetArmStatus-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetArmStatus-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetArmStatus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetArmStatus-request
))
;//! \htmlinclude GetArmStatus-response.msg.html

(cl:defclass <GetArmStatus-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetArmStatus-response (<GetArmStatus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetArmStatus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetArmStatus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ArmController-srv:<GetArmStatus-response> is deprecated: use ArmController-srv:GetArmStatus-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <GetArmStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ArmController-srv:status-val is deprecated.  Use ArmController-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetArmStatus-response>) ostream)
  "Serializes a message object of type '<GetArmStatus-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetArmStatus-response>) istream)
  "Deserializes a message object of type '<GetArmStatus-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetArmStatus-response>)))
  "Returns string type for a service object of type '<GetArmStatus-response>"
  "ArmController/GetArmStatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetArmStatus-response)))
  "Returns string type for a service object of type 'GetArmStatus-response"
  "ArmController/GetArmStatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetArmStatus-response>)))
  "Returns md5sum for a message object of type '<GetArmStatus-response>"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetArmStatus-response)))
  "Returns md5sum for a message object of type 'GetArmStatus-response"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetArmStatus-response>)))
  "Returns full string definition for message of type '<GetArmStatus-response>"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetArmStatus-response)))
  "Returns full string definition for message of type 'GetArmStatus-response"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetArmStatus-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetArmStatus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetArmStatus-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetArmStatus)))
  'GetArmStatus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetArmStatus)))
  'GetArmStatus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetArmStatus)))
  "Returns string type for a service object of type '<GetArmStatus>"
  "ArmController/GetArmStatus")