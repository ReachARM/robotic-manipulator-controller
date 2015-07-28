; Auto-generated. Do not edit!


(cl:in-package ArmController-srv)


;//! \htmlinclude MoveRelativeTool-request.msg.html

(cl:defclass <MoveRelativeTool-request> (roslisp-msg-protocol:ros-message)
  ((tx
    :reader tx
    :initarg :tx
    :type cl:integer
    :initform 0)
   (ty
    :reader ty
    :initarg :ty
    :type cl:integer
    :initform 0)
   (tz
    :reader tz
    :initarg :tz
    :type cl:integer
    :initform 0)
   (roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass MoveRelativeTool-request (<MoveRelativeTool-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveRelativeTool-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveRelativeTool-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ArmController-srv:<MoveRelativeTool-request> is deprecated: use ArmController-srv:MoveRelativeTool-request instead.")))

(cl:ensure-generic-function 'tx-val :lambda-list '(m))
(cl:defmethod tx-val ((m <MoveRelativeTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ArmController-srv:tx-val is deprecated.  Use ArmController-srv:tx instead.")
  (tx m))

(cl:ensure-generic-function 'ty-val :lambda-list '(m))
(cl:defmethod ty-val ((m <MoveRelativeTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ArmController-srv:ty-val is deprecated.  Use ArmController-srv:ty instead.")
  (ty m))

(cl:ensure-generic-function 'tz-val :lambda-list '(m))
(cl:defmethod tz-val ((m <MoveRelativeTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ArmController-srv:tz-val is deprecated.  Use ArmController-srv:tz instead.")
  (tz m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <MoveRelativeTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ArmController-srv:roll-val is deprecated.  Use ArmController-srv:roll instead.")
  (roll m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <MoveRelativeTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ArmController-srv:pitch-val is deprecated.  Use ArmController-srv:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <MoveRelativeTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ArmController-srv:yaw-val is deprecated.  Use ArmController-srv:yaw instead.")
  (yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveRelativeTool-request>) ostream)
  "Serializes a message object of type '<MoveRelativeTool-request>"
  (cl:let* ((signed (cl:slot-value msg 'tx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ty)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'tz)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveRelativeTool-request>) istream)
  "Deserializes a message object of type '<MoveRelativeTool-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tx) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ty) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tz) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveRelativeTool-request>)))
  "Returns string type for a service object of type '<MoveRelativeTool-request>"
  "ArmController/MoveRelativeToolRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveRelativeTool-request)))
  "Returns string type for a service object of type 'MoveRelativeTool-request"
  "ArmController/MoveRelativeToolRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveRelativeTool-request>)))
  "Returns md5sum for a message object of type '<MoveRelativeTool-request>"
  "cebd33aacfc803188fa479cfc00240bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveRelativeTool-request)))
  "Returns md5sum for a message object of type 'MoveRelativeTool-request"
  "cebd33aacfc803188fa479cfc00240bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveRelativeTool-request>)))
  "Returns full string definition for message of type '<MoveRelativeTool-request>"
  (cl:format cl:nil "int32 tx~%int32 ty~%int32 tz~%float32 roll~%float32 pitch~%float32 yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveRelativeTool-request)))
  "Returns full string definition for message of type 'MoveRelativeTool-request"
  (cl:format cl:nil "int32 tx~%int32 ty~%int32 tz~%float32 roll~%float32 pitch~%float32 yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveRelativeTool-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveRelativeTool-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveRelativeTool-request
    (cl:cons ':tx (tx msg))
    (cl:cons ':ty (ty msg))
    (cl:cons ':tz (tz msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
))
;//! \htmlinclude MoveRelativeTool-response.msg.html

(cl:defclass <MoveRelativeTool-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MoveRelativeTool-response (<MoveRelativeTool-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveRelativeTool-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveRelativeTool-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ArmController-srv:<MoveRelativeTool-response> is deprecated: use ArmController-srv:MoveRelativeTool-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveRelativeTool-response>) ostream)
  "Serializes a message object of type '<MoveRelativeTool-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveRelativeTool-response>) istream)
  "Deserializes a message object of type '<MoveRelativeTool-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveRelativeTool-response>)))
  "Returns string type for a service object of type '<MoveRelativeTool-response>"
  "ArmController/MoveRelativeToolResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveRelativeTool-response)))
  "Returns string type for a service object of type 'MoveRelativeTool-response"
  "ArmController/MoveRelativeToolResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveRelativeTool-response>)))
  "Returns md5sum for a message object of type '<MoveRelativeTool-response>"
  "cebd33aacfc803188fa479cfc00240bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveRelativeTool-response)))
  "Returns md5sum for a message object of type 'MoveRelativeTool-response"
  "cebd33aacfc803188fa479cfc00240bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveRelativeTool-response>)))
  "Returns full string definition for message of type '<MoveRelativeTool-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveRelativeTool-response)))
  "Returns full string definition for message of type 'MoveRelativeTool-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveRelativeTool-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveRelativeTool-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveRelativeTool-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveRelativeTool)))
  'MoveRelativeTool-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveRelativeTool)))
  'MoveRelativeTool-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveRelativeTool)))
  "Returns string type for a service object of type '<MoveRelativeTool>"
  "ArmController/MoveRelativeTool")