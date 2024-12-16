; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude TaskPath.msg.html

(cl:defclass <TaskPath> (roslisp-msg-protocol:ros-message)
  ((robotName
    :reader robotName
    :initarg :robotName
    :type cl:string
    :initform "")
   (currentWorkflowStep
    :reader currentWorkflowStep
    :initarg :currentWorkflowStep
    :type cl:string
    :initform "")
   (path
    :reader path
    :initarg :path
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass TaskPath (<TaskPath>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TaskPath>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TaskPath)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<TaskPath> is deprecated: use custom_msgs-msg:TaskPath instead.")))

(cl:ensure-generic-function 'robotName-val :lambda-list '(m))
(cl:defmethod robotName-val ((m <TaskPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:robotName-val is deprecated.  Use custom_msgs-msg:robotName instead.")
  (robotName m))

(cl:ensure-generic-function 'currentWorkflowStep-val :lambda-list '(m))
(cl:defmethod currentWorkflowStep-val ((m <TaskPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:currentWorkflowStep-val is deprecated.  Use custom_msgs-msg:currentWorkflowStep instead.")
  (currentWorkflowStep m))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <TaskPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:path-val is deprecated.  Use custom_msgs-msg:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TaskPath>) ostream)
  "Serializes a message object of type '<TaskPath>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robotName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robotName))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'currentWorkflowStep))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'currentWorkflowStep))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TaskPath>) istream)
  "Deserializes a message object of type '<TaskPath>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robotName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'robotName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'currentWorkflowStep) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'currentWorkflowStep) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'path) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'path)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TaskPath>)))
  "Returns string type for a message object of type '<TaskPath>"
  "custom_msgs/TaskPath")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TaskPath)))
  "Returns string type for a message object of type 'TaskPath"
  "custom_msgs/TaskPath")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TaskPath>)))
  "Returns md5sum for a message object of type '<TaskPath>"
  "970b44acb4bfcd0885f6e14494a46276")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TaskPath)))
  "Returns md5sum for a message object of type 'TaskPath"
  "970b44acb4bfcd0885f6e14494a46276")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TaskPath>)))
  "Returns full string definition for message of type '<TaskPath>"
  (cl:format cl:nil "string robotName~%string currentWorkflowStep~%geometry_msgs/Point[] path~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TaskPath)))
  "Returns full string definition for message of type 'TaskPath"
  (cl:format cl:nil "string robotName~%string currentWorkflowStep~%geometry_msgs/Point[] path~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TaskPath>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'robotName))
     4 (cl:length (cl:slot-value msg 'currentWorkflowStep))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TaskPath>))
  "Converts a ROS message object to a list"
  (cl:list 'TaskPath
    (cl:cons ':robotName (robotName msg))
    (cl:cons ':currentWorkflowStep (currentWorkflowStep msg))
    (cl:cons ':path (path msg))
))
