;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::TaskPath)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'TaskPath (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::TASKPATH")
  (make-package "CUSTOM_MSGS::TASKPATH"))

(in-package "ROS")
;;//! \htmlinclude TaskPath.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass custom_msgs::TaskPath
  :super ros::object
  :slots (_robotName _currentWorkflowStep _path ))

(defmethod custom_msgs::TaskPath
  (:init
   (&key
    ((:robotName __robotName) "")
    ((:currentWorkflowStep __currentWorkflowStep) "")
    ((:path __path) ())
    )
   (send-super :init)
   (setq _robotName (string __robotName))
   (setq _currentWorkflowStep (string __currentWorkflowStep))
   (setq _path __path)
   self)
  (:robotName
   (&optional __robotName)
   (if __robotName (setq _robotName __robotName)) _robotName)
  (:currentWorkflowStep
   (&optional __currentWorkflowStep)
   (if __currentWorkflowStep (setq _currentWorkflowStep __currentWorkflowStep)) _currentWorkflowStep)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:serialization-length
   ()
   (+
    ;; string _robotName
    4 (length _robotName)
    ;; string _currentWorkflowStep
    4 (length _currentWorkflowStep)
    ;; geometry_msgs/Point[] _path
    (apply #'+ (send-all _path :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _robotName
       (write-long (length _robotName) s) (princ _robotName s)
     ;; string _currentWorkflowStep
       (write-long (length _currentWorkflowStep) s) (princ _currentWorkflowStep s)
     ;; geometry_msgs/Point[] _path
     (write-long (length _path) s)
     (dolist (elem _path)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _robotName
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robotName (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _currentWorkflowStep
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _currentWorkflowStep (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Point[] _path
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _path (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _path)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get custom_msgs::TaskPath :md5sum-) "970b44acb4bfcd0885f6e14494a46276")
(setf (get custom_msgs::TaskPath :datatype-) "custom_msgs/TaskPath")
(setf (get custom_msgs::TaskPath :definition-)
      "string robotName
string currentWorkflowStep
geometry_msgs/Point[] path

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :custom_msgs/TaskPath "970b44acb4bfcd0885f6e14494a46276")


