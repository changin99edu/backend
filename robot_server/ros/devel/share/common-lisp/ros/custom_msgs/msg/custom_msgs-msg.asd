
(cl:in-package :asdf)

(defsystem "custom_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "TaskPath" :depends-on ("_package_TaskPath"))
    (:file "_package_TaskPath" :depends-on ("_package"))
  ))