
(cl:in-package :asdf)

(defsystem "ArmController-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetMotorAngle" :depends-on ("_package_GetMotorAngle"))
    (:file "_package_GetMotorAngle" :depends-on ("_package"))
    (:file "MoveRelativeTool" :depends-on ("_package_MoveRelativeTool"))
    (:file "_package_MoveRelativeTool" :depends-on ("_package"))
    (:file "GetArmStatus" :depends-on ("_package_GetArmStatus"))
    (:file "_package_GetArmStatus" :depends-on ("_package"))
    (:file "MoveAbsoluteMotor" :depends-on ("_package_MoveAbsoluteMotor"))
    (:file "_package_MoveAbsoluteMotor" :depends-on ("_package"))
  ))