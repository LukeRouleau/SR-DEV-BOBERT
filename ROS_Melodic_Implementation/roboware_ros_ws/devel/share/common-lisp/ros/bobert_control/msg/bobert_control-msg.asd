
(cl:in-package :asdf)

(defsystem "bobert_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "armCmd" :depends-on ("_package_armCmd"))
    (:file "_package_armCmd" :depends-on ("_package"))
    (:file "bobertTelemetry" :depends-on ("_package_bobertTelemetry"))
    (:file "_package_bobertTelemetry" :depends-on ("_package"))
  ))