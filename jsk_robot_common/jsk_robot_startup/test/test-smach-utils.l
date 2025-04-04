(require :unittest "lib/llib/unittest.l")
(require :smach-utils "package://jsk_robot_startup/euslisp/smach-utils.l")


(defmacro def-state-func-test (func-name)
  `(deftest ,(intern (string-upcase (format nil "~A-test" func-name))) ()
     ,@(mapcar #'(lambda (out expected)
                   `(let ((output (,func-name ',out)))
                      (assert (equal output ',expected)
                              (format nil ,(format nil "~~%func: ~A error, output: ~~S~~%" func-name) output))))
               (list '((description . "hoge") (image . "fuga"))
                     '((description . "hoge") (image . "fuga") (other . "value"))
                     '((description . "hoge"))
                     '((image . "fuga"))
                     nil)
               (list '((description . "") (image . ""))
                     '((description . "") (image . "") (other . "value"))
                     '((description . ""))
                     '((image . ""))
                     nil))))


(def-state-func state-func (userdata) userdata)
(def-state-func state-func-rest (&rest userdata) (car userdata))
(def-state-func state-func-optional (&optional userdata) userdata)
(def-state-func state-func-optional-default (&optional (userdata nil)) userdata)


(init-unit-test)


(def-state-func-test state-func)
(def-state-func-test state-func-rest)
(def-state-func-test state-func-optional)
(def-state-func-test state-func-optional-default)


(run-all-tests)
(exit)
