(require :unittest "lib/llib/unittest.l")
(require :smach-utils "package://jsk_robot_startup/euslisp/smach-utils.l")


(defmacro def-state-func-test (func-name result)
  `(deftest ,(intern (format nil "~A-test" func-name)) ()
     (let* ((func '(,func-name '((description . "hoge") (image . "fuga"))))
            (output (eval func)))
       (assert (equal output ,result)
               (format nil "~%func: ~A error, output: ~A~%" ,(format nil "~A" func-name) output)))))


(def-state-func state-func (userdata) userdata)
(def-state-func state-func-rest (&rest userdata) (car userdata))
(def-state-func state-func-optional (&optional userdata) userdata)
(def-state-func state-func-optional-default (&optional (userdata nil)) userdata)


(init-unit-test)


(def-state-func-test state-func '((description . "") (image . "")))
(def-state-func-test state-func-rest '((description . "") (image . "")))
(def-state-func-test state-func-optional '((description . "") (image . "")))
(def-state-func-test state-func-optional-default '((description . "") (image . "")))


(run-all-tests)
(exit)
