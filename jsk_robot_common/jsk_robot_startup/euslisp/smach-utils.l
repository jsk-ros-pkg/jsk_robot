(defmacro def-state-func (name args &rest body)
  (let ((userdata
          (cond ((eq (car args) '&optional)
                 (cond
                   ((listp (cadr args))
                    (if (listp (caadr args))
                      (cadr (caadr args))
                      (caadr args)))
                   (t (cadr args))))
                ((eq (car args) '&rest) `(car ,(cadr args)))
                (t (car args)))))
    `(defun ,name ,args
       (setf (cdr (assoc 'description ,userdata)) "")
       (setf (cdr (assoc 'image ,userdata)) "")
       ,@body)))
