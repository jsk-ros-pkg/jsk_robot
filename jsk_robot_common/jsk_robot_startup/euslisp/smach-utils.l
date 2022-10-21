(defmacro def-state-func (name args &rest body)
  (let ((userdata (cond ((eq (car args) '&optional) (cadr args))
                      ((eq (car args) '&rest) (cadr args))
                      (t (car args)))))
    `(defun ,name ,args
       (set-alist 'description "" ,userdata)
       (set-alist 'image "" ,userdata)
       ,@body)))
