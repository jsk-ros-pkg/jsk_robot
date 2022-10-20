(defmacro def-state-func (name args &rest body)
  `(defun ,name ,args
     (set-alist 'description "" ,(car args))
     (set-alist 'image "" ,(car args))
     ,@body))
