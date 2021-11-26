spotkinovaeus
=============

# Setup
Create workspace which includes `spoteus` and `kinovaeus`.
Make sure that each euslisp model is created with `spot_description` and `kortex_description`.

# Usage
With IRT Viewer
```
roseus spotkinova-utils.l
(spotkinova :type :gen3_lite_gen3_lite_2f)
(objects (list *spotkinova*))
(send *spotkinova* :init-pose)
(send *spotkinova* :head :inverse-kinematics (make-coords :pos #f(700 0 500)) :rotation-axis nil)
```

With real robot
```

```
