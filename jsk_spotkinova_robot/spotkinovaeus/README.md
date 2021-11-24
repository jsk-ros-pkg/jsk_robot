spotkinovaeus
=============

# Setup
Create workspace which includes `spoteus` and `kinovaeus`.
Make sure that each euslisp model is created with `spot_description` and `kortex_description`.

# Usage
With IRT Viewer
```
roseus spotkinova-util.l
(spot-kinova :type :gen3_lite_gen3_lite_2f)
(objects (list *spot-kinova*))
(send *spot-kinova* :init-pose)
(send *spot-kinova* :head :inverse-kinematics (make-coords :pos #f(700 0 500)) :rotation-axis nil)
```

With real robot
```

```
