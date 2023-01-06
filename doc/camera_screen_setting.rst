camera screen setting
==================
install v4l
::
  $ sudo apt install v4l-utils

flip: vertical
::
  $ v4l2-ctl -c vertical_flip=1
  $ in_fmt=`media-ctl --get-v4l2 '"rkisp-csi-subdev":0'`
  $ media-ctl --set-v4l2 "'rkisp-isp-subdev':0$in_fmt"

flip: horizontal
::
  $ v4l2-ctl -c horizontal_flip=1
  $ in_fmt=`media-ctl --get-v4l2 '"rkisp-csi-subdev":0'`
  $ media-ctl --set-v4l2 "'rkisp-isp-subdev':0$in_fmt"
