#!/bin/bash

#rosrun image_view video_recorder image:=/duplex_h/image_raw _filename:=out.avi &> video_rec.log
#rosrun image_view video_recorder image:=/duplex_h/image_raw _filename:=out_h.avi _codec:=XVID _fps:=2 &> video_rec.log
#rosrun image_view video_recorder image:=/duplex/image_raw _filename:=out.avi _codec:=XVID &> video_rec.log

rosrun image_view video_recorder image:=/image_raw _filename:=out.avi _codec:=XVID _fps:=2 &> video_rec.log
