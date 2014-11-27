#!/bin/bash

NITE_DIR=$(rospack find cob_openni2_tracker)
#echo $NITE_DIR
ln -s $NITE_DIR/include/NiTE-Linux-x64-2.2/Redist/NiTE2/  ~/.ros/NiTE2
