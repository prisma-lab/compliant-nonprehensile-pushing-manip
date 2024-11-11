xhost +local:root

docker run -it --rm --privileged --name=pushing_raisim_container --net=host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume ./pushing_reduced/:/home/user/raisimLib/pushing_reduced/ --volume ./dependencies/raisimLib/rsc/:/home/user/raisimLib/rsc/ pushing_raisim
