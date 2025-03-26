#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/asl-ss-guna/catkin_ws/src/robotiq/robotiq_control"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/asl-ss-guna/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/asl-ss-guna/catkin_ws/install/lib/python3/dist-packages:/home/asl-ss-guna/catkin_ws/build/robotiq_control/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/asl-ss-guna/catkin_ws/build/robotiq_control" \
    "/usr/bin/python3" \
    "/home/asl-ss-guna/catkin_ws/src/robotiq/robotiq_control/setup.py" \
     \
    build --build-base "/home/asl-ss-guna/catkin_ws/build/robotiq_control" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/asl-ss-guna/catkin_ws/install" --install-scripts="/home/asl-ss-guna/catkin_ws/install/bin"
