vio_common

# Build instructions

## Install opencv 2.4 and eigen 3.3

```
#e.g., 
sudo apt-get install libopencv-dev libeigen3-dev
```

If you want to install a specific version of these libraries or install to a specific path (note -DCMAKE\_INSTALL\_PREFIX), 
you may build them from source.

## Build vio_common

### Without ROS
```
cd workspace
git clone https://github.com/JzHuai0108/vio_common.git vio_common
mkdir build && cd build
#cmake .. -DOpenCV_DIR=/folder/of/OpenCVConfig.cmake -DEIGEN_INCLUDE_DIR=/folder/of/Eigen -DCMAKE_INSTALL_PREFIX=$HOME/workspace/local_install/vio_common -DUSE_ROS=OFF
#e.g.,
cmake .. -DOpenCV_DIR=$HOME/workspace/local_install/opencv/share/OpenCV -DEIGEN_INCLUDE_DIR=$HOME/workspace/local_install/eigen -DCMAKE_INSTALL_PREFIX=$HOME/workspace/local_install/vio_common -DUSE_ROS=OFF
make
make install
```

### With ROS
```
catkin build vio_common -DCMAKE_BUILD_TYPE=Release -j4
```

## Build and run gtests

```
# Build tests
catkin build vio_common --catkin-make-args run_tests
# or
catkin build --make-args tests -- vio_common

# list tests
rosrun vio_common vio_common_test --gtest_list_tests
# run a test
rosrun vio_common vio_common_test --gtest_filter="*some_test*"
```

## which ROS?
To bypass the tight coupling of ros and ubuntu, we use the [robostack](https://robostack.github.io/GettingStarted.html) to manage ros envs.
This way, we can use noetic on ubuntu 22.


## References
[1] [Build and run tests with catkin tools](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html#building-and-running-tests). 
[2] [Run unit tests with catkin](https://personalrobotics.cs.washington.edu/software/unit-testing/)

