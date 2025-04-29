# Install pybind11
```
sudo apt -y install python3-pybind11 # ubuntu 20
sudo apt-get install python3-dev # ubuntu 20
sudo apt -y install python-pybind11 # ubnntu 18
```

# build TimestampCorrector
```
mkdir build && cd build
cmake ..
# cmake -DPYTHON_EXECUTABLE=$(which python3.8) .. # for systems with multiple python3
make
```

# test TimestampCorrector
```
python3 test.py

cd vio_common/python
python3 lidar/livox_phone_bagcreater.py
```

# Troubleshooting

1. /usr/include/pybind11/detail/common.h:112:10: fatal error: Python.h: No such file or directory
  112 | #include <Python.h>

```
include_directories(/usr/include/python3.8) in CMakeLists.txt
```

2. After make, File "./lidar/livox_phone_bagcreater.py", line 15, in <module>
    import TimestampCorrector as TC
ModuleNotFoundError: No module named 'TimestampCorrector'

Your /usr/bin may have multiple pythons like python3.8 and python3.9, specify the one you want to use,
```
cmake -DPYTHON_EXECUTABLE=$(which python3.8) ..
```
