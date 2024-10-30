# Install pybind11
```
sudo apt -y install python3-pybind11 # ubnntu 20
# sudo apt -y install python-pybind11 # ubnntu 18
```

# build TimestampCorrector
```
mkdir build && cd build
cmake ..
make
```

# test TimestampCorrector
```
python3 test.py
```

