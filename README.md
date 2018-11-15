DSAAM : A time management library for simulation middlewares
======================================================================

This a proof of concept for a proposed time management scheme for building
decentralized simulations.

Contrary to HLA or DIS, the time management scheme is completely decentralized.

The library is implemented in C++ and Python for the ROS middleware, as well as
for intra-process communication between threads.

Requirements
------------

### C++
A C++14 compatible compiler

### Python
Python>=3.5

For the tests:
* numpy
* matplotlib

### ROS
ROS>=kinetic with rospy and roscpp packages

Installing
---------
### C++
`mkdir build && cd build && cmake .. && make install`

### Python
Use the provided setup.py script, for help see:
`python3 setup.py install --help`

Tests and examples
-------

A synthetic example has been implemented in both python and C++, for the
bare intra-process communication method as well as for the ROS middleware

### Intra-process communication between threads
#### Python
To run the example (with graphical output):
`python3 test/python/test_nbody.py`

It can also be run as a test using noesetests3:
`PYTHON3_INTERPRETER=python3 nosetests3`

#### C++
Building the example:
`mkdir build && cd build && cmake .. -DBUILD_TEST=ON && make build_tests`

Running it (no graphical output):
`./nbody_threads`

### ROS
To build the ROS workspace:
`mkdir build && cd build && cmake .. -DBUILD_TEST=ON -DBUILD_ROS && make build_tests`

To run the example (from the project root folder):
`source build/ros/devel/setup.bash && test/ros/src/dsaam_nbody/scripts/test_nbody.py`

The ROS example mixes python and C++ nodes.
