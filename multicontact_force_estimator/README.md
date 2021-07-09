## install

```
mkdir -p catkin_ws/src
cd catkin_ws/src
wstool init .
wstool merge [このディレクトリの.rosintall]
cd ..
rosdep install -r --from-paths src --ignore-src -y
catkin build muticontact_force_estimator
```