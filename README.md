# yolov8_trt_ros2
Yolov8 Tensorrt with ros2 support. Tested with Jetson Orion Nano

1. The repository is based on https://github.com/wang-xinyu/tensorrtx/tree/master/yolov8 with some modification so it can work with live camera feed and ros 2. First build and convert weights as stated in the official resposity.
```
git clone https://github.com/ultralytics/ultralytics.git
cp {yolov8_txt_ros2}/yolov8/gen_wts.py {ultralytics}/ultralytics
cd {ultralytics}/ultralytics
//copy weights to this folder from official ultralytics repository
python gen_wts.py -w yolov8n.pt -o yolov8n.wts -t detect
// a file 'yolov8n.wts' will be generated.
```
2. build yolov8_trt_ros2/yolov8 and run
```
cd {yolov8_trt_ros2}
// update kNumClass in config.h if your model is trained on custom dataset
mkdir build
cd build
cp {ultralytics}/ultralytics/yolov8.wts {yolov8_trt_ros2}/yolov8/build
cmake ..
make
sudo ./yolov8_det -s [.wts] [.engine] [n/s/m/l/x/n6/s6/m6/l6/x6]  // serialize model to plan file
sudo ./yolov8_det -d [.engine] [image folder]  [c/g] // deserialize and run inference, the images in [image folder] will be processed.
// For example yolov8
sudo ./yolov8_det -s yolov8n.wts yolov8.engine n
sudo ./yolov8_det -d yolov8n.engine ../images c //cpu postprocess
sudo ./yolov8_det -d yolov8n.engine ../images g //gpu postprocess
```
4. Camera and ROS 2 (humble) based inference.
```
// Camera
python3 infer.py
//ROS 2
python3 infer_ros.py
  // Make sure to change weights and the camera topic.
```
