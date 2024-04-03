# yolov8_trt_ros2
Yolov8 Tensorrt with ros2 support. Tested with Jetson Orion Nano Jetpack 5.1.2

1. Requirements and Environment Setup.
    * Jetpack 5.1.2
    * CUDA 11.4+
    * cuDNN 8.6+
    * TensorRT 8.5+
    * OpenCV 4.5+
    * Python 3.8
    * ROS 2 Foxy/Humble (Optional)

If all the above mentioned required are satisfied proceed with environment setup.
Download the repository at home directory.
```
git clone https://github.com/sulemank137/yolov8_trt_ros2.git
```
Copy the dl_setup.sh file to home directory, grant permission and execute to build a python environment with all the other dependencies. 
```
cp ~/yolov8_trt_ros2/dl_setup.sh /home/$USER
sudo chmod +x dl_setup.sh
./dl_setup.sh
```
Executing above script will create the python virtual environment named "dl_dep", activate the environment to proceed further.
```
source dl_dep/bin/activate
```
2. This repository is based on https://github.com/wang-xinyu/tensorrtx/tree/master/yolov8 with some modification so it can work with live camera feed and ros 2. Firstly convert yolov8 pytorch weights to plain weights as stated in the official resposity. We used yolov8n but other models can also be converted. For other weights see the instructions on the aforementioned repository (tensorrtx).
```
git clone https://github.com/ultralytics/ultralytics.git
wget https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8n.pt
cp ~/yolov8_trt_ros2/gen_wts.py ~/ultralytics
mv yolov8n.pt  ~/ultralytics
cd ~/ultralytics
//copy weights to this folder from official ultralytics repository
python3 gen_wts.py -w yolov8n.pt -o yolov8n.wts -t detect
// a file 'yolov8n.wts' will be generated.
```
3. Build yolov8_trt_ros2 and serialize weights to tensort engine.
```
cd ~/yolov8_trt_ros2
// update kNumClass in config.h if your model is trained on custom dataset
mkdir build
cd build
cp ~/ultralytics/yolov8n.wts ~/yolov8_trt_ros2/build
cmake ..
make
sudo ./yolov8_det -s yolov8n.wts yolov8n.engine n
// For other models change argument  as follows.
//sudo ./yolov8_det -s [.wts] [.engine] [n/s/m/l/x/n6/s6/m6/l6/x6]  // serialize model to plan file
//Custom yolov8 models can also be used in the same way.
```
4. Inference with camera or ROS 2 image topic (humble/foxy).
Before inference make sure to change the paths (engine_file_path, library) and camera topics in scripts infer.py & infer_ros.py. Execute the scripts below for inference.\
`engine_file_path = "/home/{$USER}/yolov8_trt_ros2/build/yolov8s.engine"`\
`library = "/home/{$USER}/yolov8_trt_ros2/build/libmyplugins.so"`\
Change {$USER} to approprite host name.

```
cd ~/yolov8_trt_ros2
// Camera
python3 infer.py
//ROS 2
python3 infer_ros.py
  // Make sure to change weights and the camera topic.
```
After execution of the inference script opencv window will appear showing the detections. Write "deactivate" to close the python virtual environment. For reusing the model activate the environment again and run the inference script only as setup is already complete.
Additionally, if you want to publish detection data (class label, score and bbox) along with output image use the following script instead of "infer_ros.py".
```
python3 yolo_pub.py
```
The detections will be published as ROS 2 topic named `/yolov8/result`, subscribe the detections data and parse using the following script.
```
python3 yolo_sub.py
```
5. Results\
\
![alt text](https://github.com/sulemank137/yolov8_trt_ros2/blob/525a0636d1fa51f37355d43aea834e9980cedc10/images/yolov8_sub.png)\

6. For weights with custom data please modify the class labels in script "yolov8_det_vid.py". The variable name is `self.categories` (line 83). 
