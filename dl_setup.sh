#!/bin/bash

# Update package lists (assuming Debian/Ubuntu-based system)
sudo apt-get update

# Install necessary system dependencies
sudo apt-get install -y python3-pip python3-venv libopenblas-dev libhdf5-serial-dev hdf5-tools libhdf5-dev \
                        zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran libjpeg-dev

# Create a Python virtual environment named "dl_env"
python3 -m venv dl_dep --system-site-packages

# Activate the virtual environment (source command might differ for other shells)
source dl_dep/bin/activate  # For Bash

# Upgrade pip within the virtual environment
python3 -m pip install --upgrade pip

# Install dependencies (excluding those requiring sudo)
pip3 install testresources setuptools==65.5.0 tqdm imutils pyyaml

# Set environment variable for PyTorch installation
TORCH_INSTALL=https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl

# Install PyTorch using pip within the virtual environment (avoiding sudo)
pip3 install --no-cache $TORCH_INSTALL

# Install NumPy and other dependencies with specific versions (using pip within the virtual environment)
pip3 install -U numpy==1.22 future==0.18.2 mock==3.0.5 keras_preprocessing==1.1.2 keras_applications==1.0.8 gast==0.4.0 \
                protobuf pybind11 pkgconfig packaging h5py==3.7.0

# Install TensorFlow with specific version and NVIDIA compatibility (using pip within the virtual environment)
pip3 install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v512 tensorflow==2.12.0+nv23.06


#copy open_cv to virtual environment from local (Not required now as virtual env is made with system site packages)
#cp -r /home/wego/.local/lib/python3.8/site-packages/cv2 /home/wego/dl_env/lib/python3.8/site-packages/
#cp -r /home/wego/.local/lib/python3.8/site-packages/opencv_contrib_python-4.9.0.80.dist-info/ /home/wego/dl_env/lib64/python3.8/site-packages/
#cp -r /home/wego/.local/lib/python3.8/site-packages/opencv_contrib_python.libs/ /home/wego/dl_env/lib64/python3.8/site-packages/

#install pycuda
python3 -m pip install pycuda



# Clone and install torchvision (consider modifying commands if using a different branch or version)
git clone --branch v0.16.0 https://github.com/pytorch/vision torchvision
cd torchvision
pip3 install .  # User-based installation within the virtual environment
cd ..
# Deactivate the virtual environment (optional)
deactivate

echo "** Dependencies installed successfully! **"
