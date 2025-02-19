# GNSS-FreeNav  

GNSS-FreeNav is a deep learning-based navigation system designed for **GNSS-denied environments**, leveraging **IMU, LiDAR, and Camera data** to estimate precise vehicle positioning. This project aims to enhance autonomous navigation capabilities in high-risk or GPS-denied areas, such as defense applications and remote environments.  


## Features  
- **Deep Learning-Based Localisation** – Uses AI models & Machine Learning Techniques to predict GNSS coordinates from onboard sensor data.  
- **Sensor Fusion** – Integrates IMU, LiDAR, and Camera inputs for improved accuracy.  
- **Robust to GNSS Outages** – Provides navigation solutions when GPS signals are unavailable.  
- **Optimised for Embedded Systems** – Designed for deployment on platforms like the Jetson Nano.  
- **Incorporates Open-Source Models** – Builds upon existing research, including LVI-SAM.  


## Installation  
To get the program up and running, you will need to install the following dependencies:

### 1. [ROS](http://wiki.ros.org/ROS/Installation) (Tested with Kinetic and Melodic)
   Install the correct version of ROS on your system.

### 2. [Ceres](http://ceres-solver.org/installation.html) (C++ library for modeling and solving large, complicated optimization problems)
   Install Ceres by following these steps:
   ```bash
   sudo apt-get install -y libgoogle-glog-dev
   sudo apt-get install -y libatlas-base-dev
   wget -O ~/Downloads/ceres.zip https://github.com/ceres-solver/ceres-solver/archive/1.14.0.zip
   cd ~/Downloads/ && unzip ceres.zip -d ~/Downloads/
   cd ~/Downloads/ceres-solver-1.14.0
   mkdir ceres-bin && cd ceres-bin
   cmake ..
   sudo make install -j4 
   ```

### 3. [GTSAM](https://gtsam.org/get_started/) (Georgia Tech Smoothing and Mapping library)
Follow these steps to install GTSAM:
   #### 1. Clear old versions (if applicable)
   To remove any existing GTSAM installations, run the following commands:
   ```bash
   sudo rm -rf /usr/local/include/gtsam
   sudo rm -rf /usr/local/lib/libgtsam*
   sudo rm -rf /usr/local/share/cmake/GTSAM
   ```

   #### 2. Install Eigen
   Make sure the system uses the correct version of Eigen by installing it with:
   ```bash
   sudo apt install libeigen3-dev
   ```

   #### 3. Install GTSAM
   Clone the GTSAM repository and build it:
   ```bash
   cd ~/Documents/GNSS-FreeNav  # Adjust path if needed
   git clone --branch 4.2a1 https://github.com/borglab/gtsam.git gtsam
   cd gtsam
   mkdir build && cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DGTSAM_USE_SYSTEM_EIGEN=ON
   make -j$(nproc)
   sudo make install
   ```

   #### 4. Verify the installation
   After confirming that GTSAM is working, you can remove the temporary folder:
   ```bash
   rm -rf ~/Documents/GNSS-FreeNav/gtsam
   ```

### 4. Library Troubleshooting  
If you encounter issues with missing libraries or include path errors, update your `c_cpp_properties.json` file to include the **ROS**, **Eigen**, and **OpenCV** libraries.  

*Be sure to include the correct versions installed on your system—Eigen3, for example.*  


## Usage  
   Now you need to set up the environment in the root folder of the repository. Run the following commands:
   ```bash
   source /opt/ros/noetic/setup.bash
   source ~/"Path to Repository"/devel/setup.bash
   catkin_make clean
   catkin_make
   source devel/setup.bash
   ```
(TODO: Provide details on how to run the model tests, input data format, and expected outputs)  





## References  
- [LVi-SAM](https://github.com/TixiaoShan/LVI-SAM)  
(TODO: Add relevant papers or projects)  

## License  
(Add license details, if applicable)