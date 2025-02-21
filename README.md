# Installation  
To get the program up and running, you will need to install the following dependencies:

### 1. [ROS](http://wiki.ros.org/ROS/Installation) (Tested with Kinetic and Melodic)
   Install the correct version of ROS on your system, by following the link. 
   </br></br>
   

### 2. [Ceres](http://ceres-solver.org/installation.html) (C++ library for modeling and solving large, complicated optimisation problems)
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
   </br>


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
   </br>
   
### 4. Library Troubleshooting  
If you encounter issues with missing libraries or include path errors, update your `c_cpp_properties.json` file to include the **ROS**, **Eigen**, and **OpenCV** libraries.  

*Be sure to include the correct versions installed on your system—Eigen3, for example.*  
</br></br>


## Usage
   ### 1. Set up the Environment
   Now you need to set up the environment in the root folder of the repository. Run the following commands:
   ```bash
   source /opt/ros/noetic/setup.bash
   source ~/"Path to Repository"/devel/setup.bash
   catkin_make clean
   catkin_make
   source devel/setup.bash
   ```
   
   ### 2. Download the Bag File
   If you are not using real hardware, you can simulate sensor data using a pre-recorded ROS bag file. Go to the Goolge Drive and download some bag files for testing.
   ```bash
   https://drive.google.com/drive/folders/1q2NZnsgNmezFemoxhHnrDnp1JV_bqrgV?usp=sharing
   ```

   ### 3. Launch LVI-SAM
   Once you have the bag file, you can launch the LVI-SAM node to prepare it to start processing the data. Depending on whether you're using the modified version or the official version of LVI-SAM, use the appropriate launch file:
   ```bash
   roslaunch lvi_sam run.launch
   ```   
   This will start processing the data from the bag file and you can monitor the output in RViz or the terminal.
      
   ### 4. Play the Bag File
   Now, in a new terminal navigate to the root of the repository. Now you can play the bag file you previously downloaded. For example, to use the `handheld.bag` file, you can play it using the following command:
   ```bash
   rosbag play bags/handheld.bag
   ```
   This will simulate the sensor data (e.g., IMU, camera, etc.) that LVI-SAM will process.
   </br></br>


## Avaliable Datasets  
The available datasets for testing include the **Garden Dataset**, which captures indoor and outdoor environments using LiDAR, IMU, and camera data, and is designed for SLAM models in structured settings. The **Jackal Dataset** provides LiDAR, camera, and GNSS/IMU data, making it suitable for localisation and navigation model tests. Additionally, a **Researcher Custom Dataset** may be used, containing various sensor modalities for specific navigation tasks. For ground truth, datasets like **M2DGR**, **UrbanNav**, and **KITTI RAW** are available, each offering GNSS, LiDAR, IMU, and camera data for testing GNSS-denied and multi-sensor localisation algorithms. The **Handheld Dataset**, on the other hand, does not provide ground truth but offers LiDAR and IMU data, ideal for evaluating SLAM performance in handheld systems.

### Ground Truth Avaliable
- [Jackal Dataset](https://drive.google.com/drive/folders/1q2NZnsgNmezFemoxhHnrDnp1JV_bqrgV)  
- [M2DGR](https://github.com/SJTU-ViSYS/M2DGR?tab=readme-ov-file#dataset-sequences) 
- [UrbanNav](https://www.dropbox.com/scl/fi/6ffoquf2y9kzyzdzhuig4/2020-03-14-16-45-35.bag.tar.gz?rlkey=kagl01g4774u2mwi595xgi8l0&e=1&dl=0)
- [KITTI RAW](https://onedrive.live.com/?redeem=aHR0cHM6Ly8xZHJ2Lm1zL3UvcyFBcVlhakVfZnQ5bHdnMHR1aHF5WnFkNE1VanFwP2U9aG52a1pv&cid=70D9B7DF4F8C1AA6&id=70D9B7DF4F8C1AA6%21459&parId=root&o=OneUp)
- [Researcher Custom Dataset](https://onedrive.live.com/?id=70D9B7DF4F8C1AA6!458&resid=70D9B7DF4F8C1AA6!458&redeem=aHR0cHM6Ly8xZHJ2Lm1zL3UvcyFBcVlhakVfZnQ5bHdnMHBhSlF1X0RSelUtR1E1P2U9QTk1eWZu&migratedtospo=true&cid=70d9b7df4f8c1aa6)

### NO Ground Truth
- [Handeld Dataset](https://drive.google.com/drive/folders/1q2NZnsgNmezFemoxhHnrDnp1JV_bqrgV)   
- [Garden Dataset](https://drive.google.com/drive/folders/1q2NZnsgNmezFemoxhHnrDnp1JV_bqrgV)
</br></br>


## References  
- [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM)  
- [LVI-SAM (New)](https://github.com/Cc19245/LVI-SAM-Easyused/tree/new)  
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
- [VINS-MONO](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
