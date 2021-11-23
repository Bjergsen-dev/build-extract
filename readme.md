# Buiding Extraction
## 1. What's it？
&ensp;&ensp;This is a method to extract buildings in UAV images with a rough mask border prediction provided by point-cloud or CNN result. This program is now in developing under [Dream_BIG]() group, and the developers can join this for more details.
## 2. The pipeline of Build-Extract
![avatar](./images/pipeline.png)

- Extract lines in UAV Image.
- Filter the noise with rough mask or point-cloud.
- Predict the un-tracked lines.
- E-C(extract and classify) the inside lines for whole roof rebuild.
- Rebuild 3D-wireframe model.

## 3. Dependency Compier
- Opencv
  ```shell
  tar -xzvf opencv-3.4.12.tar.gz
  cd opencv-3.4.12
  mkdir build
  cd build
  cmake ..
  make -j8
  sudo make install
  ```

- Gdal
  ```shell
  tar -xzvf gdal-2.4.2.tar.gz
  cd gdal-2.4.2/gdal
  ./configure
  make -j8
  sudo make install
  ```

- Eigen
  ```shell
  tar -xzvf eigen-3.3.9.tar.gz
  cd eigen-3.3.9
  mkdir build
  cd build
  cmake ..
  make -j8
  sudo make install
  ```

 - boost
   ```shell
   tar -xzvf boost_1_74_0.tar.gz
   cd boost_1_74_0
   apt-get install mpi-default-dev libicu-dev python-dev python3-dev libbz2-dev zlib1g-dev
   ./bootstrap.sh
   ./b2
   sudo ./b2 install
   ```

 - Flann
   ```shell
   tar -xzvf flann-1.9.1.tar.gz
   cd flann-1.9.1
   mkdir build
   cd build
   cmake ..
   make -j8
   sudo make install
   ```

 - VTK
   ```shell
   tar -xzvf VTK-8.2.0.tar.gz
   cd VTK-8.2.0
   mkdir build
   cd build
   cmake ..
   make -j8
   sudo make install
   ```  

 - PCL
   ```shell
   tar -xzvf pcl-pcl-1.11.1.tar.gz
   cd pcl-pcl-1.11.1
   mkdir build
   cd build
   cmake ..
   make -j8
   sudo make install
   ```
 

## 4. Dependency Download
- [OpenCV 3.XX ](https://pan.baidu.com/s/1Vm0TK5JDuFM0zj81Thdasw) (file-code:rpvd)
- [Gdal 2.XX](https://pan.baidu.com/s/1UT3rxCP66Czx1Dun4xNJYQ) (file-code:xqqk)
- [Eigen 3.XX](https://pan.baidu.com/s/19_58E-3PKd-bs7-nbN6R7w) (file-code:re9b)
- [PCL 1.11](https://pan.baidu.com/s/14zDoalpt7JcLA-j7XLw5kA) (file-code:mgif)
  - VTK 8.2.0
  - boost 
  - Flann

## 4. Exp-Data
- [uav images](https://pan.baidu.com/s/19EZl3WRQHebIPi3WMfGxrQ) (file-code:pvgd)
- [point-cloud](https://pan.baidu.com/s/1jNif1r5aWR1537pi4Y3YFw) (file-code:4g6k)
  
