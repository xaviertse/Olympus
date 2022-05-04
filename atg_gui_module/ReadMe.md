# Installation Instructions of Automatic Toolpath Generation #
 Author: Srinivasan L (ARA)
 Date Created: February 20, 2019
 Last Modified: August 11, 2021

## Step 1: Installing  the ROS and other dependencies ##

### ROS ###
```
sudo apt-get update
sudo apt-get upgrade
sudo sh -c '. /etc/lsb-release && echo "deb http://mirror-ap.packages.ros.org/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Other libraries ###
```
sudo apt-get install ros-kinetic-qt-*
sudo apt-get install python-catkin-tools
easy_install numpy scipy Sphinx numpydoc nose pykalman
sudo apt-get install meshlab
sudo apt-get install ftp wput xdotool iputils-ping
```

### Python
```
sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose
```

### Install python3.6
```
sudo add-apt-repository ppa:jonathonf/python-3.6
sudo apt-get update
sudo apt-get install python3.6
sudo apt-get install python3.6-dev libmysqlclient-dev
sudo apt-get install python-skimage
```

### Moveit Package and Eigen3
```
sudo apt-get install ros-kinetic-moveit
sudo apt-get install libeigen3-dev
```

### Ros industrial installation
```
sudo apt-get install ros-kinetic-industrial-core
sudo apt-get install ros-kinetic-abb ros-kinetic-universal-robot ros-kinetic-ros-canopen (Type this)
```


### Eigen
```
wget http://launchpadlibrarian.net/209530212/libeigen3-dev_3.2.5-4_all.deb
sudo dpkg -i libeigen3-dev_3.2.5-4_all.deb
 sudo apt-mark hold libeigen3-dev

```

## Step 2: Install QT Creator
```
sudo add-apt-repository ppa:levi-armstrong/qt-libraries-xenial
sudo add-apt-repository ppa:levi-armstrong/ppa
sudo apt update && sudo apt install qt59creator
sudo apt install qt57creator-plugin-ros
```

### Install VTK dependancies
```
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential

sudo add-apt-repository -y ppa:webupd8team/java && sudo apt update && sudo apt -y install oracle-java8-installer

sudo apt -y install g++ cmake cmake-gui doxygen mpi-default-dev openmpi-bin openmpi-common libusb-1.0-0-dev libqhull* libusb-dev libgtest-dev

sudo apt -y install git-core freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libphonon-dev libphonon-dev phonon-backend-gstreamer

sudo apt -y install phonon-backend-vlc graphviz mono-complete libflann-dev

sudo apt -y install libflann1.8 libboost1.58-all-dev
sudo apt-get install rviz

sudo apt-get install libqt4-sql-mysql
sudo apt install apt-file
sudo apt-file update

wget http://launchpadlibrarian.net/209530212/libeigen3-dev_3.2.5-4_all.deb

sudo dpkg -i libeigen3-dev_3.2.5-4_all.deb
sudo apt-mark hold libeigen3-dev
```

## Step 3: Install VTK 8.1.1
```

wget http://www.vtk.org/files/release/8.1/VTK-8.1.1.tar.gz
tar -xf VTK-8.1.1.tar.gz
cd VTK-8.1.1/
mkdir build
cd build
cmake-gui

# Click on  VTK_Group_QT to be ON
make -j4
sudo make install
```


## Step 4: Install PCL 1.8.1
```
cd ..
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz
tar -xf pcl-1.8.1.tar.gz
cd pcl-pcl-1.8.1 && mkdir build && cd build
cmake-gui
# Click on  BUILD_Surface_on_nurbs to be ON
make -j4
sudo make install
sudo apt-get install rviz

Install pcl-ros - sudo apt-get install ros-kinetic-pcl-ros

Install tf_geometry_msgs sudo apt-get install ros-kinetic-tf2-geometry-msgs
```

## Step 5: To run the program
```
cd
cd atg_modules/
catkin_make -j8
source devel/setup.bash
rosrun atg_gui atg_gui

```



# Command history that was successful:
## Srini
```
    1 sudo apt-get install rviz
    2  cd eigen/
    3  cd ..
    4  sudo mkdir eigen_dir
    5  ls
    6  cd eigen_dir/
    7  sudo cmake eigen
    8  cd ..
    9  cd eigen_dir/
   10  sudo cmake eigen
   11  sudo make install
   12  cd Downloads/
   13  tar -xjvf firefox-64.0.2.tar.bz2
   14  cd firefox
   15  ./configure
   16  . /configure
   17  ./ configure
   18  make
   19  cd
   20  clear
   21  apt-cache search firefox
   22  sudo apt-get instalkl firefox //wrong
   23  sudo apt-get install firefox
   24  sudo apt-get update
   25  sudo apt-get upgrade
   26  cd
   27  sudo sh -c '. /etc/lsb-release && echo "deb http://mirror-ap.packages.ros.org/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
   28  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
   29  sudo apt-get update
   30  sudo apt-get install ros-kinetic-desktop-full
   31  sudo rosdep init
   32  rosdep update
   33  echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
   34  source ~/.bashrc
   35  sudo apt-get install ros-kinetic-qt-*
   36  sudo apt-get install python-catkin-tools
   37  sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
   38  sudo add-apt-repository -y ppa:webupd8team/java && sudo apt update && sudo apt -y install oracle-java8-installer
   39  sudo apt -y install g++ cmake cmake-gui doxygen mpi-default-dev openmpi-bin openmpi-common libusb-1.0-0-dev libqhull* libusb-dev libgtest-dev
   40  sudo apt -y install git-core freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libphonon-dev libphonon-dev phonon-backend-gstreamer
   41  sudo apt -y install phonon-backend-vlc graphviz mono-complete libflann-dev
   42  sudo apt -y install libflann1.8 libboost1.58-all-dev
   43   cd
   44  cd
   45  wget http://launchpadlibrarian.net/209530212/libeigen3-dev_3.2.5-4_all.deb
   46  sudo dpkg -i libeigen3-dev_3.2.5-4_all.deb
   47  sudo apt-mark hold libeigen3-dev
   48  sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose
   49  sudo apt-get install python3.6-dev libmysqlclient-dev
   50  sudo apt-get install python3.6
   51  sudo add-apt-repository ppa:jonathonf/python-3.6
   52  sudo apt-get update
   53  sudo apt-get install python3.6
   54  sudo apt-get install python3.6-dev libmysqlclient-dev
   55  sudo apt-get install python-skimage
   56  sudo apt-get install ros-kinetic-moveit
   57  sudo apt-get install libeigen3-dev
   58  sudo apt-get install ros-kinetic-industrial-core
   59  sudo apt-get install ros-kinetic-abb ros-kinetic-universal-robot  ros-kinetic-ros-canopen
   60  sudo apt-get install ros-kinetic-industrial-core
   61  sudo apt-get install ros-kinetic-abb ros-kinetic-universal-robot ros-kinetic ros-canopen
   62  sudo apt-get install ros-kinetic-abb ros-kinetic-universal-robot ros-kinetic-ros-canopen
   63  sudo add-apt-repository ppa:levi-armstrong/qt-libraries-xenial
   64  sudo add-apt-repository ppa:levi-armstrong/ppa
   65  sudo apt-get update && sudo apt-get install qt57creator-plugin-ros libqtermwidget57-0-dev
   66  sudo apt update && sudo apt install qt59creator
   67  sudo apt remove --purge '^qt57.*'
   68  sudo apt update && sudo apt install qt59creator
   69  sudo apt install qt57creator-plugin-ros
   70  cd
   71  wget http://launchpadlibrarian.net/209530212/libeigen3-dev_3.2.5-4_all.deb
   72  sudo dpkg -i libeigen3-dev_3.2.5-4_all.deb
   73  sudo apt-mark hold libeigen3-dev
   74  cd
   75  wget http://www.vtk.org/files/release/8.0/VTK-8.0.1.tar.gz
   76  tar -xf VTK-8.0.1.tar.gz
   77  cd VTK-8.0.1/
   78  mkdir build
   79  cd build
   80  cmake-gui
   81  make -j4
   82  sudo make install
   83  cd
   84  wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz
   85  tar -xf pcl-1.8.1.tar.gz
   86  cd pcl-pcl-1.8.1 && mkdir build && cd build
   87  cmake-gui
   88  make -j4
   89  cd ..
   90  sudo rmdir build
   91  sudo rm -r build
   92  cd
   93  apt-get install libqt4-sql-mysql
   94  sudo apt-get install libqt4-sql-mysql
   95  cd pcl-pcl-1.8.1 && mkdir build && cd build
   96  cmake-gui
   97  make -j4
   98  cd
   99  cd pcl-pcl-1.8.1
  100  sudo rm -r build
  101  cd
  102  sudo apt-get install libqt4-dev-sql-mysql
  103  sudo apt-get install libqt4-dev
  104  sudo apt-get install libqt5-dev
  105  sudo apt-get install libqt4-*
  106  cd pcl-pcl-1.8.1
  107  ls
  108  cd pcl-pcl-1.8.1 && mkdir build && cd build
  109  cd
  110  cd pcl-pcl-1.8.1 && mkdir build && cd build
  111  cmake-gui
  112  make -j4
  113  cd..
  114  sudo rm -r build
  115  cd ..
  116  sudo rm -r build
  117  cd
  118  sudo apt-get install libqt4-core libqt4-gui
  119  sudo apt-get install libqt4
  120  sudo apt install libqt4-designer libqt4-opengl libqt4-svg libqtgui4 libqtwebkit4
  121  $ apt-file search /usr/include/qt4/QtGui | grep '/usr/include/qt4/QtGui/QtGui'
  122  libqt4-dev: /usr/include/qt4/QtGui/QtGui
  123  $ apt-get install libqt4-dev
  124  sudo $ apt-get install libqt4-dev
  125  sudo apt-get install libqt4-dev
  126  apt-file search /usr/include/qt4/QtGui | grep '/usr/include/qt4/QtGui/QtGui'
  127  sudo apt install apt-file
  128  sudo apt-file search /usr/include/qt4/QtGui | grep '/usr/include/qt4/QtGui/QtGui'
  129  sudo apt update apt-file
  130  sudo apt-file update
  131  sudo apt-file search /usr/include/qt4/QtGui | grep '/usr/include/qt4/QtGui/QtGui'
  132  /usr/include/qt4/QtGui/QtGui
  133  sudo /usr/include/qt4/QtGui/QtGui
  134  sudo apt-file search /usr/include/qt4/QtGui | grep '/usr/include/qt4/QtGui/QtGui'
  135  sudo apt-get install build-essential subversion libsdl1.2-dev libgtk2.0-dev libgconf2-dev libsdl-ttf2.0-dev gcc-4.6 g++-4.6
  136  cd pcl-pcl-1.8.1 && mkdir build && cd build
  137  cmake-gui
  138  clear
  139  cd
  140  sudo apt-get update
  141  sudo apt-get upgrade
  142  sudo sh -c '. /etc/lsb-release && echo "deb http://mirror-ap.packages.ros.org/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
  143  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
  144  sudo apt-get update
  145  sudo apt-get install ros-kinetic-desktop-full
  146  sudo rosdep init
  147  echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
  148  source ~/.bashrc
  149  sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
  150  sudo add-apt-repository -y ppa:webupd8team/java && sudo apt update && sudo apt -y install oracle-java8-installer
  151  sudo apt -y install g++ cmake cmake-gui doxygen mpi-default-dev openmpi-bin openmpi-common libusb-1.0-0-dev libqhull* libusb-dev libgtest-dev
  152  sudo apt -y install git-core freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libphonon-dev libphonon-dev phonon-backend-gstreamer
  153  sudo apt -y install phonon-backend-vlc graphviz mono-complete libflann-dev
  154  sudo apt -y install libflann1.8 libboost1.58-all-dev
  155  wget http://launchpadlibrarian.net/209530212/libeigen3-dev_3.2.5-4_all.deb
  156  sudo dpkg -i libeigen3-dev_3.2.5-4_all.deb
  157  sudo apt-mark hold libeigen3-dev
  158  wget http://www.vtk.org/files/release/8.1/VTK-8.1.1.tar.gz
  159  tar -xf VTK-8.1.1.tar.gz
  160  cd VTK-8.1.1/
  161  mkdir build
  162  cd build
  163  cmake-gui
  164  make -j4
  165  cd
  166  ls
  167  sudo rm -r VTK-8.0.1
  168  ls
  169  cd VTK-8.1.1/build
  170  sudo make install
  171  cd ..
  172  ls
  173  sudo rm -r pcl-pcl-1.8.1
  174  ls
  175  cd VTK-8.1.1/
  176  wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.tar.gz
  177  tar -xf pcl-1.8.0.tar.gz
  178  cd pcl-pcl-1.8.0 && mkdir build && cd build
  179  cmake-gui
  180  make -j4
  181  cd..
  182  cd ..
  183  sudo rm -r pcl-pcl-1.8.0
  184  ls
  185  wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.tar.gz
  186  tar -xf pcl-1.8.0.tar.gz
  187  sudo rm -r pcl-pcl-1.8.0
  188  sudo rm -r pcl-pcl-1.8.0.tar.gz
  189  wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz
  190  tar -xf pcl-1.8.1.tar.gz
  191  cd pcl-pcl-1.8.1 && mkdir build && cd build
  192  cmake-gui
  193  make -j4
  194  sudo make install
  195  cd
  196  cd diamond_ws/
  197  catkin_make -j8
  198  source devel/setup.bash
  199  rosrun diamond_gui diamond_gui
  200  catkin_make -j8
  201  rosrun diamond_gui diamond_gui
  ```
