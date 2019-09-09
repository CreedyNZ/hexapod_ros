

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt update
sudo apt upgrade

sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake

sudo rosdep init
rosdep update

mkdir -p ~/ros_catkin_ws
cd ~/ros_catkin_ws

rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
wstool init src kinetic-ros_comm-wet.rosinstall

# if fail

# mkdir -p ~/ros_catkin_ws/external_src
# cd ~/ros_catkin_ws/external_src
# wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
# unzip assimp-3.1.1_no_test_models.zip
# cd assimp-3.1.1
# cmake .
# make
# sudo make install

# end fail

cd ~/ros_catkin_ws
rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:buster


sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic

# if missing boost

wget -O boost_1_58_0.tar.gz https://sourceforge.net/projects/boost/files/boost/1.58.0/boost_1_58_0.tar.gz/download
tar xzvf boost_1_58_0.tar.gz
cd boost_1_58_0/

sudo apt-get update
sudo apt-get install build-essential g++ python-dev autotools-dev libicu-dev build-essential libbz2-dev libboost-all-dev -y

./bootstrap.sh --prefix=/usr/

./b2

sudo ./b2 install

# end missing boost


echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc


# to install more packages

cd ~/ros_catkin_ws
rosinstall_generator 'package name' --rosdistro kinetic --deps --wet-only --tar > kinetic-custom_ros.rosinstall

wstool merge -t src kinetic-custom_ros.rosinstall
wstool update -t src

rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -r --os=debian:buster

sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic

#end install more packages


rosinstall_generator tf2 tf move_base xacro --rosdistro kinetic --deps --wet-only --tar > kinetic-custom_ros.rosinstall
