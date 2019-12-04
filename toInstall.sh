#!/bin/bash
#FORMAT
FRM='\033['
BLACK='0;30'
RED='1;31'
GREEN='1;32'
YELLOW='1;33'
BLUE='1;34'
PURPLE='1;35'
CYAN='1;36'
WHITE='1;37'
BGBLACK=';40m'
BGRED=';41m'
BGGREEN=';42m'
BGYELLOW=';43m'
BGBLUE=';44m'
BGWHITE=';47m'
NC='\033[0m'

# These script was edited by Edd-2, and have code lines taken from JustinasGPU script
# original made by Reynaldos Team

INSTALL_DIR=$HOME

################################################
####       KINECT    DRIVERS        ############
echo -e "${FRM}${WHITE}${BGBLUE} Preparing to build Prime sense drivers ${NC}"
cd $INSTALL_DIR
mkdir -p prime_sense
cd prime_sense
sensorKinect_file="$(pwd)/SensorKinect"
if [ ! -d "$sensorKinect_file" ]; then
    git clone https://github.com/ph4m/SensorKinect.git
fi
cd SensorKinect
git checkout unstable
echo -e "${FRM}${GREEN}${BGBLUE} Prime sense drivers has been prepared ${NC}"
cd ../SensorKinect/Platform/Linux/CreateRedist
echo -e "${FRM}${WHITE}${BGBLUE} Installing Prime sense drivers ${NC}"
./RedistMaker
cd ../Redist/Sensor-Bin-Linux-x64-v5.1.2.1/
sudo ./install.sh
echo -e "${FRM}${GREEN}${BGBLUE}Prime sense drivers has been installed${NC}"


################################################
####       INSTALLING   NITE        ############
echo -e "${FRM}${WHITE}${BGBLUE}Installing NITE for skeleton traking${NC}"
cd $INSTALL_DIR
nite_file="$(pwd)/NITE-Bin-Linux-x64-v1.5.2.23.tar.zip"
if [ ! -f "$nite_file" ]; then
    wget http://www.openni.ru/wp-content/uploads/2013/10/NITE-Bin-Linux-x64-v1.5.2.23.tar.zip
    unzip NITE-Bin-Linux-x64-v1.5.2.23.tar.zip
    tar -xvf NITE-Bin-Linux-x64-v1.5.2.23.tar.bz2
fi
cd NITE-Bin-Dev-Linux-x64-v1.5.2.23
sudo ./install.sh
echo -e "${FRM}${GREEN}${BGBLUE}NITE correctly installed ${NC}"


#################################################
####       INSTALLING  OPENNI        ############
echo -e "${FRM}${WHITE}${BGBLUE}Installing OpenNI to update default libraries${NC}"
cd $INSTALL_DIR
openni_file_dir="$(pwd)/OpenNI"
if [ ! -f "$openni_file_dir" ]; then
    git clone https://github.com/OpenNI/OpenNI
fi
cd OpenNI/
git checkout unstable
cd Platform/Linux/CreateRedist
./RedistMaker	
cd ../Redist/OpenNI-Bin-Dev-Linux-x64-v1.5.8.5/
sudo ./install.sh
echo -e "${FRM}${GREEN}${BGBLUE}OpenNI has been installed to update default libraries${NC}"


##################################################################
#######                  DEPENDENCIES              ###############
echo -e "${FRM}${WHITE}${BGBLUE} Installing opencv dependencies ${NC}"
sudo apt-get install -y pkg-config unzip ffmpeg qtbase5-dev python-dev python3-dev python-numpy python3-numpy
sudo apt-get install -y libopencv-dev libgtk-3-dev libdc1394-22 libdc1394-22-dev libjpeg-dev libpng12-dev libtiff5-dev libjasper-dev
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libxine2-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
sudo apt-get install -y libv4l-dev libtbb-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev
#python-vtk is a package that uninstall some package of ros
#sudo apt-get install -y libvorbis-dev libxvidcore-dev v4l-utils python-vtk
sudo apt-get install -y libvorbis-dev libxvidcore-dev v4l-utils
sudo apt-get install -y liblapacke-dev libopenblas-dev checkinstall
sudo apt-get install -y libgdal-dev
sudo apt-get install -y libatlas-base-dev
#This libraries are only for test 
#sudo apt-get install -y libvtk6*
#sudo apt-get install -y libopenni-dev
sudo easy_install pip


############################################################
#######              CUDA 8                        #########
echo -e "${FRM}${GREEN}${BGBLUE} Opencv dependencies Have been installed${NC}"
echo -e "${FRM}${WHITE}${BGBLUE} Installing Cuda 8, Patch Cuda 8 and cuDNN 5.1${NC}"
cd $INSTALL_DIR
cudaFile="$(pwd)/cuda_8.0.61_375.26_linux-run"
cudaPatchFile="$(pwd)/cuda_8.0.61.2_linux-run"
if [ ! -f "$cudaFile" ]; then
    echo -e "${FRM}${WHITE}${BGBLUE} Downloading CUDA 8 ${NC}"
    wget https://developer.nvidia.com/compute/cuda/8.0/Prod2/local_installers/cuda_8.0.61_375.26_linux-run
    echo -e "${FRM}${GREEN}${BGBLUE} CUDA 8 has been downloaded ${NC}"
fi
if [ ! -f "$cudaPatchFile" ]; then
    echo -e "${FRM}${WHITE}${BGBLUE} Downloading Patch CUDA 8 ${NC}"
    wget https://developer.nvidia.com/compute/cuda/8.0/Prod2/patches/2/cuda_8.0.61.2_linux-run
    echo -e "${FRM}${GREEN}${BGBLUE} Patch CUDA 8 has been downloaded ${NC}"
fi
chmod +x cuda_8.0.61_375.26_linux-run
chmod +x cuda_8.0.61.2_linux-run 
echo -e "${FRM}${WHITE}${BGBLUE} Installing CUDA 8${NC}"
sudo ./cuda_8.0.61_375.26_linux-run --silent --toolkit --samples --samplespath=$INSTALL_DIR
#sudo ./cuda_8.0.61_375.26_linux-run
echo -e "${FRM}${GREEN}${BGBLUE} CUDA 8 has been installed ${NC}"
echo -e "${FRM}${WHITE}${BGBLUE} Installing Patch CUDA 8${NC}"
sudo ./cuda_8.0.61.2_linux-run --silent --accept-eula --installdir=/usr/local/cuda-8.0
#sudo ./cuda_8.0.61.2_linux-run
echo -e "${FRM}${GREEN}${BGBLUE} Patch CUDA 8 has been installed ${NC}"
echo -e "${FRM}${WHITE}${BGBLUE} Installing cuDNN 5.1 for CUDA 8 ${NC}"
cd $INSTALL_DIR
ubuntu_version="$(lsb_release -r)"
CUDNN_URL="http://developer.download.nvidia.com/compute/redist/cudnn/v5.1/cudnn-8.0-linux-x64-v5.1.tgz"
wget -c ${CUDNN_URL}
mkdir cudnn_5_1
tar -xvf cudnn-8.0-linux-x64-v5.1.tgz -C cudnn_5_1
sudo cp cudnn_5_1/cuda/include/* /usr/local/cuda-8.0/include
sudo cp -av cudnn_5_1/cuda/lib64/* /usr/local/cuda-8.0/lib64
echo -e "${FRM}${GREEN}${BGBLUE} cuDNN has been installed ${NC}"
echo -e "${FRM}${GREEN}${BGBLUE} Cuda 8, Patch Cuda 8 and cuDNN 5.1 has been installed${NC}"



###########################################################
###########   DOWNLOAD-COMPILE   OPENCV3.3       ##########  
cd $INSTALL_DIR
opencvFile="$(pwd)/opencv-3.3.1.zip"
opencv_contrib_file="$(pwd)/opencv_contrib-3.3.1.zip"
if [ ! -f "$opencvFile" ]; then
    echo -e "${FRM}${WHITE}${BGBLUE} Downloading OpenCV 3.3.1 ${NC}"
    wget https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/3.3.1/opencv-3.3.1.zip
    echo -e "${FRM}${GREEN}${BGBLUE} OpenCV 3.3.1 has been downloaded ${NC}"
    unzip opencv-3.3.1.zip
fi
if [ ! -f "$opencv_contrib_file" ]; then
    echo -e "${FRM}${WHITE}${BGBLUE} Downloading OpenCV 3.3.1 contrib ${NC}"
    wget https://github.com/opencv/opencv_contrib/archive/3.3.1.zip
    echo -e "${FRM}${GREEN}${BGBLUE} OpenCV 3.3.1 contrib  has been downloaded ${NC}"
    mv 3.3.1.zip opencv_contrib-3.3.1.zip
    unzip opencv_contrib-3.3.1.zip
fi
echo -e "${FRM}${WHITE}${BGBLUE} Installing OpenCV 3.3.1 ${NC}"
cd opencv-3.3.1
mkdir build
cd build
cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPEPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON -D WITH_OPENNI=ON -D WITH_OPENCL=OFF -D CMAKE_BUILD_TYPE=RELEASE FORCE_VTK=ON -D WITH_CUBLAS=ON -D CUDA_NVCC_FLAGS="-D_FORCE_INLINES" -D WITH_GDAL=ON -D WITH_XINE=ON -D BUILD_EXAMPLES=ON -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.3.1/modules ..
make -j4
sudo make install
sudo touch /etc/ld.so.conf.d/opencv.conf
sudo /bin/su -c "echo '/usr/local/lib' >> /etc/ld.so.conf.d/opencv.conf"
sudo ldconfig
echo -e "${FRM}${GREEN}${BGBLUE} OpenCV 3.3.1 has been installed ${NC}"


#####################################################################
##########    REPLACING  OPENCV(Kinect)  ---> OPENCV(ROS)     #######
echo -e "${FRM}${WHITE}${BGBLUE}Coping OpenCV libraries to ros directory${NC}"
FILES="/usr/local/lib/libopencv*"
pathCopy="/opt/ros/kinetic/lib/"
pattherDelete=$pathCopy"libopencv*"
for f in $pattherDelete
do
    sudo rm $f
done

for f in $FILES
do
    filename=$(basename "$f")
    extension="${filename##*.}"
    if [ $extension = "so" ]; then
	sudo cp $f $pathCopy
	newfilename="${filename%.*}"3.$extension
	sudo mv $pathCopy$filename $pathCopy$newfilename
	sudo echo $newfilename
	cd $pathCopy
	sudo ln -s $newfilename.3.3 $newfilename.3.3.1
	sudo ln -s $newfilename $newfilename.3.3
    fi
    #echo $f
    #mv $f $f
done
echo -e "${FRM}${GREEN}${BGBLUE}Have been copying the OpenCV libraries to ROS directory${NC}"
