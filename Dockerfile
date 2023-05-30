# Use an official Ubuntu runtime as a parent image
FROM osrf/ros:kinetic-desktop
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
# Set the working directory in the container
WORKDIR /usr/src/app

# Install any needed packages specified in requirements
# RUN apt-get update 
# RUN apt-get install -y cmake
# RUN apt-get install -y libboost1.53-all-dev 
# RUN libeigen3-dev
# RUN libpcl-dev
# RUN libomp-dev

# Copy the current directory contents into the container 
COPY . /usr/src/app/src/precision-tracking

# Copy the Boost .tar.bz2 file into the Docker image
# COPY ./boost_1_58_0.tar.gz /usr/src/app/

# # # Extract and install Boost
# RUN tar xvf /usr/src/app/boost_1_58_0.tar.gz && \
#     cd boost_1_58_0 && \
#     ./bootstrap.sh --prefix=/opt/boost-install && \
#     ./b2 install && \
#     cd /usr/src/app && \
#     rm -rf /usr/src/app/boost_1_58_0.tar.gz && \
#     rm -rf /usr/src/app/boost_1_58_0

# ENV OLD_PATH=$PATH
# ENV PATH=/opt/boost-install:$PATH

# # Set the working directory to the cloned directory
WORKDIR /usr/src/app/

RUN apt update && \
    apt install -y \
    cmake \
    libeigen3-dev \
    libpcl-dev \
    libomp-dev \
    wget

ENV PATH=/usr/include:$PATH

RUN source /opt/ros/kinetic/setup.bash && \
    catkin_make

# # Build the project
# RUN mkdir build && \
#     cd build && \
#     cmake .. && \
#     make -j${nproc}

# ENV PATH=$OLD_PATH
# # # Download the test data
# # RUN wget http://cs.stanford.edu/people/davheld/public/test.tm
CMD ["/bin/bash"]