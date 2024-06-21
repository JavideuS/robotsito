# Using the official ROS 2 Jazzy image as the base image
FROM ros:jazzy

#Labels
ARG email="javi.rm2005@gmail.com"
ARG username="JavideuS"
LABEL email=$email
LABEL mantainer=$username


#Workdir
ARG ws=/robotsito/robotsito_ws
WORKDIR $ws

# Install necessary dependencies
RUN apt-get update && apt-get install -y python3-venv python3-pip dos2unix\
    && rm -rf /var/lib/apt/lists/*

# Copy the src directory into the container (Also as src folder)
COPY src src
#Copy the bash script to work with the venv environment
COPY update_setup_scripts.sh update_setup_scripts.sh 

# Install ROS 2 package dependencies and build the workspace
RUN . /opt/ros/jazzy/setup.sh \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    #Building the package before activating the venv because it causes trouble
    && colcon build

# Create a virtual environment in the project directory and install the adafruit-circuitpython-servokit package
RUN python3 -m venv $ws/venv \
    #Source == . 
    && . $ws/venv/bin/activate \ 
    && pip install adafruit-circuitpython-servokit

#GIving exec permissions and running the script to modify setup.bash and setup.sh
RUN dos2unix update_setup_scripts.sh \
    && chmod +x update_setup_scripts.sh \
    && ./update_setup_scripts.sh

# Source the workspace setup script
RUN echo "source $ws/install/setup.bash" >> ~/.bashrc 

# Set the entrypoint to bash
ENTRYPOINT ["/bin/bash"]
