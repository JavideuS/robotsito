# Using the official ROS 2 Jazzy image as the base image
FROM ros:jazzy

#Labels
ARG email="javi.rm2005@gmail.com"
ARG username="JavideuS"
LABEL email=$email
LABEL mantainer=$username


#Workdir
ARG ws=~/robotsito/robotsito_ws/
WORKDIR $ws

# Install necessary dependencies
RUN apt-get update && apt-get install -y python3-venv python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create a virtual environment in the project directory and install the adafruit-circuitpython-servokit package
RUN python3 -m venv $ws/venv \
    #Source == . 
    && . $ws/venv/bin/activate \ 
    && pip install adafruit-circuitpython-servokit

# Copy the src directory into the container (Also as src folder)
COPY src src

# Set the environment variables for the virtual environment
ENV VIRTUAL_ENV=$ws/venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Install ROS 2 package dependencies and build the workspace
RUN . /opt/ros/jazzy/setup.sh \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build

# Source the workspace setup script
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "source $ws/install/setup.bash" >> ~/.bashrc

# Set the entrypoint to bash
ENTRYPOINT ["/bin/bash"]
