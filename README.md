# Quadruped Robot Project - Robotsito

This repository contains all the necessary code and designs to build and control the quadruped robot. 
And it is organized into several branches, each focusing on different aspects of the robot's development.

## Repository Structure

- **main**: This branch contains the functional versions of the robot.
- **ros2**: This branch contains ROS 2 packages and Docker images to run on both AMD and ARM architectures (Linux and Raspberry Pi).
- **freeCad**: This branch contains the design and structural components of the robot, created using FreeCAD. It includes both the .stl file and the freecad file.

## Branch Details

### main Branch

The main branch is reserved for the final, functional version of the quadruped robot, after merging and testing all the branches. 

### ros2 Branch

The ros2 branch contains the ROS 2 packages required for controlling the quadruped robot. This branch also includes Docker files for easy deployment on different architectures.

#### ROS 2 Package Instructions

1. **Clone the Repository**:
    ```sh
    git clone -b ros2_ws https://github.com/JavideuS/robotsito.git
    cd robotsito/robotsito_ws
    ```

2. **Build the Packages using colcon**:
    ```sh
    colcon build
    ```

3. **Source the Setup File**:
    ```sh
    source install/setup.bash
    ```

4. **Running the Docker Images**:
    - **For AMD Architecture**:
        ```sh
        docker pull javideus/robotsito
        docker run --platform linux/amd64 -it javideus/robotsito:v1.0.1
        ```
    - **For ARM Architecture (Raspberry Pi)**:
        ```sh
        docker pull javideus/robotsito
        docker run --platform linux/arm64 -it javideus/robotsito:v1.0.1
        ```

### freeCad Branch

The freeCad branch contains the design files for the quadruped robot. These files are organized into folders for easy navigation.

#### Structure

- **materials**: Contains the specific 3D designs for each component and support structures.
- **legs**: Contains the designs for the robot's legs.
- **main file**: A comprehensive file that simulates and joins all the structures and pieces of the robot.

#### Accessing the Designs

1. **Clone the Repository**:
    ```sh
    git clone -b freecad https://github.com/JavideuS/robotsito.git
    cd robotsito/freecad_models
    ```

2. **Navigate to the Desired Folder**:
    - For materials:
        ```sh
        cd Materials
        ```
    - For legs:
        ```sh
        cd Legs
        ```
    - For the main/robot file:
        ```sh
        open SpiderV1_3.2.FCStd
        ```
        
## Contributing

Contributions are welcome! If you have any ideas, suggestions, or issues, please open an issue or submit a pull request.

## License

This project is licensed under the GPL License. See the LICENSE file for more details.

### <3
