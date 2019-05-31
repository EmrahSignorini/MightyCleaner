# Robotics Final Project - MightyCleaner

  - The following code contains the implementation of the Robotics final project.
  - The code, to be able to run, needs the `ros-aseba` and `thymioid` repository, a detailed explanation on how to clone them can be found [here](https://github.com/romarcg/MightyCleaner/tree/master/assignment#install-gazebo-myt-model-and-plugins). Make **sure** to do the said steps first, otherwise there will be errors.
  - You will also need the markers, which can be found [here](http://wiki.ros.org/ar_track_alvar?action=AttachFile&do=view&target=markers9to17.png) (the ones relative to the trash) and [here](http://wiki.ros.org/ar_track_alvar?action=AttachFile&do=view&target=markers0to8.png), the latter you need just the center marker since it corresponds to the trashcan
  - We suppose that you have calibrated the motors of the wheels, if you haven't probably there will be some incoherence with the velocity of the robot 
  - If you have not already, please install the `ar_track_alvar` ROS package, instructions on how to do it can be found [here](http://wiki.ros.org/ar_track_alvar), the installation guide supposes that you have the `indigo` distribution, if you have a different one, just replace `indigo` with your distribution  
## How to run
  - First thing that you have to do is:
  ```bash
    cd <catkin_ws>/src
    git clone https://github.com/EmrahSignorini/MightyCleaner.git
```
  - We have made a bash script called `run.sh` that will run the program for you. Before explaining how to run it, it is important for you to know that if you are using `catkin_make` to build the packages, then you should use the `-m` option so that the packages will be built using the `catkin_make` command. To recap:
    -   If you are using `catkin build` what you have to do is to execute the following command `./src/MightyCleaner/run.sh` from inside your `<catkin_workspace>` folder.
    -   Instead, if you are using `catkin_make` then the command is:
             `./src/MightyCleaner/run.sh -m`
## Option of the script
The script takes 2 options:
 - `-m`: &nbsp;&nbsp; As explained above use this option only if you are using `catkin_make` to build the package
 - `-r`: &nbsp;&nbsp; Robot name, the default value is `thymio17`, if you are using another robot use  `-r <robot_name>`, so the program would look like this in the case I want to name my robot `thymio0`:
    <br>In case you are running it from your catkin workspace folder
    ```bash
    ~/catkin_ws $ ./src/MightyCleaner/run.sh -r thymio0
    ```
    In case you are running it from the `MightyCleaner` folder
    ```bash
    ~/catkin_ws/src/MightyCleaner $ ./run.sh -r thymio0
    ```
 - `-s`: &nbsp;&nbsp; This option is relative to the markers to search for. Default value is 2, if you want to change it, just run: 
    ```bash
    ~/catkin_ws $ ./src/MightyCleaner/run.sh -s 3
    ```
    In case you are running it from the `MightyCleaner` folder
    ```bash
    ~/catkin_ws/src/MightyCleaner $ ./run.sh -s 3
    ```
 - `-t`: &nbsp;&nbsp; This option is initial task of the robot, if set to true the robot will rotate initially.
    ```bash
    ~/catkin_ws $ ./src/MightyCleaner/run.sh -t true
    ```
    In case you are running it from the `MightyCleaner` folder
    ```bash
    ~/catkin_ws/src/MightyCleaner $ ./run.sh -t true
    ```
#### Running the program
##### catkin build
In case you are running it from your catkin workspace folder
```bash
~/catkin_ws $ ./src/MightyCleaner/run.sh
```
In case you are running it from the `MightyCleaner` folder
```bash
~/catkin_ws/src/MightyCleaner $ ./run.sh
```
##### catkin_make
In case you are running it from your catkin workspace folder
```bash
~/catkin_ws $ ./src/MightyCleaner/run.sh -m
```
In case you are running it from the `MightyCleaner` folder
```bash
~/catkin_ws/src/MightyCleaner $ ./run.sh -m
```
Once launched the program, wait about 10 seconds so that everything is set up, once you are good to go, put in front of the camera a marker that refers to the trash and wait until the robot adjust himself. Once he is done you will see a message similar to:
```
    Switching states
    Looking for trashcan
```
Now what you have to do is put in front of the camera the marker corresponding to trashcan. Once done, if there are more trash markers to scan, you will see something like:
```
    Switching states
    Looking for more markers
```

Otherwise you will see something like:
```bash
    No more markers to see, shutting down.....
```
If there are no more markers to search for.