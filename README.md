# image_cropping
This is a ROS project for image cropping. There are two branches, both have same functionality but their implementation differs. For the cropping, four values are required, `x_pix`, `y_pix`, `x_width`, `y_width`. (`x_pix`,`y_pix`) is the top let image pixel from where cropping starts. `x_width` and `y_width` are the dimensions of the final image.

## Branch-A
There are two nodes - `client_node` and `server_node`. 
The `client_node` has a subscriber (subscribing from `/image`) and a ROS service client (for `/image_cropping_service`). User inputs `x_pix` and `y_pix` via commandline (roslaunch arg). Client requests for the servie, subsciber prints the incoming image message. 
The `server_node` has a publisher (publishing at `/image`) and a ROS service server (`/image_cropping_service`). This node has access to the image, and takes in `x_width` and `y_width` via rosparam. Publisher, publishes the current image.

## Branch-B
There are two nodes - `client_node` and `server_node`. 
`client_node` is an action client.  User inputs `x_pix`,`y_pix`,`x_width`,`y_width` via commandline (roslaunch arg). 
`server_node` is an action server. Takes in request from the action client and crops the image in steps, reducing 100x100 image in each step.

### Build
Create a catkin workspace.
```
mkdir catkin_ws/src
cd catkin_ws
catkin build
```
Clone this repository in the `src` folder.
```
cd src
git clone https://github.com/akshatpandya/image_cropping.git
```
Build the workspace, from `catkin_ws`.
```
cd ..
catkin build image_cropping
```
### Launch
Launch the `roslaunch` file.
```
roslaunch image_cropping image_cropping.launch
```
