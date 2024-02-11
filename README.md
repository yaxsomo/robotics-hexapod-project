# robotics-hexapod-project
This repository is dedicated to the final robotics evaluation course

## Project Setup

##### Cloning the repository
In order to make this project work on your pc, you will first have to clone this repository with the following command : 

```bash
git clone https://github.com/isaiah-dev/HexapodeV_18.git
```

##### Installing the dependencies

After cloning the repository, you will have to install the relative dependencies. We've made the work for you, by creating the 'requirements.txt' file. Go to the repository folder and use the following command :

```bash
pip install -r requirements.txt
```

## Launching the software

Once everything above is setted up correctly, you can now launch the software with the following command :

```bash
python3 main.py
```


## CLI Interface - Menus

##### Main Menu

The Main menu is structured as follows : 

```bash
- Select Software Mode : -
1. Robot
2. Simulation
3. Exit
Choice : 
```

##### Behavior Selection Menu

You can choose the simulation mode, or the real robot control mode.
Once you've selected the mode, you will be prompted to a behavior selection menu :

```bash
- Behavior Mode Selection -
Select Behaviour :
1. Move leg to position
2. Move robot center to position
3. Walk in a straight line
4. Rotate robot
5. Exit
Choice : 
```

## CLI Interface - Behaviors


##### 1 - Move leg to position

With this behavior, you can choose an arbitrary x, y, z position and select the leg that will be enabled. Here's how it presents itself :

```bash
Enter desired X: 
Enter desired Y: 
Enter desired Z: 
Enter desired duration: 
Enter desired Leg ID: 
```

##### 2 - Move robot center to position

With this behavior, you can balance the robot to different axis. You can type one of those balance modes :

```bash
tilt-x | tilt-y | tilt-z | tilt-xy 
```

##### 3 - Walk in a straight line

With this behavior, you can make the robot walk in a straight line. You will have the opportunity to enter the orientation of the robot.

```bash
Enter desired walk direction |in fractions of pi|: 
```

##### 4 - Rotate robot

With this behavior, you can make the robot rotate 



### Members of the project

Yassine DEHHANI, Emile BAILEY & Alexis TORBIO


