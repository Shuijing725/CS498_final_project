# COVID Testing Robot
This repository contains the codes for our final project titled COVID Testing Robot for CS 498IR, taught by Kris Hauser in University of Illinois Urbana Champaign in Spring 2021. 
For experiment demonstrations, please refer to the [youtube video](https://www.youtube.com/watch?v=H_gwcFDu2kw).

<img src="/figures/setup.png" width="600" />

## Introduction
COVID-19 has been spreading for more than a year. Around 1.5M COVID tests are conducted daily in US.
However, current testing procedure requires lots of human labors and exposes medical workers in high risk.
In this project, we build a simulation pipeline for automatic COVID tests with two Kinova-gen3 robot arms:
- The left robot picks up a swab, uses the swab to collect testing samples from the human’s mouth, transfers the sample to a testing tube and then to a reaction plate, and disposes the swab into the trash can;
- The right robot picks up the reaction plate, and puts the reaction plate onto the PCR testing machine.


We use [Klampt](http://motion.cs.illinois.edu/software/klampt/latest/pyklampt_docs/) to solve robot configurations in each waypoint and plan paths for the robots.



## Installation
1. Install Python 3.x

2. Install Python packages. On Linux / Mac this can be as easy as using pip
```
python -m pip install klampt PyOpenGL numpy scipy scikit-learn matplotlib notebook
```
For Windows, you may need to download the install files from this repository.


## Run the code
1. Start the simulation:
```
python main.py 
```

2. After the simulator window pops up, press key 'p' to plan the left robot. After the plan trajectories (orange lines) show up, press 'e' to execute the plan.

3. Press 'p' and 'e' to do the same for the right robot.

(We only tested our code in Ubuntu 16.04 with Python 3.6.)


## Credits
A lot of the code is based on [CS 498IR MPs](https://github.com/krishauser/cs498ir_s2021).

Other collaborators:
- [Peixin Chang](https://github.com/PeixinC)
- [Shaoyu Meng](https://github.com/smeng9)