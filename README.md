# 2020Season
Projects for the 2020 season
<<<<<<< HEAD
Although the 2020Vision is under the 2020 season repo
we are using the 2019 robot to practice vision on
We are using the parade robot to practice Auton
using the NAVX

The code we have under this repository is to start
using the new working area that WPILIB has provided
for us.
=======


To upload Robot Code first install FileZilla from https://filezilla-project.org/download.php?type=client
After installation connect to the robot via USB Cable
Run FileZilla Type in roboRIO-68-frc.local for Host: for the username type in lvuser :KEEP PASSWORD BLANK IF IT PROMTS YOU DONT PUT ONE IN: and type in 22 for the port and quick connect
After connection find home/lvuser/deploy/paths
Drag the left and right CSV files into the paths directory and disconnect the cable from the robot

In VS Code navigate to the Robot.java and go to autonomousInit and where we insantiate a new autonTraj change the string from "/stleft.csv" to "/filename.csv" ALWAYS KEEP THE /
To tune pid navigate to the PathFollower subsystem and change the double P double I and double D numbers to calculate a more accurate auton :-)
-Connor 68
>>>>>>> 2020HelixTest
