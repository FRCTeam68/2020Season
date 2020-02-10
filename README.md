# 2020Season
Projects for the 2020 season

THIS BRANCH WE SHOULD ALL UPLOAD THE CODE TO PLEASE IF YOU CREATE 
A NEW SUBSYSTEM SUCCSESSFULY PLEASE MERGE IT INTO THIS BRANCH\


To upload Robot Code first install FileZilla from https://filezilla-project.org/download.php?type=client
After installation connect to the robot via USB Cable
Run FileZilla Type in roboRIO-68-frc.local for Host: for the username type in lvuser :KEEP PASSWORD BLANK IF IT PROMTS YOU DONT PUT ONE IN: and type in 22 for the port and quick connect
After connection find home/lvuser/deploy/paths
Drag the left and right CSV files into the paths directory and disconnect the cable from the robot

In VS Code navigate to the Robot.java and go to autonomousInit and where we insantiate a new autonTraj change the string from "/stleft.csv" to "/filename.csv" ALWAYS KEEP THE /
To tune pid navigate to the PathFollower subsystem and change the double P double I and double D numbers to calculate a more accurate auton :-)
-Connor 68


NOTICE
Auton
-If robot do not turn enough change Robot Base in Bob traectory
increase = turn more decrease = turn less (Changes jerk)
-NEVER CALL FOR NAVX RESET ONLY RESET YAW
-RESTART ROBOT CODE BETWEEN AUTONS
-SET SENSOR PHASE CHANGES WHICH WAY THE ENCODER READS

