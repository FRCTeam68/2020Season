# 2020Season
Projects for the 2020 season
To upload Robot Code first install FileZilla from https://filezilla-project.org/download.php?type=client
After installation connect to the robot via USB Cable
Run FileZilla Type in roboRIO-68-frc.local for Host: for the username type in lvuser :KEEP PASSWORD BLANK: and port 22 and quick connect
After connection find home/lvuser/deploy/paths
Drag the left and right CSV files in

In VS Code navigate to the Robot.java and go to autonomousInit and where we insantiate a new autonTraj change the string from "/stleft.csv" to "/filename.csv" ALWAYS KEEP THE /
To tune pid navigate to the PathFollower subsystem and change the double P double I and double D numbers to calculate a more accurate auton :-)
-Connor 68
