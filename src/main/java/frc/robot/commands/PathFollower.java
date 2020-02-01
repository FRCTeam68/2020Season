/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.team2363.commands.HelixFollower;
import frc.team2363.controller.PIDController;

import com.team319.trajectory.Path;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import frc.robot.Robot;

public class PathFollower extends HelixFollower  {


    // These are the 2 PID controllers that will handle error for your total travel distance and heading

    

    private PIDController headingController = new PIDController(Constants.AUTON_ANGLE_KP, Constants.AUTON_ANGLE_KI, Constants.AUTON_ANGLE_KD, 0.001);
    private PIDController distanceController = new PIDController(Constants.AUTON_DISTANCE_KP, Constants.AUTON_DISTANCE_KI, Constants.AUTON_DISTANCE_KD, 0.001);

    public PathFollower(Path path, DriveTrain drivetrain) {
        super(path);
        System.out.println("STARTING TRAJECTORY");
        // Make sure to require your subsystem so you don't have conflicting commands
        addRequirements(drivetrain);
    }

	@Override
    public void resetDistance() {
        // We need to reset the encoders back to 0 at the start of the path
        Robot.driveTrain.ResetEncoders();
    }

    @Override
    public PIDController getHeadingController() {
        // Here we return the PID controller that we're using to correct the heading error through the path
        return headingController;
    }

    @Override
    public PIDController getDistanceController() {
        // Here we return the PID controller that we're using to correct the distance error through the path
        return distanceController;
    }

    @Override
    public double getCurrentDistance() {
        // Here we need to return the overall robot distance traveled in FEET in this example we are averaging 
        // the two sides of the DriveTrain to give is the robot's distance travelled
        return (Robot.driveTrain.getLeftPosition() + Robot.driveTrain.getRightPosition()) / 2.0;
        //create these methods that gets the positions in feet
    }

    @Override
    public double getCurrentHeading() {
        // Here we need to return the current heading of the robot in RADIANS (positive counter-clockwise).
        return Math.toRadians(Robot.driveTrain.getYAW());
    }

    @Override
    public void useOutputs(double left, double right) {
        // Here we will use the provided parameters in FPS and send them off to our DriveTrain. In this example 
        // the max velocity of our DriveTrain is 12 FPS. We are dividing the two provided parameters by the max 
        // veocity to convert them into a percentage and sending them off to our DriveTrain.
        SmartDashboard.putNumber("leftSpeed", left);
        SmartDashboard.putNumber("right speed", right);
        Robot.driveTrain.setSpeedAuto(left, right);
        if(isFinished()){
        Robot.driveTrain.setSpeedAuto(0, 0);
        }
    }

}