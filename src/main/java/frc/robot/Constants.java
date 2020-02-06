/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Controller Inputs

    public static final int XBOX_DRIVE = 0;
        
    public static final int XBOX_DRIVE_X = 2;
    public static final int XBOX_DRIVE_CIRCLE = 3;
    public static final int XBOX_DRIVE_SQUARE = 1;
    public static final int XBOX_DRIVE_TRIANGLE = 4;
    public static final int XBOX_DRIVE_LY = 1;			// left joystick
    public static final int XBOX_DRIVE_LT = 3;
    public static final int XBOX_DRIVE_LT_BUTTON = 7;
    public static final int XBOX_DRIVE_RT = 4;
    public static final int XBOX_DRIVE_RT_BUTTON = 8;
    public static final int XBOX_DRIVE_RY = 5;			// right joystick
    public static final int XBOX_DRIVE_SL = 11 ;
    public static final int XBOX_DRIVE_SR = 12 ;
    public static final int XBOX_DRIVE_RB = 6;
    public static final int XBOX_DRIVE_LB = 5;
    public static final int XBOX_DRIVE_SHARE = 9;
	public static final int XBOX_DRIVE_OPTIONS = 10;
    public static final int XBOX_DRIVE_POV_DOWN = 180;
    public static final int XBOX_DRIVE_POV_RIGHT = 90;
    public static final int XBOX_DRIVE_POV_LEFT = 270;
    public static final int XBOX_DRIVE_POV_UP = 0;

    public static final int XBOX_MANIPULATE = 1;

    public static final int XBOX_MANIPULATE_X = 2;
    public static final int XBOX_MANIPULATE_CIRCLE = 3;
    public static final int XBOX_MANIPULATE_SQUARE = 1;
    public static final int XBOX_MANIPULATE_TRIANGLE = 4;
    public static final int XBOX_MANIPULATE_LY = 1;			// left joystick
    public static final int XBOX_MANIPULATE_LT = 7;
    public static final int XBOX_MANIPULATE_RT = 8;
    public static final int XBOX_MANIPULATE_RY = 5;			// right joystick
    public static final int XBOX_MANIPULATE_SL = 11;
    public static final int XBOX_MANIPULATE_SR = 12;
    public static final int XBOX_MANIPULATE_RB = 6;
    public static final int XBOX_MANIPULATE_LB = 5;
    public static final int XBOX_MANIPULATE_SHARE = 9;
	public static final int XBOX_MANIPULATE_OPTIONS = 10;
    public static final int XBOX_MANIPULATE_POV_DOWN = 180;
    public static final int XBOX_MANIPULATE_POV_RIGHT = 90;
    public static final int XBOX_MANIPULATE_POV_LEFT = 270;
    public static final int XBOX_MANIPULATE_POV_UP = 0;
    
    // pid values provided by the almight An
    public static final double PID_F = 0.03; //0.025
    public static final double PID_P = .4;
    public static final double PID_I = 0.0;
    public static final double PID_D = 0.0;

    //drive motor values

    public static final int FALCON_FR = 1;
    public static final int FALCON_FL = 2;
    public static final int FALCON_BR = 3;
    public static final int FALCON_BL = 4;
    

    // Drivetrain Left PID Config
    public static final int DRIVETRAIN_LEFT_PID_SLOT = 0;
    public static final double DRIVETRAIN_LEFT_PID_F = 0.507;
    public static final double  DRIVETRAIN_LEFT_PID_P = 1;
    public static final double DRIVETRAIN_LEFT_PID_I = 0;
    public static final double DRIVETRAIN_LEFT_PID_D = 0;
    // Drivetrain Right PID Config
    public static final int DRIVETRAIN_RIGHT_PID_SLOT = 0;
    public static final double DRIVETRAIN_RIGHT_PID_F = 0.507;
    public static final double DRIVETRAIN_RIGHT_PID_P = 1;
    public static final double DRIVETRAIN_RIGHT_PID_I = 0;
    public static final double DRIVETRAIN_RIGHT_PID_D = 0;

    public static final double DRIVETRAIN_RIGHT_SLOT = 0;
    public static final double DRIVETRAIN_LEFT_SLOT = 0;

    //auton
    public static final double WHEEL_DIAMETER = 7;
    public static final double MAX_SPEED = 7;
    public static final int ENCODER_TICK_LEFT_REVOLUTION = 24500; //-24427
    public static final int ENCODER_TICK_RIGHT_REVOLUTION = 24500; //24508
    public static final double AUTON_DISTANCE_KP = 2.8; //.08
    public static final double AUTON_DISTANCE_KI = 0;
    public static final double AUTON_DISTANCE_KD = .4;
    public static final double AUTON_ANGLE_KP = 4;//SET THESE VALUES 
    public static final double AUTON_ANGLE_KI = 0;
    public static final double AUTON_ANGLE_KD = 1.6;
    public static final double AUTON_PI_OFFSET = -0.2; //TLDR this will make robot turn more or less 
    /*
    Only change PI offset if the 
    Robot doesnt match 6.3 Rads on a full 360 Degree turn
    This is very experimental
    Set to negative if the robot doesnt turn enough
    Set to positive if the robot turns too much
     */
}
 