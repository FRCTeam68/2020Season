/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

/**
    I have chosen to create a class for Following a path to make it easier to adjust just path following
 */
public class PathFollower{

    /*
    / Upload the Stinky.path file for the robot to run!!!!!!!!!!!!!!
    */

    private static final int k_ticks_per_rev = 1024;
    private static final double k_wheel_diameter = 4.0 / 12.0;
    private static final double k_max_velocity = 10;
  
   // private static final int k_gyro_port = m_gyro.class.; // use m_gyro  
    

    private WPI_TalonSRX m_left_motor;
    private WPI_TalonSRX m_right_motor;

    private Encoder m_left_encoder;
    private Encoder m_right_encoder;

    private AHRS m_gyro;


    private EncoderFollower m_left_follower;
    private EncoderFollower m_right_follower;

    private Notifier m_follower_notifier;


    //PIDController turnController;
    double rotateToAngleRate; //this is the output
    double setPoint = 0;
    double last_world_linear_accel_x;
    double last_world_linear_accel_y;
    static double kCollisionThreshold_DeltaG = 0.8f; 
    static final double kTargetAngleDegrees = 90.0f;

    public static PathFollower pathFollower;

    public static PathFollower getPathFollower() {
      if (pathFollower == null) {
        pathFollower = new PathFollower();
      }
      return pathFollower;
    }
    public void autoSetVarsInit() {
        m_gyro = new AHRS(SPI.Port.kMXP);
        m_left_motor = new WPI_TalonSRX(Constants.FALCON_FL);
        m_right_motor = new WPI_TalonSRX(Constants.FALCON_FR);
        m_left_encoder = new Encoder(Constants.FALCON_FL, Constants.FALCON_BL);
        m_right_encoder = new Encoder(Constants.FALCON_FR,Constants.FALCON_BR);
      }

  public double getYaw() {
    return -m_gyro.getYaw();
  }

  public boolean isCalibrated() {
    return m_gyro.isCalibrating();
  }

  public double getPidOutput() {
    return rotateToAngleRate;
  }

  public double getRoll() {
    return m_gyro.getRoll();
  }

  public double getAngle() {
    return m_gyro.getAngle();
  }

  public boolean gyroActiveCheck() {
    return m_gyro.isConnected();
  }

  public void autonomousInit(String k_path_name) {

    k_path_name = "";
    
    try {
      final Trajectory left_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".left");
      final Trajectory right_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".right");

      m_left_follower = new EncoderFollower(left_trajectory);
      m_right_follower = new EncoderFollower(right_trajectory);

      m_left_follower.configureEncoder(m_left_encoder.get(), k_ticks_per_rev, k_wheel_diameter);
      // You must tune the PID values on the following line!
      m_left_follower.configurePIDVA(Constants.PID_P, Constants.PID_I, Constants.PID_D, 1 / k_max_velocity, 0);

      m_right_follower.configureEncoder(m_right_encoder.get(), k_ticks_per_rev, k_wheel_diameter);
      // You must tune the PID values on the following line!
      m_right_follower.configurePIDVA(Constants.PID_P, Constants.PID_I, Constants.PID_D, 1 / k_max_velocity, 0);

      m_follower_notifier = new Notifier(this::followPath);
      m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
    } catch (final IOException e) {
      e.printStackTrace();
    }
  }

  private void followPath() {
    if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
      m_follower_notifier.stop();
    } else {
      final double left_speed = m_left_follower.calculate(m_left_encoder.get());
      final double right_speed = m_right_follower.calculate(m_right_encoder.get());
      final double heading = m_gyro.getAngle();
      final double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
      final double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      final double turn = 0.8 * (-1.0 / 80.0) * heading_difference;
          m_left_motor.set(left_speed + turn);
          m_right_motor.set(right_speed - turn);
        }
      }
      
}