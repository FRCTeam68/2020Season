/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.IntakeManual;
*/
/*
public class Intake extends SubsystemBase {
 
   //Declare class variables
   private VictorSPX intakeMotor;
   private DigitalInput beamBreak;
   private static Intake intake;

   public static Intake getIntake() {
     if (intake == null) {
       intake = new Intake();
     }
     return intake;
   }
  public Intake() {
    intakeMotor = new VictorSPX(Constants.INTAKE_MOTOR); //Setting what motor this is associated with
    beamBreak = new DigitalInput(Constants.INTAKE_BEAM_BREAK); //set the port that this is on later

  }

   


  @Override
  public void periodic() {
    CommandScheduler.getInstance().setDefaultCommand(Robot.intake, new IntakeManual());
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speedA){

  }


public double getIntakeSpeed()
{
    return intakeMotor.getMotorOutputPercent();

}

}
*/