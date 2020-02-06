/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Compressor extends SubsystemBase {
  /**
   * Creates a new Compressor.
   */

  private Boolean highOrLow; //false = low / true = high

  private DoubleSolenoid shiftGear;

  private static Compressor compressor;
    
  public static Compressor getCompressor() {
    if (compressor == null) {
      compressor = new Compressor();
    }
    return compressor;
  }

  public Compressor() {
    shiftGear = new DoubleSolenoid(Constants.DRIVE_SHIFTER_PCM_A, Constants.DRIVE_SHIFTER_PCM_B);
    setShiftLow();
    highOrLow = false; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setShifterHigh() {
    shiftGear.set(Value.kForward);
    highOrLow = true;
} 

  public void setShiftLow() {
    shiftGear.set(Value.kReverse);
    highOrLow = false;
}
  public DoubleSolenoid.Value getShifter() {
    return shiftGear.get();
  }
  public Boolean gearState(){
    return highOrLow;
  }
}
