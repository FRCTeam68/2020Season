/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;

public class AirPump extends SubsystemBase {

  private Compressor compressor;
  private static AirPump airPump;
    
  public static AirPump getAir() {
    if (airPump == null) {
      airPump = new AirPump();
    }
    return airPump;
  }
  
  /**
   * Creates a new Compressor.
   * 
   * @param pcmMain
   */
  private AirPump() {
    //compressor = new Compressor(Constants.PCM_MAIN);  
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
