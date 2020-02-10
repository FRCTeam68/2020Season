/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Servo;
public class EndGame extends SubsystemBase {
  /**
   * Creates a new EndGame.
   */
  private WPI_TalonSRX liftWinch1;
  private WPI_TalonSRX liftWinch2;
  private Servo releaseEndGame;

  public EndGame() {
    liftWinch1 = new WPI_TalonSRX(Constants.ENDGAME_WINCH_1);
    liftWinch1 = new WPI_TalonSRX(Constants.ENDGAME_WINCH_2);
    releaseEndGame = new Servo(Constants.ENDGAME_SERVO);

    liftWinch1.setSensorPhase(false);
    liftWinch2.setSensorPhase(false);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
