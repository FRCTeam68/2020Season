/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class AddressableLEDs extends SubsystemBase {
  /**
   * Creates a new AddressableLED.
   */  

  AddressableLED m_led0;
  AddressableLED m_led1;
  AddressableLED m_led2;
  AddressableLEDBuffer m_ledBuffer ;
  int m_rainbowFirstPixelHue;
  

  public static AddressableLEDs addressableLED;

	public static AddressableLEDs getAddressableLED() {
		if (addressableLED == null) {
			addressableLED = new AddressableLEDs();
		}
		return addressableLED;
	}

  public AddressableLEDs() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led0 = new AddressableLED(0);
    m_led1 = new AddressableLED(1);
    m_led2 = new AddressableLED(2);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led0.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led0.start();
    rainbow();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void rainbow() {
    // For every pixel

    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength()) % 180);
      // Set the value
      m_ledBuffer.setHSV(i, (int) hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;

    m_led0.setData(m_ledBuffer);
  }  

}
