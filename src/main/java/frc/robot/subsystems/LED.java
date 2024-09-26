// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// originally from https://github.com/mavericks2252/2023RobotCodeNew.git

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer ledBufferOff;
  private final AddressableLEDBuffer ledBuffer;
  private final Timer ledTimerOff;
  private final Timer ledTimerOn;
  public LEDMode mode;
  
  public static enum LEDMode {
    ON, OFF, BLINK
  }

  /** Creates a new LED. */
  public LED() {
    
    //create led blink timers
    ledTimerOff = new Timer();
    ledTimerOn = new Timer();

    mode = LEDMode.ON;
    
    // initialize LED's LED's
    led = new AddressableLED(LEDConstants.kLEDStripPWMPort);
    ledBufferOff = new AddressableLEDBuffer(LEDConstants.kLEDStripLength);
    ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDStripLength);

    led.setLength(ledBuffer.getLength());
    
    // create off and on buffers. default on buffer to orange.
    for (var i = 0; i < ledBufferOff.getLength(); i++){
      ledBufferOff.setLED(i, Color.kBlack);// off
    }    
    for (var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setLED(i, Color.kOrange);
    }
    
    // defualt to orange
    led.setData(ledBuffer);
    led.start();
   
    //led blinking timers
    ledTimerOn.start();
    ledTimerOff.reset();
    ledTimerOn.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    switch ( mode ) {
        case ON:
            on();
            break;

        case OFF:
            off();
            break;

        case BLINK:
            if (ledTimerOn.get() >= LEDConstants.kLEDBlinkOnTime) {
                ledTimerOn.reset();
                ledTimerOn.stop();      
                ledTimerOff.start();
                on();        
            }
            else if (ledTimerOff.get() >= LEDConstants.kLEDBlinkOffTime) {
                ledTimerOff.reset();
                ledTimerOff.stop();
                ledTimerOn.start();
                off();
            }
        }
  }

  // turn LED's off
  public void off(){
    mode = LEDMode.OFF;
  }

  // turn LED's on
  public void on(){
    mode = LEDMode.ON;
  }

  public void blink() {
    mode = LEDMode.BLINK;
  }

  public void setColor( Color color ) {
    for (var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setLED(i, color);
    }
    led.setData(ledBuffer);
  }

}