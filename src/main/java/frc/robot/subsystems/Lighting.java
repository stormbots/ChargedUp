// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.stormbots.devices.BlinkenPattern;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase {
  Spark leds = new Spark(0);
  public enum LedPattern{
    BLUE,RED,
    NEED_CONE,HAVE_CONE,
    NEED_CUBE,HAVE_CUBE,
    POSITION,EXECUTE
    // Note to Dan: 
    // - I was told that NEED_CUBE and NEED_CONE aren't needed 
    // - READY will be the team alliance color
  }
  private LedPattern pattern = LedPattern.BLUE;

  /** Creates a new Lighting. */
  public Lighting() {
  }

  public void setColor(LedPattern newPattern){
    this.pattern = newPattern;
  }

  @Override
  public void periodic() {
    // leds.set(BlinkenPattern.BLINKING_RAINBOW.pwm()); // Test function
    BlinkenPattern newpattern = BlinkenPattern.CODE_BLUE;
    // This method will be called once per scheduler run
    switch(pattern){
      case BLUE: 
        newpattern = BlinkenPattern.SOLID_LIGHT_BLUE; 
        break;
      case RED: 
        newpattern = BlinkenPattern.SOLID_ORANGE; 
      break;
      case NEED_CONE: 
      case HAVE_CONE:
        newpattern = BlinkenPattern.SOLID_GOLD; 
        break;
      case NEED_CUBE: 
      case HAVE_CUBE: 
        newpattern = BlinkenPattern.SOLID_VIOLET;
        break;
      default:
    }
    leds.set(newpattern.pwm());
  }
}
