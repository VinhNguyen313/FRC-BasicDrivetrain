/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  
  private  XboxController main = new XboxController(RobotMap.Controller.MAIN);
  /* RobotMap.Controller.MAIN is the ID of the controller, 
   * you can check this using Driver Station 
   * */

  //since 'main' is private, we need a public method to access the values of the joysticks.
  public double getLeftYStick() {
     return main.getY(Hand.kLeft); 
  }
  public double getRightYStick() {
    return main.getY(Hand.kRight); 
  }
  
}


