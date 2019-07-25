/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  /** We use nested classes as a way to organize constants. 
   * Examples:
   * - Right DriveTrain motor ID will be called by RobotMap.Drive.rightMotor
   * - Elevator motor ID will be called by RobotMap.Drive.rightMotor
   * */

    public class Drive{
      public static final int R1 = 3;
      public static final int R2 = 6;
      public static final int L1 = 7;
      public static final int L2 = 1;
    }

    public class Controller{
      public static final int MAIN = 0;
      public static final int RIGHT_Y_TRIGGER = 1;
      public static final int LEFT_Y_TRIGGER = 1;
    }
  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
