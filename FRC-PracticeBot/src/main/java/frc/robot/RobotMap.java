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

  /**
   * We use nested classes as a way to organize these constants. Examples: - Right
   * DriveTrain motor ID will be called by RobotMap.Drive.rightMotor - Elevator
   * motor ID will be called by RobotMap.Drive.rightMotor
   */
  public class Drive {
    // These values can be found by looking at the wiring of your actual robot.
    public static final int R1 = 9;
    public static final int R2 = 8;
    public static final int L1 = 0;
    public static final int L2 = 1;
  }

  public class Controller {
    // These values can be found using your Driver Station.
    public static final int MAIN = 0;

  }

}
