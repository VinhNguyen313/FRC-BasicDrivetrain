/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.drive.PIDControllerDrive;
import frc.robot.commands.drive.PIDPositionDrive;
import frc.robot.util.VortxController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

   /*
    * RobotMap.Controller.MAIN is the ID of the controller, you can check this
    * using Driver Station
    */
   public VortxController main = new VortxController(RobotMap.Controller.MAIN);
   public VortxController co = new VortxController(RobotMap.Controller.CO);

   public OI() {
      co.b.whenPressed(new ToggleIntake());
      co.a.whenPressed(new PIDControllerDrive(150));
   }

   public double getShooterRollValue() {
      return -co.getY(Hand.kRight);
   }

   public double getShooterLiftValue() {
      return -co.getY(Hand.kLeft);
   }

   public double getDriveValue() {
      return -(main.getTriggerAxis(Hand.kRight) - main.getTriggerAxis(Hand.kLeft));
   }

   public double getTurnValue() {
      return main.getX(Hand.kLeft);
   }

}
