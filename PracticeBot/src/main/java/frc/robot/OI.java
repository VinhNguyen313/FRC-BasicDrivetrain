/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LiftIntake;
import frc.robot.commands.PIDPositionDrive;
import frc.robot.commands.drive.ToggleQuickTurn;
import frc.robot.util.Direction;
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

   public OI() {
      main.b.whenPressed(new ToggleQuickTurn());
      main.a.whenPressed(new PIDPositionDrive(100, .5, .5, .5));
      main.x.whenPressed(new LiftIntake(Direction.Down));
      main.y.whenPressed(new LiftIntake(Direction.Up));
   }

   public double getDriveValue() {
      return -(main.getTriggerAxis(Hand.kRight) - main.getTriggerAxis(Hand.kLeft));
   }

   public double getTurnValue() {
      return main.getX(Hand.kLeft);
   }

}
