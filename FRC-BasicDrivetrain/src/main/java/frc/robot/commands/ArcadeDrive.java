/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ArcadeDrive extends Command {
  public ArcadeDrive() {
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
     /* copied from RobotDrive class. essentially lowers the speed of one motor first, 
      * rather than increases
      * one and decreases the other at the same time.
      * */

    double leftSpeed;
    double rightSpeed;

    double move = Robot.oi.getDriveValue();
    double rotate = Robot.oi.getTurnValue();
      
    if (move > 0.0) {
      if (rotate < 0.0) {
          leftSpeed = move + rotate;
          rightSpeed = Math.max(move, -rotate);
      } else {
          leftSpeed = Math.max(move, rotate);
          rightSpeed = move - rotate;
        }
    } else {
        if (rotate < 0.0) {
          leftSpeed = -Math.max(-move, -rotate);
          rightSpeed = move - rotate;
        } else {
          leftSpeed = move + rotate;
          rightSpeed = -Math.max(-move, rotate);
          }
    }
        
    Robot.drive.setLeftSpeed(leftSpeed);
    Robot.drive.setRightSpeed(rightSpeed);

    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
