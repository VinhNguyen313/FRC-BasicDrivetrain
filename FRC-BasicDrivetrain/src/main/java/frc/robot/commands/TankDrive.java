/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class TankDrive extends Command {
  public TankDrive() {
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double rightSpeed = Robot.oi.getStickValue(RobotMap.Controller.RIGHT_Y_TRIGGER);
    double leftSpeed = Robot.oi.getStickValue(RobotMap.Controller.LEFT_Y_TRIGGER);
    
    Robot.drive.setRightSpeed(Math.copySign(rightSpeed*rightSpeed,rightSpeed));
    Robot.drive.setLeftSpeed(Math.copySign(leftSpeed*rightSpeed,leftSpeed));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // We never want the robot to stop moving, thus this method always returns false.
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
