/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

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
    //these 2 variables store the value of the sticks on the controller.
    double rightSpeed = Robot.oi.main.getY(Hand.kRight);
    double leftSpeed = Robot.oi.main.getY(Hand.kLeft);
    
    //We use the square of input from the controller to make driving smoother at low speeds.
    //And since the input is squared, we need Math.copySign to preserve the sign of the inputs.
    Robot.drive.setRightSpeed(Math.copySign(rightSpeed*rightSpeed, rightSpeed));
    Robot.drive.setLeftSpeed(Math.copySign(leftSpeed*leftSpeed, leftSpeed));
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
