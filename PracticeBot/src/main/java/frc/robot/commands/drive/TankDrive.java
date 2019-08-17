/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.util.VortxMath;

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
      /*
     * These 2 variables store the scaled value (after applyDeadband) of the sticks
     * on the controller. it's basically getting the input value from the 'main'
     * controller of the oi object in Robot.java.
     */
    double rightSpeed = VortxMath.applyDeadband(-Robot.oi.main.getY(Hand.kRight), .02);
    double leftSpeed = VortxMath.applyDeadband(-Robot.oi.main.getY(Hand.kLeft), .02);

    // We use the square of input from the controller to make driving smoother at
    // low speeds.
    // And since the input is squared, we need Math.copySign to preserve the sign of
    // the inputs.
    rightSpeed = Math.copySign(rightSpeed * rightSpeed, rightSpeed);
    leftSpeed = Math.copySign(leftSpeed * leftSpeed, leftSpeed);

    SmartDashboard.putNumber("Right Speed", rightSpeed);
    Robot.drive.setRightSpeed(rightSpeed);

    SmartDashboard.putNumber("Left Speed", leftSpeed);
    Robot.drive.setLeftSpeed(leftSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.setLeftSpeed(0);
    Robot.drive.setRightSpeed(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.drive.setLeftSpeed(0);
    Robot.drive.setRightSpeed(0);
  }

}
