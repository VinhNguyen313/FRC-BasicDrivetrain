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

public class PIDPositionDrive extends Command {
  public double kP, kI, kD, inches;

  public PIDPositionDrive(double inches, double kP, double kI, double kD) {
    requires(Robot.drive);
    this.inches = inches;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drive.configPID(kP, kI, kD);
    Robot.drive.setStraightPosition(inches);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double leftError = Math
        .abs(Robot.drive.getCurrentPosition(Hand.kLeft) - Robot.drive.getClosedLoopTarget(Hand.kLeft));
    double rightError = Math
        .abs(Robot.drive.getCurrentPosition(Hand.kRight) - Robot.drive.getClosedLoopTarget(Hand.kRight));
    if (leftError < 500 && rightError < 500)
      return true;
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
  }
}
