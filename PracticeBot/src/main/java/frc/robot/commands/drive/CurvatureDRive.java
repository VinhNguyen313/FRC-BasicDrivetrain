/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.util.VortxMath;

public class CurvatureDRive extends Command {

  private double quickStopThreshold = .2;
  private double quickStopAccumulator = 0;
  private double quickStopAlpha = .1;

  public CurvatureDRive() {
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double move = VortxMath.applyDeadband(Robot.oi.getDriveValue(), .02);
    double rotate = VortxMath.applyDeadband(Robot.oi.getTurnValue(), .02);

    move = Math.copySign(move * move, move);
    rotate = Math.copySign(rotate * rotate, rotate);

    double angularPower;
    boolean overPower;

    boolean isQuickTurn = Robot.oi.getYToggle();

    if (isQuickTurn) {
      if (Math.abs(move) < quickStopThreshold) {
        quickStopAccumulator = (1 - quickStopAlpha) * quickStopAccumulator + quickStopAlpha * rotate * 2;
      }
      overPower = true;
      angularPower = rotate;
    } else {
      overPower = false;
      angularPower = Math.abs(move) * rotate - quickStopAccumulator;

      if (quickStopAccumulator > 1) {
        quickStopAccumulator -= 1;
      } else if (quickStopAccumulator < -1) {
        quickStopAccumulator += 1;
      } else {
        quickStopAccumulator = 0.0;
      }
    }

    double leftMotorOutput = move + angularPower;
    double rightMotorOutput = move - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) {
      if (leftMotorOutput > 1.0) {
        rightMotorOutput -= leftMotorOutput - 1.0;
        leftMotorOutput = 1.0;
      } else if (rightMotorOutput > 1.0) {
        leftMotorOutput -= rightMotorOutput - 1.0;
        rightMotorOutput = 1.0;
      } else if (leftMotorOutput < -1.0) {
        rightMotorOutput -= leftMotorOutput + 1.0;
        leftMotorOutput = -1.0;
      } else if (rightMotorOutput < -1.0) {
        leftMotorOutput -= rightMotorOutput + 1.0;
        rightMotorOutput = -1.0;
      }
    }

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    if (maxMagnitude > 1.0) {
      leftMotorOutput /= maxMagnitude;
      rightMotorOutput /= maxMagnitude;
    }

    SmartDashboard.putNumber("Left Speed", leftMotorOutput);
    Robot.drive.setRightSpeed(leftMotorOutput);

    SmartDashboard.putNumber("Right Speed", rightMotorOutput);
    Robot.drive.setLeftSpeed(rightMotorOutput);
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
