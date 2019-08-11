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

public class DriveCommand extends Command {
  public DriveCommand() {
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    ArcadeDrive();
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

  protected void ArcadeDrive() {
    /*
     * Alrgorithm from WPI's DifferentialDrive class. essentially lowers the speed
     * of one motor first, rather than increases one and decreases the other at the
     * same time.
     */
    double leftSpeed;
    double rightSpeed;

    double move = applyDeadband(Robot.oi.getDriveValue(), .02);
    double rotate = applyDeadband(Robot.oi.getTurnValue(), .02);

    move = Math.copySign(move * move, move);
    rotate = Math.copySign(rotate * rotate, rotate);

    double maxInput = Math.copySign(Math.max(Math.abs(move), Math.abs(rotate)), move);

    if (move >= 0.0) {
      if (rotate >= 0.0) {
        leftSpeed = maxInput;
        rightSpeed = move - rotate;
      } else {
        leftSpeed = move + rotate;
        rightSpeed = maxInput;
      }
    } else {
      if (rotate >= 0.0) {
        leftSpeed = move + rotate;
        rightSpeed = maxInput;
      } else {
        leftSpeed = maxInput;
        rightSpeed = move - rotate;
      }
    }

    Robot.drive.setLeftSpeed(leftSpeed);
    Robot.drive.setRightSpeed(rightSpeed);
  }

  protected void CurvatureDrive() {
    double move = applyDeadband(Robot.oi.getDriveValue(), .02);
    double rotate = applyDeadband(Robot.oi.getTurnValue(), .02);

    move = Math.copySign(move * move, move);
    rotate = Math.copySign(rotate * rotate, rotate);

    double angularPower;
    boolean overPower;

    double quickStopThreshold = .2;
    double quickStopAccumulator = 0;
    double quickStopAlpha = .1;

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

    Robot.drive.setRightSpeed(leftMotorOutput);
    Robot.drive.setLeftSpeed(rightMotorOutput);
  }

  protected void TankDrive() {
    /*
     * These 2 variables store the scaled value (after applyDeadband) of the sticks
     * on the controller. it's basically getting the input value from the 'main'
     * controller of the oi object in Robot.java.
     */
    double rightSpeed = applyDeadband(-Robot.oi.main.getY(Hand.kRight), .02);
    double leftSpeed = applyDeadband(-Robot.oi.main.getY(Hand.kLeft), .02);

    // We use the square of input from the controller to make driving smoother at
    // low speeds.
    // And since the input is squared, we need Math.copySign to preserve the sign of
    // the inputs.
    Robot.drive.setRightSpeed(Math.copySign(rightSpeed * rightSpeed, rightSpeed));
    Robot.drive.setLeftSpeed(Math.copySign(leftSpeed * leftSpeed, leftSpeed));
  }

  // Copied from WPI's RobotBase
  /**
   * Returns 0.0 if the given value is within the specified range around zero. The
   * remaining range between the deadband and 1.0 is scaled from 0.0 to 1.0.
   *
   * @param value    value to clip
   * @param deadband range around zero
   */
  protected double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

}
