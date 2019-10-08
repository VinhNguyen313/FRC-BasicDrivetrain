/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.util.Direction;
import frc.robot.util.VortxMath;

public class ShooterLift extends Command {
  Direction dir;

  public ShooterLift() {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double liftSpeed = VortxMath.applyDeadband(-Robot.oi.getShooterLiftValue(), .02);
    double rollSpeed = VortxMath.applyDeadband(Robot.oi.getShooterRollValue(), .015);

    Robot.shooter.setLiftSpeed(liftSpeed);
    Robot.shooter.setRollSpeed(rollSpeed);
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
