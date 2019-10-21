/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Shooter.ShooterLift;

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {
  private WPI_TalonSRX left;
  private WPI_TalonSRX right;

  private WPI_TalonSRX lift;

  public Shooter() {
    left = new WPI_TalonSRX(RobotMap.Shooter.left);
    right = new WPI_TalonSRX(RobotMap.Shooter.right);

    right.follow(left);
    
    lift = new WPI_TalonSRX(RobotMap.Shooter.lift);

  }

  public void setRollSpeed(double speed) {
    left.set(ControlMode.PercentOutput, speed);
  }

  public void setLiftSpeed(double speed) {
    lift.set(ControlMode.PercentOutput, speed*.5);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ShooterLift());
  }
}
