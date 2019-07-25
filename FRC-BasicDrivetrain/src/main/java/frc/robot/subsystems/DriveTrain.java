/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.TankDrive;

/**
 * This substystem is used to model the characteristics and capabilities of the drive train of the robot.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Must have CTRE libraries installed for these to work.
  TalonSRX l1;
  TalonSRX r1;
  TalonSRX l2;
  TalonSRX r2;

  public DriveTrain(){
    l1 = new TalonSRX(RobotMap.Drive.L1);
    l2 = new TalonSRX(RobotMap.Drive.L2);
    r1 = new TalonSRX(RobotMap.Drive.R1);
    r2 = new TalonSRX(RobotMap.Drive.R2);

    l1.setInverted(true);
    l2.setInverted(true);

    // These 2 lines make sure the motors on the same side are spinning at the same speed.
    l2.follow(l1);
    r2.follow(r1);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TankDrive());
  }

  public void setRightSpeed(double speed){
      r1.set(ControlMode.PercentOutput,speed);
  }

  public void setLeftSpeed(double speed){
      l1.set(ControlMode.PercentOutput,speed);
  }
}
