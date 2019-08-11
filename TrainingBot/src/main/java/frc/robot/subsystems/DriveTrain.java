/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveCommand;

/**
 * This substystem is used to model the characteristics and capabilities of the
 * drive train of the robot.
 */
public class DriveTrain extends PIDSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Must have CTRE libraries installed for these to work.
  WPI_TalonSRX l1;
  WPI_TalonSRX r1;
  WPI_TalonSRX l2;
  WPI_TalonSRX r2;
  WPI_TalonSRX l3;
  WPI_TalonSRX r3;

  DifferentialDrive curvatureDrive;

  public DriveTrain() {
    super("Drive", 1, 2, 3);
    l1 = new WPI_TalonSRX(RobotMap.Drive.L1);
    l2 = new WPI_TalonSRX(RobotMap.Drive.L2);
    l3 = new WPI_TalonSRX(RobotMap.Drive.L3);

    r1 = new WPI_TalonSRX(RobotMap.Drive.R1);
    r2 = new WPI_TalonSRX(RobotMap.Drive.R2);
    r3 = new WPI_TalonSRX(RobotMap.Drive.R3);

    /*
     * The right motors are inverted because for the Robot to move in one direction,
     * the motors on different sides have to spin in opposite directions.
     */
    r1.setInverted(true);
    r2.setInverted(true);
    r3.setInverted(true);

    // These 2 lines make sure the motors on the same side are spinning at the same
    // speed.
    l2.follow(l1);
    l3.follow(l1);

    r2.follow(r1);
    r3.follow(r1);

  }

  @Override
  public void initDefaultCommand() {
    /*
     * This makes sure that the DriveTrain ALWAYS run the TankDrive command. We
     * always want the robot to be able to drive.
     */

    // Pick either TankDrive() or ArcadeDrive(), comment the unpicked out.
    setDefaultCommand(new DriveCommand());
    // setDefaultCommand(new TankDrive());
  }

  public void setRightSpeed(double speed) {
    /*
     * This sets the speed at which the right motors will run. There are differnet
     * ControlMode's, but in this lesson, we use PercentOutput for the sake of
     * simplicty
     */
    r1.set(ControlMode.PercentOutput, speed);
  }

  public void setLeftSpeed(double speed) {
    /*
     * This sets the speed at which the left motors will run. There are differnet
     * ControlMode's, but in this lesson, we use PercentOutput for the sake of
     * simplicty
     */
    l1.set(ControlMode.PercentOutput, speed);
  }

  @Override
  protected double returnPIDInput() {
    return 0;
  }

  @Override
  protected void usePIDOutput(double output) {
    l1.pidWrite(output);
    r1.pidWrite(output);
  }
}
