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
import frc.robot.commands.ArcadeDrive;
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

    /* The right motors are inverted because for 
     *  the Robot to move in one direction, the motors on different sides
     *   have to spin in opposite directions.
     * */
    r1.setInverted(true);
    r2.setInverted(true);

    // These 2 lines make sure the motors on the same side are spinning at the same speed.
    l2.follow(l1);
    r2.follow(r1);
  }

  @Override
  public void initDefaultCommand() {
    /* This makes sure that the DriveTrain ALWAYS run the TankDrive command.
     * We always want the robot to be able to drive.
     * */
    
    //Pick either TankDrive() or ArcadeDrive(), comment the unpicked out.
    setDefaultCommand(new ArcadeDrive());
    //setDefaultCommand(new TankDrive());
  }

  public void setRightSpeed(double speed){
      /* This sets the speed at which the right motors will run.
       * There are differnet ControlMode's, but in this lesson, 
       * we use PercentOutput for the sake of simplicty*/
      r1.set(ControlMode.PercentOutput,speed);
  }

  public void setLeftSpeed(double speed){
          /* This sets the speed at which the left motors will run.
       * There are differnet ControlMode's, but in this lesson, 
       * we use PercentOutput for the sake of simplicty*/
      l1.set(ControlMode.PercentOutput,speed);
  }
}
