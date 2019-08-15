/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Pneumatic Intake on Tempest
 */
public class Intake extends Subsystem {
  Solenoid intakeSolenoid = new Solenoid(RobotMap.Pneumatics.intake);
  
  public Intake(){
    intakeSolenoid = new Solenoid(RobotMap.Pneumatics.intake);
		liftUp();
  }
  
  public void liftUp() {
    intakeSolenoid.set(false);
  }

  public void liftDown(){
    intakeSolenoid.set(true);
  }

  @Override
  public void initDefaultCommand() {
  }
}
