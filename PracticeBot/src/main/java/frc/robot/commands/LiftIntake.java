/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.util.Direction;

/**
 * Add your docs here.
 */
public class LiftIntake extends InstantCommand {
  /**
   * Add your docs here.
   */
  Direction dir;

  public LiftIntake(Direction direction) {
    super();
    dir = direction;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (dir.equals(Direction.Down))
      Robot.intake.liftDown();
    else if (dir.equals(Direction.Up))
      Robot.intake.liftUp();
  }

}
