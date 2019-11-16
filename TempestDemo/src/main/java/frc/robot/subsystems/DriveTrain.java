/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.NormalDrive;;

/**
 * This substystem is used to model the characteristics and capabilities of the
 * drive train of the robot.
 */
public class DriveTrain extends Subsystem implements PIDSource, PIDOutput {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private boolean isQuickTurn = true;

  // Must have CTRE libraries installed for these to work.
  WPI_TalonSRX l1;
  WPI_TalonSRX l2;
  WPI_TalonSRX l3;

  WPI_TalonSRX r1;
  WPI_TalonSRX r2;
  WPI_TalonSRX r3;

  public PIDController pidController;

  public DriveTrain() {
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

    pidController = new PIDController(2, 0, .75, this, this);
    pidController.setAbsoluteTolerance(4096);// raw sensor input
    pidController.setOutputRange(-1, 1);
    pidController.disable();

    initSensors();
  }

  @Override
  public void initDefaultCommand() {
    /*
     * This makes sure that the DriveTrain ALWAYS run the __Drive() command. We
     * always want the robot to be able to drive.
     */

    // Pick one , comment the unpicked out.
    setDefaultCommand(new ArcadeDrive());
    // setDefaultCommand(new TankDrive());
    // setDefaultCommand(new CurvatureDRive());
    // setDefaultCommand(new NormalDrive());

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

  // units is in raw sensor tick
  public void setStraightPosition(double inches) {
    r1.set(ControlMode.Position, inches);
    l1.set(ControlMode.PercentOutput,r1.get());
  }

  public void initSensors() {
    l1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    l1.setSensorPhase(true);
    l1.setSelectedSensorPosition(0);
    l1.configNominalOutputReverse(0, 0);
    l1.configPeakOutputForward(1, 0);
    l1.configPeakOutputReverse(-1, 0);
    l1.setNeutralMode(NeutralMode.Brake);

    r1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    r1.setSensorPhase(true);
    r1.setSelectedSensorPosition(0);
    r1.configNominalOutputReverse(0, 0);
    r1.configPeakOutputForward(1, 0);
    r1.configPeakOutputReverse(-1, 0);
    r1.setNeutralMode(NeutralMode.Brake);

    // 0-> slotID
    r1.configAllowableClosedloopError(0, (int) (10/RobotMap.Constants.InchesPerTick));
    l1.configAllowableClosedloopError(0, (int) (10/RobotMap.Constants.InchesPerTick));

  }

  public void configPID(double kP, double kI, double kD) {
    r1.config_kP(0, kP);
    r1.config_kI(0, kI);
    r1.config_kD(0, kD);

    l1.config_kP(0, kP);
    l1.config_kI(0, kI);
    l1.config_kD(0, kD);

    // /**
    // * Grab the 360 degree position of the MagEncoder's absolute
    // * position, and intitally set the relative sensor to match.
    // */
    // int absolutePosition = _talon.getSensorCollection().getPulseWidthPosition();

    // /* Mask out overflows, keep bottom 12 bits */
    // absolutePosition &= 0xFFF;
    // if (Constants.kSensorPhase) { absolutePosition *= -1; }
    // if (Constants.kMotorInvert) { absolutePosition *= -1; }

    // /* Set the quadrature (relative) sensor to match absolute */
    // _talon.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx,
    // Constants.kTimeoutMs);
  }

  public double getClosedLoopTarget(Hand hand) {
    if (hand.equals(Hand.kLeft))
      return l1.getClosedLoopTarget();
    else
      return r1.getClosedLoopTarget();
  }

  // WPI's PIDController

  public double getCurrentPosition(Hand hand) {
    if (hand.equals(Hand.kLeft))
      return l1.getSelectedSensorPosition();
    else
      return r1.getSelectedSensorPosition();
  }
    public void zeroCurrentPosition(Hand hand) {
    if (hand.equals(Hand.kLeft))
      l1.setSelectedSensorPosition(0);
    else
      r1.setSelectedSensorPosition(0);
  }

  // PIDController
  @Override
  public void pidWrite(double output) {
    l1.set(ControlMode.PercentOutput, output);
    r1.set(ControlMode.PercentOutput,output);
  }

  @Override
  public void setPIDSourceType(PIDSourceType pidSource) {

  }

  @Override
  public PIDSourceType getPIDSourceType() {
    return PIDSourceType.kDisplacement;
  }

  @Override
  public double pidGet() {
    return r1.getSelectedSensorPosition();
  }

  public void log() {
    SmartDashboard.putNumber("DriveEncoder Val", r1.getSelectedSensorPosition()*RobotMap.Constants.InchesPerTick);
    SmartDashboard.putNumber("Left Speed", l1.get());
    SmartDashboard.putNumber("Right Speed", r1.get());
  }

  public boolean getIsQuickTurn() {
    return this.isQuickTurn;
  }

  public void setIsQuickTurn(boolean isQuickTurn) {
    this.isQuickTurn = isQuickTurn;
  }

}
