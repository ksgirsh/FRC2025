// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Dealgaefier extends SubsystemBase {
  /** Creates a new dealgaefier. */
  private static Dealgaefier mInstance;

  private PeriodicIO mPeriodicIO;

  //get instance method
  public static Dealgaefier getInstance() {
    if (mInstance == null) {
      mInstance = new Dealgaefier();
    }
    return mInstance;
  }

  private SparkMax mPivotMotor;
  private RelativeEncoder mPivotEncoder;
  private SparkClosedLoopController mPivotPIDController;

  private SparkMax mIntakeMotor;

  private TrapezoidProfile mProfile;
  private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
  private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
  private double prevUpdateTime = Timer.getFPGATimestamp();

  

  private Dealgaefier() {
    super("dealgaefier");
    mPeriodicIO = new PeriodicIO();

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    SparkMaxConfig intakeConfig = new SparkMaxConfig();

    mIntakeMotor = new SparkMax(Constants.Dealgaefier.kIntakeMotorId, MotorType.kBrushless);

    mIntakeMotor.configure(
      intakeConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    pivotConfig.closedLoop
        .pid(Constants.Dealgaefier.kP, Constants.Dealgaefier.kI, Constants.Dealgaefier.kD)
        .iZone(Constants.Dealgaefier.kIZone);

    pivotConfig.smartCurrentLimit(Constants.Dealgaefier.kMaxCurrent);

    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.limitSwitch.reverseLimitSwitchEnabled(true);

    // pivot motor
    mPivotMotor = new SparkMax(Constants.Dealgaefier.kPivotMotorId, MotorType.kBrushless);
    mPivotEncoder = mPivotMotor.getEncoder();
    mPivotPIDController = mPivotMotor.getClosedLoopController();
    mPivotMotor.configure(
        pivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    mProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            Constants.Dealgaefier.kMaxVelocity,
            Constants.Dealgaefier.kMaxAcceleration));
  }


  public enum IntakeState {
    NONE,
    INTAKE,
    REVERSE,
  }

  public enum PivotState {
    NONE,
    IN,
    OUT,
  }



  private static class PeriodicIO {
  double pivot_target = 0.0;
  double pivot_power = 0.0;
  double intake_rpm = 0.0;
  double intake_speed_diff = 0.0;

  boolean is_groundtake_pos_control = false;

  PivotState pstate = PivotState.NONE;
  IntakeState istate = IntakeState.NONE;

  }

  

  @Override
  public void periodic() {
    //intake code
    mIntakeMotor.set(mPeriodicIO.intake_rpm);
    // System.out.println("Intake RPM: " + mPeriodicIO.intake_rpm);

    //pivot code 

      double curTime = Timer.getFPGATimestamp();
      double dt = curTime - prevUpdateTime;
      prevUpdateTime = curTime;
      if(mPeriodicIO.is_groundtake_pos_control){
        // Update goal
        mGoalState.position = mPeriodicIO.pivot_target;
        // Calculate new state
        prevUpdateTime = curTime;
        mCurState = mProfile.calculate(dt, mCurState, mGoalState);
        mPivotPIDController.setReference(
          mCurState.position,
          SparkBase.ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
          Constants.Dealgaefier.kG,
          ArbFFUnits.kVoltage);
    } else {
      mCurState.position = mPivotEncoder.getPosition();
      mCurState.velocity = 0;
      mPivotMotor.set(mPeriodicIO.pivot_power);
    }
  }

  public void stop() {
    mPeriodicIO.is_groundtake_pos_control = false;
    mPeriodicIO.pivot_power = 0.0;

    mPivotMotor.set(0.0);
  }

  public Command resetPivotZero(){
    return run(()-> reset());
  }

  private void reset() {
    mPivotEncoder.setPosition(0.0);
  }
  //intake commands

  public Command Intake() {
    return run(() -> intake());
  }

  private void intake() {
    mPeriodicIO.intake_speed_diff = 0.0;
      mPeriodicIO.intake_rpm = Constants.Dealgaefier.kIntakeSpeed;
      mPeriodicIO.istate = IntakeState.INTAKE;
  }

  public Command ReverseIntake() {
    return run(() -> reverse());
  }

  private void reverse() {
    mPeriodicIO.intake_speed_diff = 0.0;
    mPeriodicIO.intake_rpm = Constants.Dealgaefier.kReverseSpeed;
    mPeriodicIO.istate = IntakeState.REVERSE;
  }

  public Command StopIntake() {
    return run(() -> stopintake());
  }

  private void stopintake() {
    mPeriodicIO.intake_speed_diff = 0.0;
    mPeriodicIO.intake_rpm = 0.0;
    mPeriodicIO.istate = IntakeState.NONE;
  }



  //flop commands 
  public Command FlopIn() {
    return run(() -> gotopivotin());
  }

  private void gotopivotin() {
    mPeriodicIO.is_groundtake_pos_control = true;
    mPeriodicIO.pivot_target= Constants.Dealgaefier.kInRotations;
    mPeriodicIO.pstate = PivotState.IN;

    mPeriodicIO.intake_speed_diff = 0.0;
    mPeriodicIO.intake_rpm = 0.0;
    mPeriodicIO.istate = IntakeState.NONE;
  }

  public Command FlopOut() {
    return run(() -> gotopivotout());
    
  }

  private void gotopivotout() {
    mPeriodicIO.is_groundtake_pos_control = true;
    mPeriodicIO.pivot_target= Constants.Dealgaefier.kOutRotations;
    mPeriodicIO.pstate = PivotState.OUT;

    mPeriodicIO.intake_speed_diff = 0.0;
    mPeriodicIO.intake_rpm = Constants.Dealgaefier.kIntakeSpeed;
    mPeriodicIO.istate = IntakeState.INTAKE;
  }



}
