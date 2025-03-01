package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.opencv.ml.Ml;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;

public class Coral extends SubsystemBase {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Coral mInstance;
  private PeriodicIO mPeriodicIO;

  public static Coral getInstance() {
    if (mInstance == null) {
      mInstance = new Coral();
    }
    return mInstance;
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    REVERSE,
    INDEX,
    READY,
    SCORE
  }

  // private ThriftyNova mLeftMotor;
  // private ThriftyNova mRightMotor;
  private SparkMax mLeftMotor;
  private SparkMax mRightMotor;
  private SparkLimitSwitch laser;
  //true while coral is going past what makes laser detect
  private boolean extend;
  //position when extend changed to true
  private double extendFromRotations;
  private Coral() {
    super("Coral");

    mPeriodicIO = new PeriodicIO();

    mLeftMotor = new SparkMax(Constants.Coral.kLeftMotorId, MotorType.kBrushless);
    mRightMotor = new SparkMax(Constants.Coral.kRightMotorId, MotorType.kBrushless);
    SparkMaxConfig coralConfig = new SparkMaxConfig();
    extend = false;
    coralConfig.idleMode(IdleMode.kBrake).apply(new LimitSwitchConfig().forwardLimitSwitchEnabled(false));
    laser = mLeftMotor.getForwardLimitSwitch();
    mLeftMotor.configure(
        coralConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    mRightMotor.configure(
        coralConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    
    laser = mRightMotor.getForwardLimitSwitch();
    
    
  }

  private static class PeriodicIO {
    double rpm = 0.0;
    double speed_diff = 0.0;

    int index_debounce = 0;

    IntakeState state = IntakeState.NONE;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    mLeftMotor.set(mPeriodicIO.rpm - mPeriodicIO.speed_diff);
    mRightMotor.set(-mPeriodicIO.rpm);

    checkAutoTasks();
  }
//uselless ah function

  // public void writePeriodicOutputs() {
  //   mLeftMotor.set(mPeriodicIO.rpm - mPeriodicIO.speed_diff);
  //   mRightMotor.set(-mPeriodicIO.rpm);
  // }

  public void stop() {
    mPeriodicIO.rpm = 0.0;
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.state = IntakeState.NONE;
  }
/*
  public void outputTelemetry() {
    putNumber("RPM/target", mPeriodicIO.rpm);

    LaserCan.Measurement measurement = mPeriodicIO.measurement;
    if (measurement != null) {
      putNumber("Laser/distance", measurement.distance_mm);
      putNumber("Laser/ambient", measurement.ambient);
      putNumber("Laser/budget_ms", measurement.budget_ms);
      putNumber("Laser/status", measurement.status);

      putBoolean("Laser/hasCoral", isHoldingCoralViaLaserCAN());
    }
  }
*/

  public void reset() {
    stopCoral();
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/
  
  public boolean isHoldingCoralViaLaserCAN() {
    return false;
  }


  public Command setSpeed(double rpm, double speed_diff) {
    return run(()->setspeed(rpm, speed_diff));

  }
  private void setspeed(double rpm, double speed_diff) {
    mPeriodicIO.speed_diff = speed_diff;
    mPeriodicIO.rpm = rpm;
  }

  public Command Intake(){
    return run(() -> intake());
  }

  private void intake() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kIntakeSpeed;
    mPeriodicIO.state = IntakeState.INTAKE;

    //m_leds.setColor(Color.kYellow);
  }

  public Command stopIntake(){
    return run(() -> stopCoral());
  }

  private void stopCoral() {
    mPeriodicIO.rpm = 0.0;
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.state = IntakeState.NONE;
  }


  public Command LaserIntake(){
    return run(()->laserIntake());
  }

  private void laserIntake(){
    if(laser.isPressed() || extend){
      if(!laser.isPressed()){
        if(!extend){
          extend = true;
          extendFromRotations = mLeftMotor.getEncoder().getPosition();
        }
        if(mLeftMotor.getEncoder().getPosition() - extendFromRotations > 0.5){
          extend = false;
          index();
        }else{
          intake();
        }

      }
    }else{
      intake();
    }
  }



  //-- not useful shit-----------

  public void reverse() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kReverseSpeed;
    mPeriodicIO.state = IntakeState.REVERSE;
  }

  public Command idle(){
    return run(() -> index());
  }

  private void index() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kIndexSpeed;
    mPeriodicIO.state = IntakeState.INDEX;

    //m_leds.setColor(Color.kBlue);
  }


  public void scoreL1() {
    mPeriodicIO.speed_diff = Constants.Coral.kSpeedDifference;
    mPeriodicIO.rpm = Constants.Coral.kL1Speed;
    mPeriodicIO.state = IntakeState.SCORE;
  }

  public void scoreL24() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kL24Speed;
    mPeriodicIO.state = IntakeState.SCORE;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
 
  private void checkAutoTasks() {
    switch (mPeriodicIO.state) {
      case INTAKE:
        if (isHoldingCoralViaLaserCAN()) {
          mPeriodicIO.index_debounce++;

          if (mPeriodicIO.index_debounce > 10) {
            mPeriodicIO.index_debounce = 0;
            index();
          }
        }
        break;
      case INDEX:
        if (!isHoldingCoralViaLaserCAN()) {
          stopCoral();

          mPeriodicIO.state = IntakeState.READY;
          //m_leds.setColor(Color.kBlue);
        }
        break;
      default:
        break;
    }
  }
}