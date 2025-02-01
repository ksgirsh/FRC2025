// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;
import java.security.PublicKey;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.generated.TunerConstants;

public class limeLightAllignment extends SubsystemBase {

  private CommandSwerveDrivetrain drivetrain  = TunerConstants.createDrivetrain();
  private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();
  private Boolean run = false;
  private double xSpeed;

  private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  /** Creates a new limeLightAllignment. */
  public limeLightAllignment() {}

  double limelight_aim_proportional(double MaxSpeed)
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= MaxSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  double limelight_range_proportional(double MaxSpeed)
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= MaxSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }



  //later in robot container call schminga(MaxSpeed)
  public void schminga(double MaxSpeed) {
    //finish this shit then drivtetrain.applyRequest(xSpeed, ySPeed, rot .... blah blah blah)
          final var rot_limelight = limelight_aim_proportional(MaxSpeed);
          //rot = rot_limelight;
  
          final var forward_limelight = limelight_range_proportional(MaxSpeed);
          xSpeed = forward_limelight;
  
          //while using Limelight, turn off field-relative driving.
          //fieldRelative = false;
          
  }

  public Command limeLightAllign(){
    return run(() -> this.driveAtTag());
  }
  
  private void driveAtTag(){
    double[] camerapose_targetspace = limelightTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
      double yawSpeed = camerapose_targetspace[4] * 0.01;
      double xSpeed = camerapose_targetspace[0] * Math.cos(camerapose_targetspace[4] * 0.0174533) * -1;
      double ySpeed = (camerapose_targetspace[2] * Math.sin(camerapose_targetspace[4] * 0.0174533) - 0.3);

      drivetrain.applyRequest(() ->
      robotCentricRequest.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(yawSpeed)
      );
      System.out.println("command work");
  }

  @Override
  public void periodic() {

  
  }
}
