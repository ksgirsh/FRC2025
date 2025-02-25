// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;
import java.security.PublicKey;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.controller.PIDController;

public class LimelightAlignment extends SubsystemBase {

  //private CommandSwerveDrivetrain drivetrain  = TunerConstants.createDrivetrain();
  private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();
  private Boolean run = false;
  private double xSpeed;
  //PID bad for Limelight don't use
  //private final PIDController xControl = new PIDController(1, 0, 0.3);
  //private final PIDController zControl = new PIDController(1, 0, 0.3);
  private final PIDController yawControl = new PIDController(0.3, 0, 0.4);
  private final double kix = 1.5;
  private final double kiy = 1.5;
  

  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** Creates a new LimelightAlignment. */
  public LimelightAlignment() {}

  public Command LimelightAlign(CommandSwerveDrivetrain drivetrain){
    return run(() -> this.driveAtTag(drivetrain));
  }
  
  public Command setYaw(double yaw){
    return run(() -> yawControl.setSetpoint(yaw));
  }

  // George Code
  private void driveAtTag(CommandSwerveDrivetrain driveT){
      Pose3d cameraPose_TargetSpace = LimelightHelpers.getCameraPose3d_TargetSpace(""); // Camera's pose relative to tag (should use Robot's pose in the future)
          
      yawControl.setTolerance(0.05);
      yawControl.enableContinuousInput(-180, 180);
      double ySpeed = kiy * cameraPose_TargetSpace.getX();
      double xSpeed = -kix * (cameraPose_TargetSpace.getZ() + 1);
      System.out.println("Z: " + cameraPose_TargetSpace.getZ());
      double yawSpeed = -yawControl.calculate(driveT.getPigeon2().getYaw().getValueAsDouble())  * 0.01;
      if(yawSpeed < 0.05){
        yawSpeed = 0;
      }
      
      //driveT.applyRequest(() ->
      //  robotCentricRequest.withVelocityX(xSpeed).withVelocityY(ySpeed)
      //);
      driveT.setControl(new SwerveRequest.RobotCentric().withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(yawSpeed));
      //System.out.println("X Speed: " + xSpeed + " Y Speed: " + ySpeed + " X Pos: " + cameraPose_TargetSpace.getX() + " Y Pos: " + cameraPose_TargetSpace.getY());

  }

  @Override
  public void periodic() {
    // Basic targeting data
    double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
    boolean tv = LimelightHelpers.getTV(""); // Do you have a valid target?

    // System.out.println("tx: " + tx + " ty: " + ty + " ta: " + ta + " tv: " + tv);
  }
}
