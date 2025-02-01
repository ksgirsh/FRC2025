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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.generated.TunerConstants;

public class LimelightAlignment extends SubsystemBase {

  private CommandSwerveDrivetrain drivetrain  = TunerConstants.createDrivetrain();
  private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();
  private Boolean run = false;
  private double xSpeed;

  /** Creates a new LimelightAlignment. */
  public LimelightAlignment() {}

  public Command LimelightAlign(){
    return run(() -> this.driveAtTag());
  }
  
  // George Code
  private void driveAtTag(){
      Pose3d cameraPose_TargetSpace = LimelightHelpers.getCameraPose3d_TargetSpace(""); // Camera's pose relative to tag (should use Robot's pose in the future)
            
      // double yawSpeed = camerapose_targetspace[4] * 0.01;
      // double xSpeed = camerapose_targetspace[0] * Math.cos(camerapose_targetspace[4] * 0.0174533) * -1;
      // double ySpeed = (camerapose_targetspace[2] * Math.sin(camerapose_targetspace[4] * 0.0174533) - 0.3);
      // drivetrain.applyRequest(() ->
      // robotCentricRequest.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(yawSpeed)
      // );
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
