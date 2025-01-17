// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static frc.robot.RobotContainer.*;
import static frc.robot.subsystems.CommandSwerveDrivetrain.*;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveAndPointAtTarget extends Command {

  

  private final DoubleSupplier xVelocitySupplier;
  private final DoubleSupplier yVelocitySupplier;
  private FieldCentricFacingAngle request;
  private final Supplier<Rotation2d> thetaSupplier;

  //private final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(null, null);
  /** Creates a new DriveAndPointAtTarget. */

  public DriveAndPointAtTarget(DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier, Supplier<Rotation2d> thetaSupplier) {

    this.xVelocitySupplier = xVelocitySupplier;
    this.yVelocitySupplier = yVelocitySupplier;
    this.thetaSupplier = thetaSupplier; 
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.   
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
   public void execute() {
    if(DriverStation.getAlliance().get() == Alliance.Red){
      request = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(DriveConstants.maxSpeed * 0.03) // Add a 3% deadband to translation
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagic)
        .withVelocityX(xVelocitySupplier.getAsDouble())
        .withVelocityY(yVelocitySupplier.getAsDouble())
        .withTargetDirection(thetaSupplier.get());
    }
    else{
      request = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(DriveConstants.maxSpeed * 0.03) // Add a 3% deadband to translation
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagic)
        .withVelocityX(-xVelocitySupplier.getAsDouble())
        .withVelocityY(-yVelocitySupplier.getAsDouble())
        .withTargetDirection(thetaSupplier.get());
    }
    
    request.HeadingController.setP(9.0);
    request.HeadingController.setD(1.5); 
    request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveSubsystem.setControl(request);
    driveSubsystem.getDistanceToGoal();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
