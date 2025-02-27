// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


//subsystems imports
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.RoboSingSubsytem;
import frc.robot.subsystems.LimelightAlignment;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.AlgaeGroundtake;



public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    //syom auto heading
    private double targetHeadingReef = 0; // in radians
    private double targetHeadingIntake = 0;


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


    //syom auto heading
    private final SwerveRequest.FieldCentricFacingAngle headingRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.05) // Add a 3% deadband to translation
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    ;

    

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    Joystick buttonBoard = new Joystick(1);
    private final JoystickButton back = new JoystickButton(buttonBoard, 10);
    private final JoystickButton right = new JoystickButton(buttonBoard, 11);
    private final JoystickButton forward = new JoystickButton(buttonBoard, 12);
    private final JoystickButton left = new JoystickButton(buttonBoard, 13);

    private final POVButton elevatorl4 = new POVButton(buttonBoard, 90);
    private final POVButton elevatorl3 = new POVButton(buttonBoard, 180);
    private final POVButton elevatorl2 = new POVButton(buttonBoard, 270);
    private final POVButton elevatorl1 = new POVButton(buttonBoard, 0);
    
    //initializing buttons, Bb stands for buttonboard
    private final JoystickButton BbReefBottomCenter = new JoystickButton(buttonBoard, 1);
    private final JoystickButton BbReefBottomRight = new JoystickButton(buttonBoard, 2);
    private final JoystickButton BbReefTopRight = new JoystickButton(buttonBoard, 3);
    private final JoystickButton BbReefTopCenter = new JoystickButton(buttonBoard, 4);
    private final JoystickButton BbReefTopLeft = new JoystickButton(buttonBoard, 5);
    private final JoystickButton BbReefBottomLeft = new JoystickButton(buttonBoard, 6);
    private final JoystickButton BbIntakeL = new JoystickButton(buttonBoard, 7);
    private final JoystickButton BbIntakeR = new JoystickButton(buttonBoard, 8);
    private final JoystickButton BbAlgaeProcessor = new JoystickButton(buttonBoard, 9);
     

    //instancing subsystems

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = Elevator.getInstance();
    public final AlgaeGroundtake algaeGroundtake = AlgaeGroundtake.getInstance();
    public final LimelightAlignment limelight = new LimelightAlignment();
    private final Coral coral = Coral.getInstance();



    //syom sing intialize
    public final RoboSingSubsytem roboSingSubsytem = new RoboSingSubsytem();
    // private final RoboSingCommand singCommand = new RoboSingCommand(roboSing);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //syom sing

        joystick.pov(90).onTrue(roboSingSubsytem.playRock());

        // george limelight 
        joystick.pov(270).onTrue(limelight.setYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble()));
        joystick.pov(270).whileTrue(limelight.LimelightAlign(drivetrain));



        // syom test for auto gyro heading
        headingRequest.HeadingController.setP(7.0);
        headingRequest.HeadingController.setI(0.0001);
        headingRequest.HeadingController.setD(1.2);

        //elevator testing syom
        joystick.a().whileTrue(
            algaeGroundtake.goToPivotIn()// go to pivot out position
        );

        joystick.b().whileTrue(
            algaeGroundtake.goToPivotOut()//o to elevator level 3 of reef
        );
        // syom sets target autoHeadingAngle nside CommandSwerveSubsytem but only executes turning to that yaw on press of right bumper(code below joystck.a ,b,x,y onTrue
        
        joystick.y().whileTrue(
            algaeGroundtake.resetPivotZero()// 0 radians (facing forward)
        );

        joystick.x().onTrue(
           algaeGroundtake.intakeCommand() // run intkae 
        );

        elevatorl4.onTrue(
               elevator.goToElevatorL4() // go to elevator level 4 of reef
        );

        elevatorl3.onTrue(
            elevator.goToElevatorL3() // go to elevator level 3 of reef
        );

        elevatorl2.onTrue(
            elevator.goToElevatorL2() // go to elevator level 2 of reef
        );

        elevatorl1.onTrue(
            elevator.goToElevatorStow() // go to elevator stow position
        );

        // temporary removed to use for elevator testing
        // joystick.a().whileTrue(
        //     drivetrain.runOnce(() -> targetHeadingReef = Math.PI) // π radians (facing backward)
        // );

        // joystick.b().whileTrue(
        //     drivetrain.runOnce(() -> targetHeadingReef = 3 * Math.PI / 2) // 3π/2 radians (facing right)
        // );




        // 6 positions for reef (pos/button 1 is the side facing the drivers, go counter-clockwise):
        BbReefBottomCenter.onTrue(
            drivetrain.runOnce(() -> {
            targetHeadingReef = 0; // close
            System.out.println("Reef Bottom Center button pressed, target heading set to 0 radians.");
            })
        );

        BbReefBottomRight.onTrue(
            drivetrain.runOnce(() -> targetHeadingReef = Math.PI / 3) // close right
        );

        BbReefTopRight.onTrue(
            drivetrain.runOnce(() -> targetHeadingReef = 2 * Math.PI / 3) // far right
        );

        BbReefTopCenter.onTrue(
            drivetrain.runOnce(() -> targetHeadingReef = Math.PI) // far
        );

        BbReefTopLeft.onTrue(
            drivetrain.runOnce(() -> targetHeadingReef = 4 * Math.PI / 3) // far left
        );

        BbReefBottomLeft.onTrue(
            drivetrain.runOnce(() -> targetHeadingReef = 5 * Math.PI / 3) // close left
        );


        // L/R intake & algae processor

        BbIntakeL.onTrue(
            drivetrain.runOnce(() -> targetHeadingIntake = 2.2 +Math.PI) // left intake = face 126 degrees = 2.2 rads from forward 
        );

        BbIntakeR.onTrue(
            drivetrain.runOnce(() -> targetHeadingIntake = 4.084 + Math.PI) // right intake = face 234 degrees = 4.084 rads from forward
        );

        BbAlgaeProcessor.onTrue(
            drivetrain.runOnce(() -> targetHeadingIntake = 3 * Math.PI / 2) // algae processor = face right
        );

        //turn to towards selected field orientated angle towards reef
        joystick.rightTrigger().whileTrue(
            drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withTargetDirection(new Rotation2d(targetHeadingReef)))
        );





        joystick.rightBumper().whileTrue(
            drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withTargetDirection(new Rotation2d(targetHeadingIntake)))
        );

        // ong we dont need this shit wtf is breaking 

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}