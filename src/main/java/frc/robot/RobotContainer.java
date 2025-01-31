// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.RoboSingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.RoboSingSubsytem;



public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    //syom auto heading
    private double targetHeading = 0; // in radians


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
    

    private final JoystickButton pos1 = new JoystickButton(buttonBoard, 1);
    private final JoystickButton pos2 = new JoystickButton(buttonBoard, 2);
    private final JoystickButton pos3 = new JoystickButton(buttonBoard, 3);
    private final JoystickButton pos4 = new JoystickButton(buttonBoard, 4);
    private final JoystickButton pos5 = new JoystickButton(buttonBoard, 5);
    private final JoystickButton pos6 = new JoystickButton(buttonBoard, 6);
    private final JoystickButton intakeL = new JoystickButton(buttonBoard, 7);
    private final JoystickButton intakeR = new JoystickButton(buttonBoard, 8);
    private final JoystickButton algaeProcessor = new JoystickButton(buttonBoard, 9);
     

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();



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



        // syom test for auto gyro heading
        headingRequest.HeadingController.setP(7.0);
        headingRequest.HeadingController.setI(0.0001);
        headingRequest.HeadingController.setD(1.2);

        // syoma schminga binga code sets target autoHeadingAngle nside CommandSwerveSubsytem but only executes trunig to that yaw on press of right bumper(code below joystck.a ,b,x,y onTrue
        
        joystick.y().whileTrue(
            drivetrain.runOnce(() -> targetHeading = 0) // 0 radians (facing forward)
        );

        joystick.x().whileTrue(
            drivetrain.runOnce(() -> targetHeading = Math.PI / 2) // π/2 radians (facing left)
        );

        joystick.a().whileTrue(
            drivetrain.runOnce(() -> targetHeading = Math.PI) // π radians (facing backward)
        );

        joystick.b().whileTrue(
            drivetrain.runOnce(() -> targetHeading = 3 * Math.PI / 2) // 3π/2 radians (facing right)
        );



        // intakes
        joystick.leftTrigger().whileTrue(
            drivetrain.runOnce(() -> targetHeading = 2.2) // angle of left intake (rads)
        );

        joystick.rightTrigger().whileTrue(
            drivetrain.runOnce(() -> targetHeading = 4.084) // angle of right intake (rads)
        );



        // 6 positions for reef (pos/button 1 is the side facing the drivers, go counter-clockwise):

        new JoystickButton(buttonBoard, 1).onTrue(
            drivetrain.runOnce(() -> targetHeading = 0) // close
        );

        new JoystickButton(buttonBoard, 2).onTrue(
            drivetrain.runOnce(() -> targetHeading = Math.PI / 3) // close right
        );

        new JoystickButton(buttonBoard, 3).onTrue(
            drivetrain.runOnce(() -> targetHeading = 2 * Math.PI / 3) // far right
        );

        new JoystickButton(buttonBoard, 4).onTrue(
            drivetrain.runOnce(() -> targetHeading = Math.PI) // far
        );

        new JoystickButton(buttonBoard, 5).onTrue(
            drivetrain.runOnce(() -> targetHeading = 4 * Math.PI / 3) // far left
        );

        new JoystickButton(buttonBoard, 6).onTrue(
            drivetrain.runOnce(() -> targetHeading = 5 * Math.PI / 3) // close left
        );


        // L/R intake & algae processor

        new JoystickButton(buttonBoard, 7).onTrue(
            drivetrain.runOnce(() -> targetHeading = 2.2) // left intake = face 126 degrees = 2.2 rads from forward 
        );

        new JoystickButton(buttonBoard, 8).onTrue(
            drivetrain.runOnce(() -> targetHeading = 4.084) // right intake = face 234 degrees = 4.084 rads from forward
        );

        new JoystickButton(buttonBoard, 9).onTrue(
            drivetrain.runOnce(() -> targetHeading = 3 * Math.PI / 2) // algae processor = face right
        );



        // face the 4 directions

        //used to be a
        new JoystickButton(buttonBoard, 10).onTrue(
            drivetrain.runOnce(() -> targetHeading = Math.PI ) // π radians (facing backward)
        );

        //used to be b
        new JoystickButton(buttonBoard, 11).onTrue(
            drivetrain.runOnce(() -> targetHeading = 3 * Math.PI / 2) // 3π/2 radians (facing right)
        );

        new JoystickButton(buttonBoard, 12).onTrue(
            drivetrain.runOnce(() -> targetHeading = Math.PI ) // π radians (facing forward)
        );

        new JoystickButton(buttonBoard, 13).onTrue(
            drivetrain.runOnce(() -> targetHeading = 3 * Math.PI / 2) // 3π/2 radians (facing left)
        );



        joystick.rightBumper().whileTrue(
            drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withTargetDirection(new Rotation2d(targetHeading)))
        );
        
        // Log a message to the console when the right trigger is pressed
        joystick.rightTrigger().onTrue(
            drivetrain.runOnce(() -> System.out.println("Right trigger pressed linga vbinga oonga boonga"))
        ); //make targetdirection reef, left targ direction intake. make xyab lrud, new 9 buttons objects


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