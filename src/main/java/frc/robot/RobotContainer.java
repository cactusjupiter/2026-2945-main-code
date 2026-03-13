// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Loader;

// import frc.robot.LimelightHelpers;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController manipulatorController = new CommandXboxController(1);

    // initialize subsystems
    private final Intake intakeSubsystem = new Intake();
    private final Shooter shooterSubsystem = new Shooter();
    private final Loader loaderSubsystem = new Loader();
    private final Agitator agitatorSubsystem = new Agitator();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // HEADER: Register named commands for auto

        NamedCommands.registerCommand("runShooter", new ParallelCommandGroup(
            agitatorSubsystem.agitatorCWCommand().withTimeout(10),
            loaderSubsystem.loaderShootCommand().withTimeout(10),
            shooterSubsystem.shooterShootCommand().withTimeout(10)
        ));
        
        NamedCommands.registerCommand("runIntake", intakeSubsystem.intakeInCommand().withTimeout(2));
        NamedCommands.registerCommand("runIntakeZoned", intakeSubsystem.intakeInCommand());

        /*NamedCommands.registerCommand("doClimb", new SequentialCommandGroup(
          intakeSubsystem.intakeOutCommand().withTimeout(1.0),
            climbSubsystem.climbAutoCommand()
        ));*/
        

        autoChooser = AutoBuilder.buildAutoChooser("NO_MOVE_SHOOT");
        SmartDashboard.putData("Auto Mode", autoChooser);


        configureBindings();

        // Switch to pipeline 0
        LimelightHelpers.setPipelineIndex(Constants.LIMELIGHT_NAME, 0);
        LimelightHelpers.setLEDMode_ForceBlink(Constants.LIMELIGHT_NAME);
        LimelightHelpers.setStreamMode_PiPSecondary("snake");

        // valid ids for limelight to detect
        int[] validIDs = {15};
        LimelightHelpers.SetFiducialIDFiltersOverride(Constants.LIMELIGHT_NAME, validIDs);
        
        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // set default commands

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        agitatorSubsystem.setDefaultCommand(agitatorSubsystem.agitatorDefaultCommand());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on y press.
        driverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        //shooter speed control
        manipulatorController.povUp().onTrue(shooterSubsystem.shooterSpeedUpCommand());
        manipulatorController.povDown().onTrue(shooterSubsystem.shooterSpeedDownCommand());
        //INTAKE
        manipulatorController.leftTrigger().whileTrue(intakeSubsystem.intakeInCommand());
        manipulatorController.leftBumper().whileTrue(intakeSubsystem.intakeOutCommand());

        // Agitator override
        manipulatorController.povLeft().whileTrue(agitatorSubsystem.agitatorCCWCommand());
        manipulatorController.povRight().whileTrue(agitatorSubsystem.agitatorCWCommand());

        // a button shoots
        manipulatorController.a().toggleOnTrue(shooterSubsystem.shooterShootCommand());

        // Loader is right trigger and bumper
        manipulatorController.rightTrigger().whileTrue(loaderSubsystem.loaderShootCommand());
        manipulatorController.rightBumper().whileTrue(loaderSubsystem.loaderReverseCommand());

        // NO MORR CLIMBAR
        // Left stick axes are for climbing
        //manipulatorController.axisGreaterThan(Constants.CONTROLLER_LY_AXIS, Constants.AXIS_THRESHOLD).whileTrue(climbSubsystem.climbBothDownCommand());
        //manipulatorController.axisLessThan(Constants.CONTROLLER_LY_AXIS, -Constants.AXIS_THRESHOLD).whileTrue(climbSubsystem.climbBothUpCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public void swerveMegaTagUpdate() {
        drivetrain.megaTagUpdate(); // performs the megatag update in drivetrain
    }
}
