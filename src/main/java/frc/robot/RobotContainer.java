// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.auto.AutoSelector;
import frc.robot.state.inputs.ControllerBindings;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.indexer.kicker.KickerSubsystem;
import frc.robot.subsystems.indexer.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.intake.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;

@SuppressWarnings("unused")
public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * .5;

  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // private static final CommandXboxController driverXbox =
  //     new CommandXboxController(0); // controllers
  // private final CommandXboxController operatorXbox = new CommandXboxController(1);

  private final CommandPS5Controller driverController = new CommandPS5Controller(0);
  private final CommandPS5Controller operatorController = new CommandPS5Controller(1);

  // subsystems :)
  private static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Vision objects
  private static final Vision leftVision = new Vision(VisionConstants.LEFT_LIMELIGHT_NAME);
  private static final Vision rightVision = new Vision(VisionConstants.RIGHT_LIMELIGHT_NAME);

  // Shooter Subsystems
  private static final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private static final FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
  private static final HoodSubsystem hoodSubsystem = new HoodSubsystem();

  // Intake Subystems
  private static final DeploySubsystem deploySubsystem = new DeploySubsystem();
  private static final RollerSubsystem rollerSubsystem = new RollerSubsystem();

  // Indexer Subsystems
  private static final SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem();
  private static final KickerSubsystem kickerSubsystem = new KickerSubsystem();

  private final AutoSelector autoSelector = new AutoSelector();

  public RobotContainer() {
    configureBindings();
    configureDogLog();

    ControllerBindings.bindAll(driverController);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> {
              return drive
                  .withVelocityX(-driverController.getLeftY() * MaxSpeed * 0.5)
                  .withVelocityY(-driverController.getLeftX() * MaxSpeed * 0.5)
                  .withRotationalRate(-driverController.getRightX() * MaxAngularRate * 0.5);
            }));

    driverController
        .L2()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(-driverController.getLeftY() * MaxSpeed)
                        .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                        .withRotationalRate(-driverController.getRightX() * MaxAngularRate)));

    driverController.povDown().onTrue(hoodSubsystem.zero());

    driverController.options().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
  }

  /** Configure DogLog options and PowerDistribution */
  public void configureDogLog() {
    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withNtPublish(true)
            .withLogExtras(false)
            .withCaptureNt(false)
            .withCaptureConsole(false));

    DogLog.setPdh(new PowerDistribution(63, ModuleType.kRev));
  }

  public Command getAutonomousCommand() {
    if (autoSelector.getSelectedAuto() != null) {
      return autoSelector.getSelectedAuto();
    }

    return Commands.none();
  }

  public AutoSelector getAutoSelector() {
    return autoSelector;
  }

  public static CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
  }

  public static Vision getLeftVision() {
    return leftVision;
  }

  public static Vision getRightVision() {
    return rightVision;
  }

  public static TurretSubsystem getTurretSubsystem() {
    return turretSubsystem;
  }

  public static FlywheelSubsystem getFlywheelSubsystem() {
    return flywheelSubsystem;
  }

  public static HoodSubsystem getHoodSubsystem() {
    return hoodSubsystem;
  }

  public static DeploySubsystem getDeploySubsystem() {
    return deploySubsystem;
  }

  public static RollerSubsystem getRollerSubsystem() {
    return rollerSubsystem;
  }

  public static SpindexerSubsystem getSpindexerSubsystem() {
    return spindexerSubsystem;
  }

  public static KickerSubsystem getKickerSubsystem() {
    return kickerSubsystem;
  }
}
