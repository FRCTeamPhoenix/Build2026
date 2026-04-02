// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.team2342.frc.Constants.CANConstants;
import org.team2342.frc.Constants.DriveConstants;
import org.team2342.frc.Constants.IndexerConstants;
import org.team2342.frc.Constants.IntakeConstants;
import org.team2342.frc.Constants.KickerConstants;
import org.team2342.frc.Constants.ShooterConstants;
import org.team2342.frc.Constants.TurretConstants;
import org.team2342.frc.Constants.VisionConstants;
import org.team2342.frc.commands.DriveCommands;
import org.team2342.frc.commands.RotationLockedDrive;
import org.team2342.frc.subsystems.Conductor;
import org.team2342.frc.subsystems.Conductor.ConductorState;
import org.team2342.frc.subsystems.drive.Drive;
import org.team2342.frc.subsystems.drive.GyroIO;
import org.team2342.frc.subsystems.drive.GyroIOPigeon2;
import org.team2342.frc.subsystems.drive.ModuleIO;
import org.team2342.frc.subsystems.drive.ModuleIOSim;
import org.team2342.frc.subsystems.drive.ModuleIOTalonFX;
import org.team2342.frc.subsystems.indexer.Disruptor;
import org.team2342.frc.subsystems.indexer.Indexer;
import org.team2342.frc.subsystems.intake.Pivot;
import org.team2342.frc.subsystems.intake.Wheels;
import org.team2342.frc.subsystems.shooter.Flywheel;
import org.team2342.frc.subsystems.shooter.Kicker;
import org.team2342.frc.subsystems.shooter.Turret;
import org.team2342.frc.subsystems.vision.Vision;
import org.team2342.frc.subsystems.vision.VisionIO;
import org.team2342.frc.subsystems.vision.VisionIOPhoton;
import org.team2342.frc.subsystems.vision.VisionIOSim;
import org.team2342.frc.util.FieldConstants;
import org.team2342.frc.util.FiringSolver;
import org.team2342.frc.util.HubShiftUtil;
import org.team2342.lib.motors.dumb.DumbMotorIO;
import org.team2342.lib.motors.dumb.DumbMotorIOSim;
import org.team2342.lib.motors.dumb.DumbMotorIOSparkFlex;
import org.team2342.lib.motors.dumb.DumbMotorIOTalonFX;
import org.team2342.lib.motors.dumb.DumbMotorIOTalonFXFOC;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOSim;
import org.team2342.lib.motors.smart.SmartMotorIOTalonFX;
import org.team2342.lib.pidff.PIDFFConfigs;
import org.team2342.lib.util.AllianceUtils;
import org.team2342.lib.util.EnhancedXboxController;

public class RobotContainer {
  @Getter private final Drive drive;
  @Getter private final Vision vision;
  @Getter private final Pivot pivot;
  @Getter private final Indexer indexer;
  @Getter private final Disruptor disruptor;
  @Getter private final Kicker kicker;
  @Getter private final Wheels wheels;
  @Getter private final Flywheel flywheel;
  @Getter private final Turret turret;

  @Getter private final Conductor conductor;

  private final LoggedDashboardChooser<Command> autoChooser;

  @Getter
  private final EnhancedXboxController driverController =
      new EnhancedXboxController(0, DriveConstants.CONTROLLER_DEADBAND);

  @Getter
  private final EnhancedXboxController operatorController =
      new EnhancedXboxController(1, DriveConstants.CONTROLLER_DEADBAND);

  private final Alert driverControllerAlert =
      new Alert("Driver controller is disconnected!", AlertType.kError);
  private final Alert operatorControllerAlert =
      new Alert("Operator controller is disconnected!", AlertType.kError);

  private final Trigger allianceZoneTrigger;
  private final Trigger shiftAboutToEnd;
  private final Trigger activeOrPassing;
  private final Trigger readyToFire;

  @Getter private double turretManual, flywheelManual;

  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(CANConstants.PIGEON_ID),
                new ModuleIOTalonFX(CANConstants.FL_IDS, DriveConstants.ENCODER_OFFSETS[0]),
                new ModuleIOTalonFX(CANConstants.FR_IDS, DriveConstants.ENCODER_OFFSETS[1]),
                new ModuleIOTalonFX(CANConstants.BL_IDS, DriveConstants.ENCODER_OFFSETS[2]),
                new ModuleIOTalonFX(CANConstants.BR_IDS, DriveConstants.ENCODER_OFFSETS[3]));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getTimestampedHeading,
                new VisionIOPhoton(
                    VisionConstants.LEFT_CAMERA_PARAMETERS,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
                new VisionIOPhoton(
                    VisionConstants.RIGHT_CAMERA_PARAMETERS,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));
        turret =
            new Turret(
                new SmartMotorIOTalonFX(
                    CANConstants.TURRET_ID,
                    TurretConstants.TURRET_CONFIG.withPIDFFConfigs(TurretConstants.PID_CONFIG)));
        flywheel =
            new Flywheel(
                new SmartMotorIOTalonFX(
                    CANConstants.FLYWHEEL_MOTOR_ID,
                    ShooterConstants.FLYWHEEL_CONFIG.withPIDFFConfigs(
                        ShooterConstants.FLYWHEEL_PID_CONFIGS)));
        kicker =
            new Kicker(
                new DumbMotorIOTalonFXFOC(CANConstants.KICKER_ID, KickerConstants.KICKER_CONFIG));

        indexer =
            new Indexer(
                new DumbMotorIOTalonFXFOC(
                    CANConstants.INDEXER_MOTOR_ID, IndexerConstants.INDEXER_MOTOR_CONFIG));

        disruptor =
            new Disruptor(
                new DumbMotorIOSparkFlex(
                    CANConstants.DISRUPTOR_ID,
                    IndexerConstants.DISRUPTOR_MOTOR_CONFIG,
                    MotorType.kBrushless));
        pivot =
            new Pivot(
                new SmartMotorIOTalonFX(
                    CANConstants.INTAKE_PIVOT_MOTOR_ID,
                    IntakeConstants.PIVOT_MOTOR_CONFIG.withPIDFFConfigs(
                        IntakeConstants.PIVOT_MOTOR_PID_CONFIGS)));
        wheels =
            new Wheels(
                new DumbMotorIOTalonFX(
                    CANConstants.INTAKE_WHEEL_MOTOR_ID,
                    IntakeConstants.INTAKE_WHEELS_MOTOR_CONFIG));

        conductor =
            new Conductor(
                flywheel,
                turret,
                drive::getPose,
                drive::getFieldRelativeChassisSpeeds,
                () -> turretManual,
                () -> flywheelManual);
        LoggedPowerDistribution.getInstance(CANConstants.PDH_ID, ModuleType.kRev);
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getTimestampedHeading,
                new VisionIOSim(
                    VisionConstants.LEFT_CAMERA_PARAMETERS,
                    PoseStrategy.CONSTRAINED_SOLVEPNP,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    drive::getRawOdometryPose),
                new VisionIOSim(
                    VisionConstants.RIGHT_CAMERA_PARAMETERS,
                    PoseStrategy.CONSTRAINED_SOLVEPNP,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    drive::getRawOdometryPose));
        indexer =
            new Indexer(
                new DumbMotorIOSim(
                    IndexerConstants.INDEXER_SIM_MOTOR, IndexerConstants.INDEXER_SIM));

        disruptor =
            new Disruptor(
                new DumbMotorIOSim(
                    IndexerConstants.DISRUPTOR_SIM_MOTOR, IndexerConstants.DISRUPTOR_SIM));

        wheels =
            new Wheels(
                new DumbMotorIOSim(
                    IntakeConstants.INTAKE_WHEELS_SIM_MOTOR, IntakeConstants.INTAKE_WHEEL_SIM));
        flywheel =
            new Flywheel(
                new SmartMotorIOSim(
                    ShooterConstants.FLYWHEEL_CONFIG.withPIDFFConfigs(new PIDFFConfigs().withKP(1)),
                    ShooterConstants.FLYWHEEL_SIM_MOTOR,
                    ShooterConstants.FLYWHEEL_SIM,
                    1));
        kicker = new Kicker(new DumbMotorIO() {});
        turret = new Turret(new SmartMotorIO() {});
        pivot = new Pivot(new SmartMotorIO() {});

        conductor =
            new Conductor(
                flywheel,
                turret,
                drive::getPose,
                drive::getFieldRelativeChassisSpeeds,
                () -> turretManual,
                () -> flywheelManual);
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getTimestampedHeading,
                new VisionIO() {},
                new VisionIO() {});
        indexer = new Indexer(new DumbMotorIO() {});
        disruptor = new Disruptor(new DumbMotorIO() {});
        wheels = new Wheels(new DumbMotorIO() {});
        pivot = new Pivot(new SmartMotorIO() {});
        flywheel = new Flywheel(new SmartMotorIO() {});
        turret = new Turret(new SmartMotorIO() {});
        kicker = new Kicker(new DumbMotorIO() {});

        conductor =
            new Conductor(
                flywheel,
                turret,
                drive::getPose,
                drive::getFieldRelativeChassisSpeeds,
                () -> turretManual,
                () -> flywheelManual);
        break;
    }

    SmartDashboard.putData(
        "Calculate Vision Heading Offset",
        Commands.runOnce(() -> drive.calculateVisionHeadingOffset())
            .alongWith(Commands.print("Calculated Vision Offset"))
            .ignoringDisable(true));

    allianceZoneTrigger =
        new Trigger(
            () ->
                withinBounds(
                    drive.getPose().getX(),
                    AllianceUtils.flipToAlliance(Pose2d.kZero).getX(),
                    AllianceUtils.flipToAlliance(FieldConstants.LeftBump.nearLeftCorner).getX()));
    shiftAboutToEnd = new Trigger(() -> (HubShiftUtil.getShiftedShiftInfo().remainingTime() < 1.0));
    activeOrPassing =
        new Trigger(
            () ->
                HubShiftUtil.getOfficialShiftInfo().active()
                    || FiringSolver.getInstance()
                        .calculate(drive.getChassisSpeeds(), drive.getPose())
                        .passing());
    readyToFire = new Trigger(() -> turret.aroundGoal() && flywheel.atGoal());

    configureBindings();
    configureNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.get();
    autoChooser.addOption(
        "Point and fire",
        conductor
            .runState(ConductorState.WARM_UP)
            .withTimeout(2.0)
            .andThen(
                conductor
                    .runState(ConductorState.TRACKED_FIRING)
                    .alongWith(pivot.holdAngle(0))
                    .alongWith(wheels.in())
                    .alongWith(indexer.in())
                    .alongWith(disruptor.in())
                    .alongWith(kicker.in())));
    if (Constants.TUNING) setupDevelopmentRoutines();
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Named Command Test", Commands.print("Named Command Test"));

    NamedCommands.registerCommand("Drop Intake", pivot.holdAngle(0.7).alongWith(wheels.in()));

    NamedCommands.registerCommand("Stow Intake", pivot.holdAngle(IntakeConstants.MAX_ANGLE));

    NamedCommands.registerCommand("Stop Drive", Commands.run(drive::stopWithX, drive));

    NamedCommands.registerCommand(
        "Shoot",
        conductor
            .runState(ConductorState.TRACKED_FIRING)
            .alongWith(
                Commands.waitSeconds(0.1)
                    .andThen(Commands.waitUntil(readyToFire))
                    .andThen(Commands.parallel(indexer.pulseIn(), kicker.in(), disruptor.in()))));

    NamedCommands.registerCommand(
        "Intake", Commands.parallel(pivot.holdAngle(IntakeConstants.MIN_ANGLE), wheels.in()));

    NamedCommands.registerCommand("Agitate", pivot.agitate().alongWith(wheels.runIntake(5)));

    NamedCommands.registerCommand(
        "Stop Intake", Commands.deadline(pivot.holdAngle(0.8).withTimeout(0.01), wheels.stop()));

    NamedCommands.registerCommand(
        "Stop Indexer", Commands.parallel(indexer.stop(), kicker.stop(), disruptor.stop()));
  }

  private void configureBindings() {
    // Basic drive controls
    drive.setDefaultCommand(
        new RotationLockedDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // X-Stop
    driverController.x().whileTrue(Commands.run(drive::stopWithX, drive));

    // Reset Gyro
    driverController
        .povDown()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Trench Drive
    driverController
        .b()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Rotation2d.kZero));

    // Run Intake
    driverController
        .leftTrigger()
        .whileTrue(wheels.in().alongWith(pivot.holdAngle(IntakeConstants.MIN_ANGLE)))
        .onFalse(wheels.stop().alongWith(pivot.holdAngle(0.875)));

    // Retract Intake
    driverController
        .leftBumper()
        .whileTrue(pivot.holdAngle(IntakeConstants.MAX_ANGLE))
        .onFalse(pivot.stop());

    driverController
        .povUp()
        .whileTrue(pivot.holdAngle(IntakeConstants.MAX_ANGLE))
        .onFalse(pivot.stop());

    driverController
        .povLeft()
        .whileTrue(pivot.agitate().alongWith(wheels.runIntake(5)))
        .onFalse(wheels.stop().alongWith(pivot.stop()));

    // Auto Shoot
    driverController
        .rightTrigger()
        .whileTrue(conductor.runState(ConductorState.TRACKED_FIRING))
        .and(readyToFire)
        .and(activeOrPassing)
        .whileTrue(Commands.parallel(indexer.pulseIn(), kicker.in(), disruptor.in()))
        .onFalse(Commands.parallel(indexer.stop(), kicker.stop(), disruptor.stop()));

    // Firing during inactive period
    driverController
        .rightTrigger()
        .and(() -> !HubShiftUtil.getShiftedShiftInfo().active())
        .onTrue(driverController.rumble(RumbleType.kBothRumble, 1.0).withTimeout(0.5));

    // Shift Timer Override
    driverController
        .rightBumper()
        .whileTrue(conductor.runState(ConductorState.TRACKED_FIRING))
        .and(readyToFire)
        .whileTrue(Commands.parallel(indexer.pulseIn(), kicker.in(), disruptor.in()))
        .onFalse(Commands.parallel(indexer.stop(), kicker.stop(), disruptor.stop()));

    // Operator override
    operatorController
        .rightTrigger()
        .whileTrue(antiJamFeed())
        .onFalse(Commands.parallel(indexer.stop(), kicker.stop()));
    operatorController
        .rightBumper()
        .whileTrue(
            Commands.parallel(indexer.out(), kicker.out(), wheels.out(), disruptor.reverse()))
        .onFalse(Commands.parallel(indexer.stop(), kicker.stop(), wheels.stop(), disruptor.stop()));

    // Turret Zero
    operatorController.back().onTrue(Commands.runOnce(() -> turret.zeroTurret()));

    // Manual Mode
    operatorController
        .start()
        .toggleOnTrue(conductor.forceManual().alongWith(Commands.runOnce(this::resetManual)));
    operatorController.povUp().whileTrue(Commands.run(() -> flywheelManual += 0.1));
    operatorController.povDown().whileTrue(Commands.run(() -> flywheelManual -= 0.1));
    operatorController
        .povLeft()
        .whileTrue(Commands.run(() -> turretManual -= Units.degreesToRadians(0.75)));
    operatorController
        .povRight()
        .whileTrue(Commands.run(() -> turretManual += Units.degreesToRadians(0.75)));

    // Location Triggers
    allianceZoneTrigger
        .and(driverController.rightTrigger().negate().and(driverController.rightBumper().negate()))
        .and(RobotModeTriggers.teleop())
        .whileTrue(conductor.runState(ConductorState.WARM_UP));

    // Shift Util Resets
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));

    // Shift Ending Rumble
    shiftAboutToEnd
        .and(RobotModeTriggers.teleop())
        .onTrue(driverController.rumble(RumbleType.kRightRumble, 1.0).withTimeout(0.25));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void setupDevelopmentRoutines() {
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Forward)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Reverse)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData(
        "Print Encoder Zeros",
        Commands.runOnce(() -> drive.printModuleAbsoluteAngles()).ignoringDisable(true));
  }

  public void updateAlerts() {
    driverControllerAlert.set(!driverController.isConnected());
    operatorControllerAlert.set(!operatorController.isConnected());
  }

  public void resetManual() {
    flywheelManual = flywheel.getVelocityMetersPerSec();
    turretManual = turret.getTurretPositionAsADouble();
  }

  public static boolean withinBounds(double value, double bound1, double bound2) {
    return value <= Math.max(bound1, bound2) && value >= Math.min(bound1, bound2);
  }

  private Command antiJamFeed() {
    return Commands.sequence(
            Commands.parallel(indexer.in(), disruptor.in(), kicker.in())
                .until(() -> indexer.isJammed() || disruptor.isJammed()),
            Commands.parallel(indexer.out(), disruptor.reverse(), kicker.out()).withTimeout(1.0),
            Commands.parallel(indexer.in(), disruptor.in(), kicker.in()))
        .repeatedly();
  }

  private Command pulsedAntiJamFeed() {
    return Commands.sequence(
            Commands.waitSeconds(1)
                .andThen(Commands.parallel(indexer.pulseIn(), kicker.in(), disruptor.in()))
                .until(() -> indexer.isJammed() || disruptor.isJammed()),
            Commands.parallel(indexer.out(), disruptor.reverse(), kicker.out()).withTimeout(1.0))
        .repeatedly();
  }

  /**
   * Create a command to warmup the pathfinder and pathfinding command
   *
   * @return Pathfinding warmup command
   */
  public static Command warmupCommand() {
    Logger.recordOutput("Pathfinding Ready", false);
    return new PathfindingCommand(
            new Pose2d(1.6, 4.0, Rotation2d.kZero),
            new PathConstraints(4, 3, 4, 4),
            () -> new Pose2d(1.5, 4, Rotation2d.kZero),
            ChassisSpeeds::new,
            (speeds, feedforwards) -> {},
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
            new RobotConfig(
                75,
                6.8,
                new ModuleConfig(
                    0.048, 15.0, 1.2, DCMotor.getKrakenX60(1).withReduction(6.14), 60.0, 1),
                0.55))
        .andThen(Commands.runOnce(() -> Logger.recordOutput("Pathfinding Ready", true)))
        .ignoringDisable(true);
  }
}
