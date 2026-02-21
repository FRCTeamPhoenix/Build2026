// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.team2342.frc.Constants.CANConstants;
import org.team2342.frc.Constants.ConductorConstants;
import org.team2342.frc.Constants.DriveConstants;
import org.team2342.frc.Constants.IndexerConstants;
import org.team2342.frc.Constants.IntakeConstants;
import org.team2342.frc.Constants.ShooterConstants;
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
import org.team2342.frc.subsystems.indexer.Indexer;
import org.team2342.frc.subsystems.intake.Wheels;
import org.team2342.frc.subsystems.shooter.Flywheel;
import org.team2342.frc.subsystems.shooter.Hood;
import org.team2342.frc.subsystems.vision.Vision;
import org.team2342.frc.subsystems.vision.VisionIO;
import org.team2342.frc.subsystems.vision.VisionIOPhoton;
import org.team2342.frc.subsystems.vision.VisionIOSim;
import org.team2342.frc.util.FieldConstants;
import org.team2342.frc.util.FiringSolver;
import org.team2342.lib.motors.dumb.DumbMotorIO;
import org.team2342.lib.motors.dumb.DumbMotorIOSim;
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
  @Getter private final Indexer indexer;
  @Getter private final Wheels wheels;
  @Getter private final Flywheel flywheel;
  @Getter private final Hood hood;

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

  private final Trigger trenchTrigger;
  private final Trigger allianceZoneTrigger;

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
                    VisionConstants.SWERVE_CAMERA_PARAMETERS,
                    PoseStrategy.CONSTRAINED_SOLVEPNP,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));
        indexer =
            new Indexer(
                new DumbMotorIOTalonFXFOC(
                    CANConstants.INDEXER_BELT_ID, IndexerConstants.INDEXER_BELT_CONFIG),
                new DumbMotorIOTalonFXFOC(
                    CANConstants.INDEXER_FEEDER_ID, IndexerConstants.INDEXER_FEEDER_CONFIG));

        wheels =
            new Wheels(
                new DumbMotorIOTalonFXFOC(
                    CANConstants.INTAKE_WHEEL_MOTOR_ID,
                    IntakeConstants.INTAKE_WHEELS_MOTOR_CONFIG));
        flywheel =
            new Flywheel(
                new SmartMotorIOTalonFX(
                    CANConstants.FLYWHEEL_MOTOR_ID,
                    ShooterConstants.FLYWHEEL_CONFIG.withPIDFFConfigs(
                        ShooterConstants.FLYWHEEL_PID_CONFIGS)));
        hood =
            new Hood(
                new SmartMotorIOTalonFX(
                    CANConstants.HOOD_MOTOR_ID,
                    ShooterConstants.HOOD_MOTOR_CONFIG.withPIDFFConfigs(
                        ShooterConstants.HOOD_MOTOR_PID_CONFIGS)));

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
                    VisionConstants.SWERVE_CAMERA_PARAMETERS,
                    PoseStrategy.CONSTRAINED_SOLVEPNP,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    drive::getRawOdometryPose),
                new VisionIOSim(
                    VisionConstants.SHOOTER_CAMERA_PARAMETERS,
                    PoseStrategy.CONSTRAINED_SOLVEPNP,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    drive::getRawOdometryPose));
        indexer =
            new Indexer(
                new DumbMotorIOSim(
                    IndexerConstants.INDEXER_BELT_SIM_MOTOR, IndexerConstants.INDEXER_BELT_SIM),
                new DumbMotorIOSim(
                    IndexerConstants.INDEXER_FEEDER_SIM_MOTOR,
                    IndexerConstants.INDEXER_FEEDER_SIM));

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
        hood =
            new Hood(
                new SmartMotorIOSim(
                    ShooterConstants.HOOD_MOTOR_CONFIG.withPIDFFConfigs(
                        new PIDFFConfigs().withKP(1)),
                    ShooterConstants.HOOD_SIM_MOTOR,
                    ShooterConstants.HOOD_SIM,
                    1));

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
        indexer = new Indexer(new DumbMotorIO() {}, new DumbMotorIO() {});
        wheels = new Wheels(new DumbMotorIO() {});
        flywheel = new Flywheel(new SmartMotorIO() {});
        hood = new Hood(new SmartMotorIO() {});

        break;
    }

    conductor = new Conductor(flywheel, hood, drive::getPose, drive::getChassisSpeeds);

    configureNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.get();

    if (Constants.TUNING) setupDevelopmentRoutines();

    SmartDashboard.putData(
        "Calculate Vision Heading Offset",
        Commands.runOnce(() -> drive.calculateVisionHeadingOffset())
            .alongWith(Commands.print("Calculated Vision Offset"))
            .ignoringDisable(true));

    trenchTrigger =
        new Trigger(
            () ->
                withinBounds(
                    drive.getPose().getX(),
                    AllianceUtils.flipToAlliance(FieldConstants.LeftBump.nearLeftCorner).getX()
                        + ConductorConstants.TRENCH_BUFFER,
                    AllianceUtils.flipToAlliance(FieldConstants.LeftBump.farLeftCorner).getX()
                        - ConductorConstants.TRENCH_BUFFER));
    allianceZoneTrigger =
        new Trigger(
            () ->
                withinBounds(
                    drive.getPose().getX(),
                    AllianceUtils.flipToAlliance(Pose2d.kZero).getX(),
                    AllianceUtils.flipToAlliance(FieldConstants.LeftBump.nearLeftCorner).getX()
                        + ConductorConstants.TRENCH_BUFFER));

    configureBindings();
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Named Command Test", Commands.print("Named Command Test"));
  }

  private void configureBindings() {
    // Basic drive controls
    drive.setDefaultCommand(
        new RotationLockedDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    driverController.leftTrigger().whileTrue(wheels.in()).onFalse(wheels.stop());

    // Shooting Controls
    driverController
        .rightBumper()
        .whileTrue(
            conductor
                .goToState(ConductorState.BUMPER_SHOT)
                .andThen(conductor.runState(ConductorState.BUMPER_SHOT).alongWith(indexer.feed())));

    driverController
        .rightTrigger()
        .whileTrue(
            conductor
                .goToState(ConductorState.TRACKED_FIRING)
                .andThen(
                    conductor.runState(ConductorState.TRACKED_FIRING).alongWith(indexer.feed()))
                .alongWith(
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () ->
                            FiringSolver.getInstance()
                                .calculate(drive.getChassisSpeeds(), drive.getPose())
                                .turretAngle())));

    //Operator Overrides
    operatorController.povRight().whileTrue(indexer.feed()).onFalse(indexer.stop());
    operatorController.povLeft().whileTrue(indexer.out()).onFalse(indexer.stop());

    operatorController.leftBumper().whileTrue(wheels.out()).onFalse(wheels.stop());
    operatorController.leftTrigger().whileTrue(wheels.in()).onFalse(wheels.stop());

    operatorController.rightTrigger()
        .whileTrue(conductor.runState(ConductorState.OVERRIDE_25));

    operatorController.rightBumper()
        .whileTrue(conductor.runState(ConductorState.OVERRIDE_23));
    // Location Triggers
    trenchTrigger
        .and(driverController.rightTrigger().negate().and(driverController.rightBumper().negate()))
        .whileTrue(conductor.runState(ConductorState.TRENCH));
    allianceZoneTrigger
        .and(driverController.rightTrigger().negate().and(driverController.rightBumper().negate()))
        .whileTrue(conductor.runState(ConductorState.WARM_UP));
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

  private boolean withinBounds(double value, double bound1, double bound2) {
    return value <= Math.max(bound1, bound2) && value >= Math.min(bound1, bound2);
  }
}
