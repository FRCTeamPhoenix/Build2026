// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.team2342.lib.motors.MotorConfig;
import org.team2342.lib.motors.MotorConfig.IdleMode;
import org.team2342.lib.motors.smart.SmartMotorConfig;
import org.team2342.lib.motors.smart.SmartMotorConfig.ControlType;
import org.team2342.lib.motors.smart.SmartMotorConfig.FeedbackConfig;
import org.team2342.lib.pidff.PIDFFConfigs;
import org.team2342.lib.util.CameraParameters;

public final class Constants {
  public static final Mode CURRENT_MODE = Mode.REAL;
  public static final boolean TUNING = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class VisionConstants {
    public static final String SWERVE_CAMERA_NAME = "swerve_arducam";
    public static final String SHOOTER_CAMERA_NAME = "shooter_arducam";

    public static final Transform3d SWERVE_CAMERA_TRANSFORM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-7.883),
                Units.inchesToMeters(10.895),
                Units.inchesToMeters(8)),
            new Rotation3d(
                0, Units.degreesToRadians(-22.0), Units.degreesToRadians(-(90 + 61.475))));

    public static final Transform3d SHOOTER_CAMERA_TRANSFORM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-13.255),
                Units.inchesToMeters(11.9),
                Units.inchesToMeters(14.755)),
            new Rotation3d(
                0,
                Units.degreesToRadians((90 - 63.435 + 180)),
                Units.degreesToRadians(-119.745 + 90)));

    public static final CameraParameters SWERVE_CAMERA_PARAMETERS =
        CameraParameters.loadFromName(SWERVE_CAMERA_NAME, 800, 600)
            .withTransform(SWERVE_CAMERA_TRANSFORM);

    public static final CameraParameters SHOOTER_CAMERA_PARAMETERS =
        CameraParameters.loadFromName(SHOOTER_CAMERA_NAME, 800, 600)
            .withTransform(SHOOTER_CAMERA_TRANSFORM);

    // Basic filtering thresholds
    public static final double MAX_AMBIGUITY = 0.1;
    public static final double MAX_Z_ERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double LINEAR_STD_DEV_BASELINE = 0.06; // Meters
    public static final double ANGULAR_STD_DEV_BASELINE = 0.12; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static final double[] CAMERA_STD_DEV_FACTORS =
        new double[] {
          1.0, // Camera 0
          1.0, // Camera 1
          1.0 // Camera 2
        };

    // Multipliers to apply for MegaTag2/ConstrainedPNP observations
    public static final double LINEAR_STD_DEV_CONSTRAINED_FACTOR =
        0.5; // More stable than full 3D solve
    public static final double ANGULAR_STD_DEV_CONSTRAINED_FACTOR =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static final class DriveConstants {
    public static final double CONTROLLER_DEADBAND = 0.1;
    public static final double ROTATION_LOCK_TIME = 0.25;

    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(15.0);
    public static final double MAX_LINEAR_ACCELERATION = 20.0;
    public static final double DRIVE_GEARING = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double TURN_GEARING = 150.0 / 7.0;
    public static final double COUPLE_RATIO = 27.0 / 17.0 / 3;

    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
    public static final double WHEEL_COF = 1.2;

    public static final double TRACK_WIDTH_X = Units.inchesToMeters(27.0 - (2.625 * 2));
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(27.0 - (2.625 * 2));
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    // TODO: New Mass & MOI
    public static final double ROBOT_MASS_KG = Units.lbsToKilograms(112);
    public static final double ROBOT_MOI = 5.278;

    public static final double TURN_CURRENT_LIMIT = 30.0;
    public static final double SLIP_CURRENT_LIMIT = 70.0;
    public static final double DRIVE_SUPPLY_LIMIT = 40.0;
    public static final double MAX_MODULE_VELOCITY_RAD = Units.degreesToRadians(1080.0);

    public static final double[] ENCODER_OFFSETS = {
      0.12109375 + 0.5, -0.142333984375 + 0.5, -0.3896484375 + 0.5, -0.150146484375 + 0.5,
    };

    // Pitch, Roll, Yaw
    public static final double[] PIGEON_CALIBRATED_MOUNT_POSE = {0, 0, 0};

    public static final boolean IS_CANFD = false;
    public static final double ODOMETRY_FREQUENCY = IS_CANFD ? 250.0 : 100.0;
  }

  public static final class ConductorConstants {
    public static final double TRENCH_BUFFER = 0.1;
  }

  public static final class IndexerConstants {
    public static final double RUN_VOLTAGE = 5.0;
    public static final double DISRUPTOR_RUN_VOLTAGE = 5.0;
    public static final double RUN_CURRENT = 30.0;
    public static final double MAX_CURRENT = 45.0;

    public static final MotorConfig INDEXER_MOTOR_CONFIG =
        new MotorConfig()
            .withMotorInverted(true)
            .withSupplyCurrentLimit(40.0)
            .withStatorCurrentLimit(70.0)
            .withIdleMode(MotorConfig.IdleMode.COAST);

    public static final DCMotor INDEXER_SIM_MOTOR = DCMotor.getKrakenX60(1);
    public static final DCMotorSim INDEXER_SIM =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INDEXER_SIM_MOTOR, 0.003, 1), INDEXER_SIM_MOTOR);
    
    public static final MotorConfig DISRUPTOR_MOTOR_CONFIG =
        new MotorConfig()
            .withMotorInverted(false)
            .withSupplyCurrentLimit(30.0)
            .withStatorCurrentLimit(60.0)
            .withIdleMode(MotorConfig.IdleMode.BRAKE);

    public static final DCMotor DISRUPTOR_SIM_MOTOR = DCMotor.getNeoVortex(1);
    public static final DCMotorSim DISRUPTOR_SIM =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DISRUPTOR_SIM_MOTOR, 0.003, 1), DISRUPTOR_SIM_MOTOR);
  }

  public static final class IntakeConstants {
    public static final double RUN_VOLTAGE = 7.0;
    public static final double RUN_CURRENT = 10.0;

    public static final MotorConfig INTAKE_WHEELS_MOTOR_CONFIG =
        new MotorConfig()
            .withMotorInverted(true)
            .withSupplyCurrentLimit(30.0)
            .withStatorCurrentLimit(70.0)
            .withIdleMode(IdleMode.COAST);

    public static final DCMotor INTAKE_WHEELS_SIM_MOTOR = DCMotor.getKrakenX60(1);
    public static final DCMotorSim INTAKE_WHEEL_SIM =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INTAKE_WHEELS_SIM_MOTOR, 0.003, 1),
            INTAKE_WHEELS_SIM_MOTOR);

    public static final double PIVOT_GEAR_RATIO = 40;
    public static final double MIN_ANGLE = 0.1;
    public static final double MAX_ANGLE = 2.23;

    public static final PIDFFConfigs PIVOT_MOTOR_PID_CONFIGS =
        new PIDFFConfigs().withKP(45).withKI(15).withKD(7);
    public static final SmartMotorConfig PIVOT_MOTOR_CONFIG =
        new SmartMotorConfig()
            .withGearRatio(1.0)
            .withControlType(ControlType.PROFILED_POSITION)
            .withIdleMode(IdleMode.BRAKE)
            .withMotorInverted(false)
            .withSupplyCurrentLimit(40.0)
            .withFeedbackConfig(
                FeedbackConfig.fused(
                    CANConstants.INTAKE_PIVOT_ENCODER_ID, PIVOT_GEAR_RATIO, 0.681, true))
            .withProfileConstraintsRad(
                new TrapezoidProfile.Constraints(
                    Units.degreesToRadians(1800), Units.degreesToRadians(540)));
  }

  public static final class ShooterConstants {
    public static final double FLYWHEEL_GEAR_RATIO = 1.0;
    public static final double FLYWHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

    public static final double IDLE_SPEED = 15.0;
    public static final double FLYWHEEL_AT_GOAL_TOLERANCE = 3.0;

    public static final PIDFFConfigs FLYWHEEL_PID_CONFIGS =
        new PIDFFConfigs()
            .withKP(0.575)
            .withKI(0.6)
            .withKS(0.15989)
            .withKV(0.019968)
            .withKA(0.005977);

    public static final SmartMotorConfig FLYWHEEL_CONFIG =
        new SmartMotorConfig()
            .withControlType(ControlType.VELOCITY)
            .withGearRatio(FLYWHEEL_GEAR_RATIO)
            .withMotorInverted(true)
            .withSupplyCurrentLimit(50)
            .withStatorCurrentLimit(80)
            .withProfileConstraintsRad(new TrapezoidProfile.Constraints(1000, 1000));
    public static final DCMotor FLYWHEEL_SIM_MOTOR = DCMotor.getKrakenX60(1);
    public static final DCMotorSim FLYWHEEL_SIM =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(FLYWHEEL_SIM_MOTOR, 0.03, FLYWHEEL_GEAR_RATIO),
            FLYWHEEL_SIM_MOTOR);

    public static final double KRAKEN_TO_ENCODER = (64.0 / 14.0) * (46.0 / 20.0);
    public static final double ENCODER_TO_HOOD = 344.0 / 22.0;
    public static final double MAX_ANGLE = 0.273;
    public static final double TARGET_TOLERANCE = 0.01;
  }

  public static final class KickerConstants {
    public static final double RUN_VOLTAGE = 7.5;

    public static final MotorConfig KICKER_CONFIG =
        new MotorConfig()
            .withMotorInverted(true)
            .withSupplyCurrentLimit(30.0)
            .withStatorCurrentLimit(60.0)
            .withIdleMode(MotorConfig.IdleMode.BRAKE);
  }

  public static final class TurretConstants {
    public static final Transform3d TURRET_TRANSFORM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-6),
                Units.inchesToMeters(0.002),
                Units.inchesToMeters(14.165)),
            new Rotation3d(Rotation2d.k180deg));

    public static final double AT_POSITION_THRESHOLD = 0.01;

    public static final double STARTING_ANGLE = Units.degreesToRadians(90);
    public static final double MIN_TURRET_ANGLE = Units.degreesToRadians(0);
    public static final double MAX_TURRET_ANGLE = Units.degreesToRadians(359);

    public static final double GEAR_RATIO = (48.0 / 12.0) * (100.0 / 10.0);

    public static final SmartMotorConfig TURRET_CONFIG =
        new SmartMotorConfig()
            .withControlType(ControlType.PROFILED_POSITION)
            .withGearRatio(GEAR_RATIO)
            .withIdleMode(IdleMode.BRAKE)
            .withProfileConstraintsRad(new TrapezoidProfile.Constraints(12 * Math.PI, 10 * Math.PI))
            .withSupplyCurrentLimit(40);

    public static final PIDFFConfigs PID_CONFIG = new PIDFFConfigs().withKP(100).withKI(10);

    public static final DCMotor TURRET_SIM_MOTOR = DCMotor.getKrakenX60(1);
    public static final DCMotorSim TURRET_SIM =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(TURRET_SIM_MOTOR, 0.003, 100.0 / 16.0),
            TURRET_SIM_MOTOR);
  }

  public static final class CANConstants {
    public static final int PIGEON_ID = 13;
    public static final int[] FL_IDS = {1, 5, 9};
    public static final int[] FR_IDS = {2, 6, 10};
    public static final int[] BL_IDS = {3, 7, 11};
    public static final int[] BR_IDS = {4, 8, 12};

    public static final int FLYWHEEL_MOTOR_ID = 14;
    public static final int KICKER_ID = 15;
    public static final int TURRET_ID = 16;
    public static final int INDEXER_MOTOR_ID = 17;

    public static final int INTAKE_WHEEL_MOTOR_ID = 18;
    public static final int INTAKE_PIVOT_MOTOR_ID = 19;
    public static final int INTAKE_PIVOT_ENCODER_ID = 20;

    public static final int CANDLE_ID = 61;
    public static final int PDH_ID = 62;
  }
}
