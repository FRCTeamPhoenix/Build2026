// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.TurretConstants;
import org.team2342.frc.RobotContainer;
import org.team2342.lib.logging.tunable.TunableNumber;
import org.team2342.lib.util.AllianceUtils;
import org.team2342.lib.util.EnhancedXboxController;

public class FiringSolver {
  private static FiringSolver instance;

  private PassingController controller = new PassingController(new EnhancedXboxController(1));

  private final TunableNumber passingSpeedOffset =
      new TunableNumber("PassingSpeedOffset", 2.5, () -> true);

  private static final int ITERATIONS = 5;

  public static final double MIN_TOF, MAX_TOF;

  private FiringSolution lastSolution = null;

  private static final InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingMap = new InterpolatingDoubleTreeMap();

  static {
    speedMap.put(1.8, 14.5);
    speedMap.put(2.01, 14.5);
    speedMap.put(2.2, 14.5);
    speedMap.put(2.4, 15.0);
    speedMap.put(2.6, 15.2);
    speedMap.put(2.8, 15.5);
    speedMap.put(3.0, 16.0);
    speedMap.put(3.21, 16.5);
    speedMap.put(3.4, 16.7);
    speedMap.put(3.6, 17.0);
    speedMap.put(3.8, 17.2);
    speedMap.put(4.0, 17.6);
    speedMap.put(4.2, 19.0);
    speedMap.put(4.4, 20.1);
    speedMap.put(4.6, 21.2);
    speedMap.put(4.8, 22.3);
    speedMap.put(5.0, 23.5);

    // old
    speedMap.put(5.166, 21.0);

    MIN_TOF = 5.49 - 4.73;
    MAX_TOF = 6.9 - 5.4;

    tofMap.put(1.8, 5.49 - 4.73);
    tofMap.put(2.01, 3.01 - 2.17);
    tofMap.put(2.2, 2.78 - 1.88);
    tofMap.put(2.4, 4.13 - 3.25);
    tofMap.put(2.6, 3.67 - 2.67);
    tofMap.put(2.8, 4.42 - 3.47);
    tofMap.put(3.0, 4.14 - 3.13);
    tofMap.put(3.21, 3.82 - 2.69);
    tofMap.put(3.4, 3.03 - 1.91);
    tofMap.put(3.6, 5.15 - 4.04);
    tofMap.put(3.8, 3.41 - 2.26);
    tofMap.put(4.0, 4.0 - 2.82);
    tofMap.put(4.2, 6.05 - 4.75);
    tofMap.put(4.4, 5.25 - 3.81);
    tofMap.put(4.6, 5.83 - 4.38);
    tofMap.put(4.8, 5.65 - 4.13);
    tofMap.put(5.0, 2.04 - 0.48);

    // old
    tofMap.put(5.166, 6.9 - 5.4);
  }

  public static FiringSolver getInstance() {
    if (instance == null) {
      instance = new FiringSolver();
    }
    return instance;
  }

  public FiringSolution calculate(ChassisSpeeds velocity, Pose2d position) {
    if (lastSolution != null) {
      return lastSolution;
    }

    boolean outsideAllianceZone =
        !RobotContainer.withinBounds(
            position.getX(),
            AllianceUtils.flipToAlliance(Pose2d.kZero).getX(),
            AllianceUtils.flipToAlliance(FieldConstants.LeftBump.nearLeftCorner).getX());

    if (outsideAllianceZone) {
      Pose2d turretPose =
          new Pose3d(position).transformBy(TurretConstants.TURRET_TRANSFORM).toPose2d();

      Translation2d turretTranslation = turretPose.getTranslation();

      Translation2d leftTarget = AllianceUtils.flipToAlliance(controller.getLeftTarget());

      Translation2d rightTarget = AllianceUtils.flipToAlliance(controller.getRightTarget());

      Logger.recordOutput("FiringSolver/LeftTarget", new Pose2d(leftTarget, Rotation2d.kZero));
      Logger.recordOutput("FiringSolver/RightTarget", new Pose2d(rightTarget, Rotation2d.kZero));

      Translation2d passTarget =
          (turretTranslation.getDistance(leftTarget) < turretTranslation.getDistance(rightTarget)
              ? leftTarget
              : rightTarget);

      Logger.recordOutput("FiringSolver/Target", new Pose2d(passTarget, Rotation2d.kZero));

      Translation2d turretToTarget = passTarget.minus(turretTranslation);

      Rotation2d turretAngle =
          turretToTarget.getAngle().minus(position.getRotation()).minus(Rotation2d.kCCW_Pi_2);
      Double turretDistance = turretToTarget.getNorm();

      // TODO: tune real passing speed
      lastSolution =
          new FiringSolution(
              turretAngle, speedMap.get(turretDistance) - passingSpeedOffset.get(), true);

      return lastSolution;
    }

    Translation2d hub =
        AllianceUtils.flipToAlliance(FieldConstants.Hub.topCenterPoint).toTranslation2d();
    Pose2d turretPose =
        new Pose3d(position).transformBy(TurretConstants.TURRET_TRANSFORM).toPose2d();
    double robotAngle = position.getRotation().getRadians();

    double velX =
        velocity.vxMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (TurretConstants.TURRET_TRANSFORM.getY() * Math.cos(robotAngle)
                    - TurretConstants.TURRET_TRANSFORM.getX() * Math.sin(robotAngle));
    double velY =
        velocity.vyMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (TurretConstants.TURRET_TRANSFORM.getX() * Math.cos(robotAngle)
                    - TurretConstants.TURRET_TRANSFORM.getY() * Math.sin(robotAngle));

    double tof;
    Pose2d predictedPose = turretPose;

    double predictedDistance = hub.getDistance(turretPose.getTranslation());
    Logger.recordOutput("FiringSolver/Distance", predictedDistance);
    for (int i = 0; i < ITERATIONS; i++) {
      tof = tofMap.get(predictedDistance);
      Logger.recordOutput("FiringSolver/PredictedTOF", tof);
      predictedPose =
          new Pose2d(
              turretPose.getTranslation().plus(new Translation2d(velX * tof, velY * tof)),
              turretPose.getRotation());
      predictedDistance = hub.getDistance(predictedPose.getTranslation());
    }
    Logger.recordOutput("FiringSolver/PredictedPose", predictedPose);
    Logger.recordOutput("FiringSolver/PredictedDistance", predictedDistance);

    Rotation2d turretAngle =
        hub.minus(predictedPose.getTranslation())
            .getAngle()
            .minus(position.getRotation())
            .minus(Rotation2d.kCCW_Pi_2)
            .plus(Rotation2d.fromDegrees(5));

    double wheelSpeed = speedMap.get(predictedDistance);

    lastSolution = new FiringSolution(turretAngle, wheelSpeed, false);

    return lastSolution;
  }

  public void clearCachedSolution() {
    lastSolution = null;
  }

  public record FiringSolution(Rotation2d turretAngle, double wheelSpeed, boolean passing) {}
}
