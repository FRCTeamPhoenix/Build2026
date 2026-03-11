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
import org.team2342.lib.util.AllianceUtils;

public class FiringSolver {
  private static FiringSolver instance;

  private static final int ITERATIONS = 5;

  public static final double MIN_TOF, MAX_TOF;

  private FiringSolution lastSolution = null;

  private static final InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

  // TODO: tune real maps
  static {
    speedMap.put(1.141, 15.0);
    speedMap.put(1.445, 17.0);

    MIN_TOF = 1.0;
    MAX_TOF = 1.0;

    tofMap.put(1.141, 12.66 - 11.46);
    tofMap.put(1.445, 8.34 - 6.95);
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

      Translation2d leftBump = AllianceUtils.flipToAlliance(FieldConstants.LeftBump.nearLeftCorner);

      Translation2d rightBump =
          AllianceUtils.flipToAlliance(FieldConstants.RightBump.farLeftCorner);

      Logger.recordOutput("FiringSolver/LeftTarget", new Pose2d(leftBump, Rotation2d.kZero));
      Logger.recordOutput("FiringSolver/RightTarget", new Pose2d(rightBump, Rotation2d.kZero));

      Translation2d passTarget =
          (turretTranslation.getDistance(leftBump) < turretTranslation.getDistance(rightBump)
              ? leftBump
              : rightBump);

      Logger.recordOutput("FiringSolver/Target", new Pose2d(passTarget, Rotation2d.kZero));

      Rotation2d turretAngle =
          passTarget
              .minus(turretTranslation)
              .getAngle()
              .minus(position.getRotation())
              .minus(Rotation2d.kCCW_Pi_2);

      // TODO: tune real passing speed
      lastSolution = new FiringSolution(turretAngle, 15.0, true);

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
            .minus(Rotation2d.kCCW_Pi_2);

    double wheelSpeed = speedMap.get(predictedDistance);

    lastSolution = new FiringSolution(turretAngle, wheelSpeed, false);

    return lastSolution;
  }

  public void clearCachedSolution() {
    lastSolution = null;
  }

  public record FiringSolution(Rotation2d turretAngle, double wheelSpeed, boolean passing) {}
}
