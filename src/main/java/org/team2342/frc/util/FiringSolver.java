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
import org.team2342.lib.util.AllianceUtils;

public class FiringSolver {
  private static FiringSolver instance;

  private static final int ITERATIONS = 5;

  public static final FiringSolution BUMPER_SHOT = new FiringSolution(new Rotation2d(), 23.5);

  private FiringSolution lastSolution = null;

  private static final InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

  // TODO: tune real maps
  static {
    speedMap.put(1.859, 15.0);
    speedMap.put(2.203, 15.0);
    speedMap.put(2.510, 17.0);
    speedMap.put(2.844, 18.5);
    speedMap.put(3.046, 19.0);
    speedMap.put(3.315, 20.5);
    speedMap.put(3.973, 20.5);
    speedMap.put(5.042, 20.0);

    tofMap.put(1.859, 1.0);
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
    for (int i = 0; i < ITERATIONS; i++) {
      tof = tofMap.get(predictedDistance);
      predictedPose =
          new Pose2d(
              turretPose.getTranslation().plus(new Translation2d(velX * tof, velY * tof)),
              turretPose.getRotation());
      predictedDistance = hub.getDistance(predictedPose.getTranslation());
    }
    Logger.recordOutput("FiringSolver/PredictedPose", predictedPose);
    Logger.recordOutput("FiringSolver/PredictedDistance", predictedDistance);

    Rotation2d turretAngle = hub.minus(predictedPose.getTranslation()).getAngle();
    turretAngle = turretAngle.minus(position.getRotation()).minus(Rotation2d.k180deg);

    double wheelSpeed = speedMap.get(predictedDistance);

    lastSolution = new FiringSolution(turretAngle, wheelSpeed);

    return lastSolution;
  }

  public void clearCachedSolution() {
    lastSolution = null;
  }

  public record FiringSolution(Rotation2d turretAngle, double wheelSpeed) {}
}
