// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.PivotConstants;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOInputsAutoLogged;

public class Pivot extends SubsystemBase {

  private final SmartMotorIO pivotMotor;
  private final SmartMotorIOInputsAutoLogged pivotMotorInputs = new SmartMotorIOInputsAutoLogged();

  public Pivot(SmartMotorIO pivotMotor) {
    this.pivotMotor = pivotMotor;
    setName("Pivot");
    setDefaultCommand(run(() -> pivotMotor.runPosition(PivotConstants.MIN_ANGLE)));
  }

  @Override
  public void periodic() {
    pivotMotor.updateInputs(pivotMotorInputs);
    Logger.processInputs("Intake/Pivot", pivotMotorInputs);
  }

  @AutoLogOutput(key = "Intake/Pivot/Angle")
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(pivotMotorInputs.positionRad);
  }

  public Command goToAngle(Rotation2d targetAngle) {
    return run(() -> pivotMotor.runPosition(targetAngle.getRadians()))
        .until(
            () -> {
              // Using minus() and getRadians() handles the wrap-around math for you!
              double error = targetAngle.minus(getAngle()).getRadians();
              return Math.abs(error) < PivotConstants.AT_TARGET_TOLERANCE;
            })
        .withName("Pivot Go To Angle"); // Fixed the quote here
  }

  public Command holdAngle(Rotation2d targetAngle) {
    return run(() -> pivotMotor.runPosition(targetAngle.getRadians())).withName("Pivot Hold Angle");
  }

  public Command stop() {
    return runOnce(() -> pivotMotor.runVoltage(0.0)).withName("Stop Pivot");
  }
}
