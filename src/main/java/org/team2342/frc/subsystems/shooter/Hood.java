// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.ShooterConstants;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOInputsAutoLogged;
import org.team2342.lib.sensors.absolute.AbsoluteEncoderIO;

public class Hood extends SubsystemBase {
  private final SmartMotorIO motor;
  private final SmartMotorIOInputsAutoLogged motorInputs = new SmartMotorIOInputsAutoLogged();

  private final Alert motorAlert = new Alert("Hood motor is disconnected!", AlertType.kError);

  public Hood(SmartMotorIO motor, AbsoluteEncoderIO encoder) {
    this.motor = motor;

    setName("Shooter/Hood");
    setDefaultCommand(run(() -> motor.runVoltage(0.0)));
  }

  @Override
  public void periodic() {
    motor.updateInputs(motorInputs);
    Logger.processInputs("Shooter/Hood", motorInputs);

    motorAlert.set(!motorInputs.motorsConnected[0]);

    ExecutionLogger.log("Shooter/Hood");
  }

  public void runAngle(double targetAngle) {
    double clampedAngle = MathUtil.clamp(targetAngle, 0.0, ShooterConstants.MAX_ANGLE);
    Logger.recordOutput("Shooter/Hood/Setpoint", clampedAngle);

    motor.runPosition(clampedAngle);
  }

  public Command goToAngle(double targetAngle) {
    return run(() -> runAngle(targetAngle))
        .until(
            () -> Math.abs(targetAngle - motorInputs.positionRad) <= ShooterConstants.TARGET_TOLERANCE)
        .withName("Hood GoToAngle");
  }

  public Command holdAngle(double targetAngle) {
    return run(() -> runAngle(targetAngle)).withName("Hood HoldAngle");
  }

  public Command stop() {
    return runOnce(() -> motor.runVoltage(0.0)).withName("Hood Stop");
  }
}
