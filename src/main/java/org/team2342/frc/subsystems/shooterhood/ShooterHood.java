// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.shooterhood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOInputsAutoLogged;
import org.team2342.lib.sensors.absolute.AbsoluteEncoderIO;
import org.team2342.lib.sensors.absolute.AbsoluteEncoderIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.HoodConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterHood extends SubsystemBase {
  private final SmartMotorIO motor;
  private final SmartMotorIOInputsAutoLogged motorInputs = new SmartMotorIOInputsAutoLogged();

  private final AbsoluteEncoderIO encoder;
  private final AbsoluteEncoderIOInputsAutoLogged encoderInputs =
      new AbsoluteEncoderIOInputsAutoLogged();

  private final Alert motorAlert = new Alert("Hood motor is disconnected!", AlertType.kError);
  private final Alert encoderAlert = new Alert("Hood Encoder is disconnected!", AlertType.kError);

  public ShooterHood(SmartMotorIO motor, AbsoluteEncoderIO encoder) {
    this.motor = motor;
    this.encoder = encoder;

    motorAlert.set(!motorInputs.motorsConnected[0]);
    encoderAlert.set(!encoderInputs.connected);

    setName("Shooter Hood");
    motorInputs.positionRad = 0.0;
    setDefaultCommand(run(() -> motor.runVoltage(0.0)));
  }

  @Override
  public void periodic() {
    motor.updateInputs(motorInputs);
    encoder.updateInputs(encoderInputs);

    Logger.processInputs("Shooter Hood/Motor", motorInputs);
    Logger.processInputs("Shooter Hood/encoder", encoderInputs);

    ExecutionLogger.log("Shooter Hood");
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(motorInputs.positionRad / HoodConstants.GEAR_RATIO);
  }

  public Command goToAngle(Rotation2d targetAngle) {
   return run(() -> motor.runPosition(targetAngle.getRadians()))
        .until(() -> Math.abs(targetAngle.getRadians() - getAngle().getRadians())
                    < HoodConstants.TARGET_TOLERANCE)
        .withName("Hood Go To Angle");
  }

  public Command holdAngle(Rotation2d targetAngle) {
    return run(() -> motor.runPosition(targetAngle.getRadians())).withName("Shooter Hold Angle Hold");
  }
  
  public Command stop() {
    return runOnce(() -> motor.runVoltage(0.0)).withName("Shooter Hood Stop");
  }
}
