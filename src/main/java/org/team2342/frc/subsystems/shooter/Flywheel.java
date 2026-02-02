// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.ShooterConstants;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOInputsAutoLogged;

public class Flywheel extends SubsystemBase {

  private final SmartMotorIO motor;
  private final SmartMotorIOInputsAutoLogged motorInputs = new SmartMotorIOInputsAutoLogged();

  private final Alert motorAlert = new Alert("Flywheel motor is disconnected!", AlertType.kError);

  public Flywheel(SmartMotorIO motor) {
    this.motor = motor;

    setName("Shooter/Flywheel");
    setDefaultCommand(run(() -> motor.runVoltage(0.0)));
  }

  @Override
  public void periodic() {
    motor.updateInputs(motorInputs);
    Logger.processInputs("Shooter/Flywheel", motorInputs);

    motorAlert.set(!motorInputs.motorsConnected[0]);

    ExecutionLogger.log("Shooter/Flywheel");
  }

  public void runVelocity(double metersPerSec) {
    double radPerSec = metersPerSec / ShooterConstants.FLYWHEEL_RADIUS_METERS;
    Logger.recordOutput("Shooter/Flywheel/SetpointMetersPerSec", metersPerSec);
    motor.runVelocity(radPerSec);
  }

  public Command shoot(double metersPerSec) {
    return run(() -> runVelocity(metersPerSec)).withName("Run Shooter");
  }

  public Command stop() {
    return runOnce(() -> motor.runVoltage(0.0)).withName("Shooter Stop");
  }

  @AutoLogOutput(key = "Shooter/Flywheel/VelocityMetersPerSec")
  public double getVelocityMetersPerSec() {
    return motorInputs.velocityRadPerSec * ShooterConstants.FLYWHEEL_RADIUS_METERS;
  }
}
