// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.IntakeConstants;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.dumb.DumbMotorIO;
import org.team2342.lib.motors.dumb.DumbMotorIOInputsAutoLogged;

public class Wheels extends SubsystemBase {
  private final DumbMotorIO motor;
  private final DumbMotorIOInputsAutoLogged motorInputs = new DumbMotorIOInputsAutoLogged();

  private final Alert motorAlert = new Alert("Wheels are disconnected!", AlertType.kError);

  public Wheels(DumbMotorIO motor) {
    this.motor = motor;
    setName("Intake/Wheels");
    setDefaultCommand(run(() -> motor.runVoltage(0.0)));
  }

  @Override
  public void periodic() {
    motor.updateInputs(motorInputs);
    Logger.processInputs("Intake/Wheels", motorInputs);
    motorAlert.set(!motorInputs.connected);
    ExecutionLogger.log("Intake/Wheels");
  }

  public Command runIntake(double voltage) {
    return run(() -> motor.runVoltage(voltage)).withName("Intake Wheels Run");
  }

  public Command runIntakeAmps(double amps) {
    return run(() -> motor.runTorqueCurrent(amps)).withName("Intake Wheels Run Amps");
  }

  public Command in() {
    return runIntake(IntakeConstants.RUN_VOLTAGE).withName("Intake Wheels In");
  }

  public Command inAmps() {
    return runIntakeAmps(IntakeConstants.RUN_CURRENT).withName("Intake Wheels In Amps");
  }

  public Command out() {
    return runIntake(-IntakeConstants.RUN_VOLTAGE).withName("Intake Wheels Out");
  }

  public Command stop() {
    return runOnce(() -> motor.runVoltage(0)).withName("Intake Wheels Stop");
  }
}
