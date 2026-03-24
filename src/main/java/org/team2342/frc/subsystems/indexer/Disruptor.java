// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.IndexerConstants;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.dumb.DumbMotorIO;
import org.team2342.lib.motors.dumb.DumbMotorIOInputsAutoLogged;

public class Disruptor extends SubsystemBase {
  private final DumbMotorIO motor;
  private final DumbMotorIOInputsAutoLogged inputs = new DumbMotorIOInputsAutoLogged();

  private final Alert motorAlert = new Alert("Disruptor Motor is diconnected", AlertType.kError);

  public Disruptor(DumbMotorIO motor) {
    this.motor = motor;
    setName("Disruptor");

    setDefaultCommand(run(() -> motor.runVoltage(0.0)));
  }

  @Override
  public void periodic() {
    motor.updateInputs(inputs);
    Logger.processInputs("Disruptor", inputs);
    motorAlert.set(!inputs.connected);
    ExecutionLogger.log("Disruptor");
  }

  public Command in() {
    return run(() -> motor.runVoltage(IndexerConstants.DISRUPTOR_RUN_VOLTAGE))
        .withName("Disruptor Feed");
  }

  public Command reverse() {
    return run(() -> motor.runVoltage(-IndexerConstants.DISRUPTOR_RUN_VOLTAGE))
        .withName("Disruptor Reverse");
  }

  public Command stop() {
    return run(() -> motor.runVoltage(0.0)).withName("Disruptor Stop");
  }

  public boolean isJammed() {
    return inputs.currentAmps > IndexerConstants.MAX_CURRENT;
  }
}
