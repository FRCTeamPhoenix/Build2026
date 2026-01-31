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
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.dumb.DumbMotorIO;
import org.team2342.lib.motors.dumb.DumbMotorIOInputsAutoLogged;

public class Indexer extends SubsystemBase {
  private final DumbMotorIO motor1;
  private final DumbMotorIO motor2;
  private final DumbMotorIO motor3;
  private final DumbMotorIOInputsAutoLogged inputs = new DumbMotorIOInputsAutoLogged();

  private final Alert motorAlert = new Alert("Indexer Motor is diconnected", AlertType.kError);

  public Indexer(DumbMotorIO motor1, DumbMotorIO motor2, DumbMotorIO motor3) {
    this.motor1 = motor1;
    this.motor2 = motor2;
    this.motor3 = motor3;
    setName("Indexer");

    setDefaultCommand(run(() -> motor1.runVoltage(0.0))); 
    setDefaultCommand(run(() -> motor2.runVoltage(0.0)));
    setDefaultCommand(run(() -> motor3.runVoltage(0.0)));
  }

  @Override
  public void periodic() {
    motor1.updateInputs(inputs);
    motor2.updateInputs(inputs);
    motor3.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    motorAlert.set(!inputs.connected);

    ExecutionLogger.log("Indexer");
  }

  public Command out() { 
    //return run(() -> motor1.runVoltage(10.0), motor2.runVoltage(10.0), motor3.runVoltage(10.0)).withName("Indexer Out");
    return run(() -> {motor1.runVoltage(10.0); motor2.runVoltage(10.0); motor3.runVoltage(10.0);}).withName("Indexer Out");

    

  }

  public Command in() {
    return run(() -> {motor1.runVoltage(-10.0); motor2.runVoltage(-10.0); motor3.runVoltage(-10.0);}).withName("Indexer In");
  }

  public Command stop() {
    return runOnce(() -> {motor1.runVoltage(0.0); motor2.runVoltage(0.0); motor3.runVoltage(0.0);}).withName("Indexer Stop");
  }
}


