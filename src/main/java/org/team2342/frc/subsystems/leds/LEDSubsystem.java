// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.leds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.subsystems.Conductor.ConductorState;
import org.team2342.lib.leds.LedIO;
import org.team2342.lib.leds.LedIO.Half;
import org.team2342.lib.leds.LedIO.LEDAnimation;
import org.team2342.lib.leds.LedIO.LEDEffect;
import org.team2342.lib.leds.LedIOInputsAutoLogged;
import org.team2342.lib.logging.ExecutionLogger;

public class LEDSubsystem extends SubsystemBase {
  private final LedIO io;
  private final String name;
  private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

  private final BooleanSupplier hasTags;
  private final Supplier<ConductorState> stateSupplier;

  private boolean hasEnabled = false;
  private Debouncer visionDebouncer = new Debouncer(0.5, DebounceType.kBoth);

  public LEDSubsystem(
      LedIO io, String name, BooleanSupplier hasTags, Supplier<ConductorState> stateSupplier) {
    this.io = io;
    this.name = name;
    this.hasTags = hasTags;
    this.stateSupplier = stateSupplier;
    setName(name);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    if (DriverStation.isEStopped()) {
      setAll(new LEDEffect(LEDAnimation.SOLID, Color.kBlack));
      return;
    }

    if (DriverStation.isTeleopEnabled()) {
      hasEnabled = true;
      LEDEffect activeEffect =
          switch (stateSupplier.get()) {
            case TRACKED_FIRING -> new LEDEffect(LEDAnimation.SOLID, Color.kCyan);
            case WARM_UP -> new LEDEffect(LEDAnimation.SOLID, Color.kYellow);
            case TUNING -> new LEDEffect(LEDAnimation.SOLID, Color.kPurple);
            default -> new LEDEffect(LEDAnimation.SOLID, Color.kOrange);
          };
      setAll(activeEffect);
    } else if (DriverStation.isAutonomousEnabled()) {
      hasEnabled = true;
      setAll(new LEDEffect(LEDAnimation.RAINBOW, Color.kWhite));
    } else {
      if (!visionDebouncer.calculate(hasTags.getAsBoolean()) && !hasEnabled) {
        setAll(new LEDEffect(LEDAnimation.SOLID, Color.kWhite));
      } else {
        if (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
          setAll(new LEDEffect(LEDAnimation.LARSON, Color.kRed));
        } else {
          setAll(new LEDEffect(LEDAnimation.LARSON, Color.kBlue));
        }
      }
    }

    ExecutionLogger.log(name);
  }

  public void setFirst(LEDEffect effect) {
    io.setEffect(LedIO.Half.FIRST, effect);
  }

  public void setSecond(LEDEffect effect) {
    io.setEffect(LedIO.Half.SECOND, effect);
  }

  public void setAll(LEDEffect effect) {
    io.setEffect(Half.ALL, effect);
  }
}
