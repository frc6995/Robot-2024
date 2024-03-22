// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

public class LightStripS {

  private static LightStripS m_instance = new LightStripS();
  private static final int STRIP_LENGTH = 39;
  private AddressableLED led = new AddressableLED(9);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(STRIP_LENGTH);
  private final PersistentLedState persistentLedState = new PersistentLedState();
  private static final double[][] defaultNoteAngles = new double[3][2];
  private Supplier<double[][]> getNoteAngles = ()->defaultNoteAngles;
  private States previousState = States.Default;

  private static class PersistentLedState {
    public int rainbowFirstPixelHue = 0;
    public double pulseOffset = 0;
  }

  /** Creates a new LightStripS. */
  private LightStripS() {
    led.setLength(buffer.getLength());

    States.Disabled.setter.accept(buffer, persistentLedState);
    led.setData(buffer);
    led.start();
  }
  public static void setNoteAngles(Supplier<double[][]> sup) {
    m_instance.getNoteAngles = sup;
  }

  public static LightStripS getInstance() {
    return m_instance;
  }

  private TreeSet<States> m_states = new TreeSet<>();

  /**
   * Different states of the robot, states placed higher in the list have higher priority
   */
  public static enum States {
    CoastMode(setColor(0, 0, 64)),
    SetupDone(setColor(0, 64, 0)), // set in robotPeriodic
    Disabled(setColor(64, 0, 0)), // set in robotPeriodic
    Error(pulse(0.25, setColor(255, 0, 0))),
    Climbing(
        (ledBuffer, persistentState) -> {
          for (int i = 0; i < ledBuffer.getLength(); i++) {
            final int hue =
                (persistentState.rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            ledBuffer.setHSV(i, hue, 255, 255);
          }
          persistentState.rainbowFirstPixelHue += 3;
          persistentState.rainbowFirstPixelHue %= 180;
        }), // set through triggers in RobotContainer
    IntakedNote(pulse(0.25, setColor(245, 224, 66))),
    Scoring(setColor(0, 0, 255)),
    AutoAlign(
      (ledBuffer, persistentState) -> {
          for (int i = 0; i < ledBuffer.getLength(); i++) {
            final int hue =
                (persistentState.rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            ledBuffer.setHSV(i, hue - 100, 255, 255);
          }
          persistentState.rainbowFirstPixelHue += 3;
          persistentState.rainbowFirstPixelHue %= 180;
        }),
      Default(
        (ledBuffer, persistentState) -> {
          for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (i % 2 == 0) {
              ledBuffer.setRGB(i, 0, 64, 0);
            
              } else {
                ledBuffer.setRGB(i, 0, 0, 0);        
            }
          }
        }
      );
    //Default(setColor(0, 255, 0));

    public final BiConsumer<AddressableLEDBuffer, PersistentLedState> setter;

    private States(BiConsumer<AddressableLEDBuffer, PersistentLedState> setter) {
      this.setter = setter;
    }
  }

  // currentStates = {Disabled, Climbing, EjectingWrongColor, Intaking, Shooting,
  // Default}

  /**
   * Requests the current state of the robot, determines whether the requested state is a higher
   * priority than the current state, sets the current state to the requested state
   *
   * @param state The requested state of the robot when the method is called
   */
  public void requestState(States state) {
    m_states.add(state);
  }

  public Command stateC(Supplier<States> state) {
    return Commands.run(() -> requestState(state.get())).ignoringDisable(true);
  }

  /**
   * Periodically checks the current state of the robot and sets the LEDs to the corresponding light
   * pattern
   */
  public void periodic() {
    requestState(States.Default);
    // if (DriverStation.isDisabled()) {
    //   requestState(States.Disabled);
    // }
    States state = m_states.first();
    if (state != previousState) {
      persistentLedState.pulseOffset = 0;
    }
    // spark.set(m_states.first().lightSpeed);
    state.setter.accept(buffer, persistentLedState);
    previousState = state;
    // Do other things with the buffer

    var noteAngles = getNoteAngles.get();
    // angles 
    for (int i = noteAngles.length - 1; i >= 0; i--) {
      if (i < 0) continue;
      var angle = noteAngles[i][0];
      // 
      var width = noteAngles[i][1];
      if (angle < -90) {
        continue;
      }
      double proportion = angle / 35.0;
      int ctrLED = (int) ((STRIP_LENGTH / 2) - (proportion * STRIP_LENGTH / 2));
      int halfWidth = Math.max(2, (int) (width * STRIP_LENGTH / 200));
      for (int j = Math.max(0, ctrLED - halfWidth); j < Math.min(STRIP_LENGTH, ctrLED + halfWidth); j++){
        buffer.setRGB(j, 255, 50, 0);
      }
    }
    led.setData(buffer);
    m_states.removeAll(Set.of(States.values()));
  }

  private static BiConsumer<AddressableLEDBuffer, PersistentLedState> setColor(
      int r, int g, int b) {
    return (buffer, state) -> {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setRGB(i, r, g, b);
      }
    };
  }

  private static BiConsumer<AddressableLEDBuffer, PersistentLedState> barFill(
      BiConsumer<AddressableLEDBuffer, PersistentLedState> start,
      BiConsumer<AddressableLEDBuffer, PersistentLedState> end) {
    return (buffer, state) -> {
      for (int i = 0; i < buffer.getLength(); i++) {
        if (i > state.pulseOffset) {
          start.accept(buffer, state);
        } else {
          end.accept(buffer, state);
        }
      }
      state.pulseOffset++;
    };
  }

  private static BiConsumer<AddressableLEDBuffer, PersistentLedState> chase(
    double speed, int width, Color8Bit onColor, Color8Bit offColor) {
      return (buffer, state) -> {
        if (state.pulseOffset > buffer.getLength()) {
          state.pulseOffset = 0;
        }
        for (int i = 0; i < buffer.getLength(); i++) {
          if (i >= state.pulseOffset && i < (state.pulseOffset + width)) {
              buffer.setRGB(i, onColor.red, onColor.green, onColor.blue);
          }
          else {
              buffer.setRGB(i, offColor.red, offColor.green, offColor.blue);
          }
        }
        state.pulseOffset += speed;
      };
  }

  private static BiConsumer<AddressableLEDBuffer, PersistentLedState> pulse(
      double period, BiConsumer<AddressableLEDBuffer, PersistentLedState> pattern) {
    return (buffer, state) -> {
      if (Timer.getFPGATimestamp() % period < 0.5 * period) {
        pattern.accept(buffer, state);
      } else {
        for (int i = 0; i < buffer.getLength(); i++) {
          buffer.setRGB(i, 0, 0, 0);
        }
      }
    };
  }
}
