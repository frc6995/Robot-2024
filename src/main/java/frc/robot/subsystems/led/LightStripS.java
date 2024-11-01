// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static frc.robot.subsystems.led.LEDPattern.*;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class LightStripS {

  private static LightStripS m_instance = new LightStripS();
  private static final int STRIP_LENGTH = 21;
  private AddressableLED led = new AddressableLED(0);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(44);
  private AddressableLEDBufferView m_topBar = new AddressableLEDBufferView(buffer, 21, 43).reversed();
  private AddressableLEDBufferView m_bottomBar = new AddressableLEDBufferView(buffer, 0, 20);
  private AddressableLEDBufferView m_leftShooterBar = new AddressableLEDBufferView(m_bottomBar, 0, STRIP_LENGTH/2 - 3);
  private AddressableLEDBufferView m_centerIndicator = new AddressableLEDBufferView(m_bottomBar, STRIP_LENGTH/2 - 2, STRIP_LENGTH/2 + 1);
  private AddressableLEDBufferView m_rightShooterBar = new AddressableLEDBufferView(m_bottomBar, STRIP_LENGTH/2 + 2, STRIP_LENGTH-1).reversed();

  private States previousState = States.Default;

  private DoubleSupplier shooterLeftSpeedPercent = ()->0;
  private DoubleSupplier shooterRightSpeedPercent = ()->0;
  private BooleanSupplier hasNote=()->false;
  private LEDPattern shooterBar(DoubleSupplier percent, int length) {
    LEDPattern notAtSetpoint = solid(Color.kGray);
    LEDPattern atSetpoint = solid(Color.kGreen);
    LEDPattern base = (reader, writer)->{
      var perc = percent.getAsDouble();
      if (perc > 0.98 && perc < 1.02) {
        atSetpoint.applyTo(reader, writer);
      } else {
        notAtSetpoint.applyTo(reader, writer);
      }
    };
    return base.mask(LEDPattern.progressMaskLayer(percent));
  }
  // Intentionally wrapping the double supplier so we can change it later.
  private LEDPattern leftBar = shooterBar(()->shooterLeftSpeedPercent.getAsDouble(), m_leftShooterBar.getLength());
  private LEDPattern rightBar = shooterBar(()->shooterRightSpeedPercent.getAsDouble(), m_rightShooterBar.getLength());
  private LEDPattern center = LEDPattern.solid(Color.kOrange).synchronizedBlink(hasNote).overlayOn(kOff);
  public void setShooterLeftSpeedPercent(DoubleSupplier percent) {
    shooterLeftSpeedPercent = percent;
  }
  public void setShooterRightSpeedPercent(DoubleSupplier percent) {
    shooterRightSpeedPercent = percent;
  }
  public void setCenterPattern(BooleanSupplier isEnabled, BooleanSupplier hasNote, BooleanSupplier isHomed, BooleanSupplier seesTag) {
    
    LEDPattern hasNoteP = solid(Color.kOrange);
    LEDPattern enabled = hasNoteP.synchronizedBlink(hasNote);
    LEDPattern disabled = (reader, writer) -> {
      if (!isHomed.getAsBoolean()) {
        writer.setLED(0, Color.kRed);
      }
      if (seesTag.getAsBoolean()) {
        writer.setLED(reader.getLength()-1, Color.kWhite);
      }
    };
    center = disabled.synchronizedBlink(()->!isEnabled.getAsBoolean()).overlayOn(enabled);
  }
  /** Creates a new LightStripS. */
  private LightStripS() {
    led.setLength(buffer.getLength());

    States.Disabled.setter.applyTo(m_topBar);
    States.Disabled.setter.applyTo(m_bottomBar);
    led.setData(buffer);
    led.start();
  }

  public static LightStripS getInstance() {
    return m_instance;
  }

  private TreeSet<States> m_states = new TreeSet<>();

  /**
   * Different states of the robot, states placed higher in the list have higher priority
   */
  public static enum States {
    CoastMode(solid(Color.kBlue).atBrightness(Value.of(0.25))),
    
    SetupDone(solid(Color.kGreen).atBrightness(Value.of(0.25))), // set in robotPeriodic
    Disabled(solid(Color.kRed).atBrightness(Value.of(0.25))), // set in robotPeriodic
    Error(solid(Color.kRed).blink(Seconds.of(0.125))),
    LeftThird(leftThird(64, 0, 0)),
    CenterThird(centerThird(64, 64, 64)),
    RightThird(rightThird(64, 0, 0)),
    Climbing(rainbow(255, 255)),
    HasNote(solid(new Color(245, 224, 66))),
    IntakedNote(solid(new Color(245, 224, 66)).blink(Seconds.of(0.125))),
    Scoring(solid(Color.kBlue)),
    AutoAlign(rainbow(255, 255)),
    Passing(solid(Color.kYellow).atBrightness(Value.of(0.25))),
      Default(
        solid(Color.kGreen).atBrightness(Value.of(0.25))
      );
    //Default(setColor(0, 255, 0));

    public final LEDPattern setter;

    private States(LEDPattern setter) {
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
    // spark.set(m_states.first().lightSpeed);
    state.setter.applyTo(m_topBar);
    leftBar.applyTo(m_leftShooterBar);
    rightBar.applyTo(m_rightShooterBar);
    center.applyTo(m_centerIndicator);
    previousState = state;
    // Do other things with the buffer

    led.setData(buffer);
    m_states.removeAll(Set.of(States.values()));
  }

  private static LEDPattern leftThird(int r, int g, int b) {
    Color color = new Color(r, g, b);
    LEDPattern pattern = LEDPattern.steps(Map.of(0, color, 1.0 / 3, Color.kBlack));
    return pattern;
  }
    private static LEDPattern centerThird(int r, int g, int b) {
      Color color = new Color(r, g, b);
      LEDPattern pattern = LEDPattern.steps(
        Map.of(0, Color.kBlack, 1.0 / 3, color, 2.0 / 3, Color.kBlack));
      return pattern;
  }

  private static LEDPattern rightThird(int r, int g, int b) {
  
      Color color = new Color(r, g, b);
      LEDPattern pattern = LEDPattern.steps(
        Map.of(0, Color.kBlack, 1. / 3, Color.kBlack, 2. / 3, color));
      return pattern;
  }


  public static void setNoteAngles(Supplier<double[][]> sup) {
    m_instance.getNotePositions = sup;
  }
  private static final double[][] defaultNoteAngles = new double[3][2];
  private Supplier<double[][]> getNotePositions = ()->defaultNoteAngles;
  private LEDPattern note() {
    return (reader, writer)->{
    
    /**
     * A 3x2 array of the closest 3 notes. 
     * The first element of each of the 3 subarrays is the horizontal angle of the note center.
     *  0 is center, negative is to the left edge of the frame, positive to the right
     * If the angle is less than -90, the note will not show (this indicates no detection)
     * The second element is the percentage width of the displayed bar that the note should take. This will be ignored if it corresponds to less than 5 LEDs
    */
    var notePositions = getNotePositions.get();
    // angles 
    for (int i = notePositions.length - 1; i >= 0; i--) {
      if (i < 0) continue;
      var angle = notePositions[i][0];
      // 
      var width = notePositions[i][1];
      if (angle < -90) {
        continue;
      }
      // assume 70 degree FoV
      double proportion = angle / 35.0;
      double halfStrip = STRIP_LENGTH / 2;
      // find the LED index of the center of the note. The strip starts on the right, so we need to negate the offset
      int ctrLED = (int) (halfStrip - (proportion * halfStrip));
      // find the number of additional LEDs each side of the center LED.
      int halfWidth = Math.max(2, (int) (halfStrip * width / 100));
      // loop over the relevant LEDs, limiting the endpoints to avoid setting nonexistent LEDs.
      for (int j = Math.max(0, ctrLED - halfWidth); j < Math.min(STRIP_LENGTH, ctrLED + halfWidth); j++){
        writer.setRGB(j, 255, 50, 0);
      }
    }
  };
}
}
