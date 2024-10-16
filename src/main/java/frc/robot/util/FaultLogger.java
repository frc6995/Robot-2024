package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.REVLibError;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

/**
 * FaultLogger allows for faults to be logged and displayed.
 *
 * <pre>
 * FaultLogger.register(spark); // registers a spark, periodically checking for hardware faults
 * spark.set(0.5);
 * FaultLogger.check(spark); // checks that the previous set call did not encounter an error.
 * </pre>
 */
public final class FaultLogger {

  /** An individual fault, containing necessary information. */
  public static record Fault(String name, String description, FaultType type) {
    @Override
    public String toString() {
      return name + ": " + description;
    }
  }

  /**
   * The type of fault, used for detecting whether the fallible is in a failure state and displaying
   * to NetworkTables.
   */
  public static enum FaultType {
    INFO,
    WARNING,
    ERROR,
  }

  /** A class to represent an alerts widget on NetworkTables */
  public static class Alerts {
    private final StringArrayPublisher errors;
    private final StringArrayPublisher warnings;
    private final StringArrayPublisher infos;

    public Alerts(NetworkTable base, String name) {
      NetworkTable table = base.getSubTable(name);
      table.getStringTopic(".type").publish().set("Alerts");
      errors = table.getStringArrayTopic("errors").publish();
      warnings = table.getStringArrayTopic("warnings").publish();
      infos = table.getStringArrayTopic("infos").publish();
    }

    public void set(Set<Fault> faults) {
      errors.set(filteredStrings(faults, FaultType.ERROR));
      warnings.set(filteredStrings(faults, FaultType.WARNING));
      infos.set(filteredStrings(faults, FaultType.INFO));
    }
  }

  // DATA
  private static final List<Supplier<Optional<Fault>>> faultSuppliers = new ArrayList<>();
  private static final Set<Fault> newFaults = new HashSet<>();
  private static final Set<Fault> activeFaults = new HashSet<>();
  private static final Set<Fault> totalFaults = new HashSet<>();

  // NETWORK TABLES
  private static final NetworkTable base = NetworkTableInstance.getDefault().getTable("DriverDisplay").getSubTable("Faults");
  private static final Alerts activeAlerts = new Alerts(base, "Active Faults");
  private static final Alerts totalAlerts = new Alerts(base, "Total Faults");

  /** Polls registered fallibles. This method should be called periodically. */
  public static void update() {
    activeFaults.clear();

    faultSuppliers.stream()
        .map(s -> s.get())
        .flatMap(Optional::stream)
        .forEach(FaultLogger::report);

    activeFaults.addAll(newFaults);
    newFaults.clear();

    totalFaults.addAll(activeFaults);

    activeAlerts.set(activeFaults);
    totalAlerts.set(totalFaults);
  }

  /** Clears total faults. */
  public static void clear() {
    totalFaults.clear();
  }

  /**
   * Returns the set of all current faults.
   *
   * @return The set of all current faults.
   */
  public static Set<Fault> activeFaults() {
    return activeFaults;
  }

  /**
   * Returns the set of all total faults.
   *
   * @return The set of all total faults.
   */
  public static Set<Fault> totalFaults() {
    return totalFaults;
  }

  /**
   * Reports a fault.
   *
   * @param fault The fault to report.
   */
  public static void report(Fault fault) {
    newFaults.add(fault);
    switch (fault.type) {
      case ERROR -> DriverStation.reportError(fault.toString(), false);
      case WARNING -> DriverStation.reportWarning(fault.toString(), false);
      case INFO -> System.out.println(fault.toString());
    }
  }

  /**
   * Reports a fault.
   *
   * @param name The name of the fault.
   * @param description The description of the fault.
   * @param type The type of the fault.
   */
  public static void report(String name, String description, FaultType type) {
    report(new Fault(name, description, type));
  }

  /**
   * Registers a new fault supplier.
   *
   * @param supplier A supplier of an optional fault.
   */
  public static void register(Supplier<Optional<Fault>> supplier) {
    faultSuppliers.add(supplier);
  }

  /**
   * Registers a new fault supplier.
   *
   * @param condition Whether a failure is occuring.
   * @param description The failure's description.
   * @param type The type of failure.
   */
  public static void register(
      BooleanSupplier condition, String name, String description, FaultType type) {
    faultSuppliers.add(
        () ->
            condition.getAsBoolean()
                ? Optional.of(new Fault(name, description, type))
                : Optional.empty());
  }

  private static FaultID[] sparkFaults = FaultID.values();
  /**
   * Registers fault suppliers for a CAN-based Spark motor controller.
   *
   * @param spark The Spark Max or Spark Flex to manage.
   */
  public static void register(CANSparkBase spark) {
    for (FaultID fault : sparkFaults) {
      register(() -> spark.getFault(fault), name(spark), fault.name(), FaultType.ERROR);
    }
    register(
        () -> spark.getMotorTemperature() > 100,
        name(spark),
        "motor above 100°C",
        FaultType.WARNING);
  }

  /**
   * Registers fault suppliers for a duty cycle encoder.
   *
   * @param encoder The duty cycle encoder to manage.
   */
  public static void register(DutyCycleEncoder encoder) {
    register(
        () -> !encoder.isConnected(),
        "Duty Cycle Encoder [" + encoder.getSourceChannel() + "]",
        "disconnected",
        FaultType.ERROR);
  }

  /**
   * Registers fault suppliers for a NavX.
   *
   * @param ahrs The NavX to manage.
   */
  public static void register(AHRS ahrs) {
    register(() -> !ahrs.isConnected(), "NavX", "disconnected", FaultType.ERROR);
  }

  /**
   * Registers fault suppliers for a power distribution hub/panel.
   *
   * @param powerDistribution The power distribution to manage.
   */
  public static void register(PowerDistribution powerDistribution) {
    for (Field field : PowerDistribution.class.getFields()) {
      register(
          () -> {
            try {
              if (field.getBoolean(powerDistribution)) {
                return Optional.of(
                    new Fault("Power Distribution", field.getName(), FaultType.ERROR));
              }
            } catch (Exception e) {
            }
            return Optional.empty();
          });
    }
  }

  /**
   * Registers fault suppliers for a camera.
   *
   * @param camera The camera to manage.
   */
  public static void register(PhotonCamera camera) {
    register(
        () -> !camera.isConnected(),
        "Photon Camera [" + camera.getName() + "]",
        "disconnected",
        FaultType.ERROR);
  }

  /**
   * Reports REVLibErrors from a spark.
   *
   * <p>This should be called immediately after any call to the spark.
   *
   * @param spark The spark to report REVLibErrors from.
   */
  public static void check(CANSparkBase spark) {
    REVLibError error = spark.getLastError();
    if (error != REVLibError.kOk) {
      report(name(spark), error.name(), FaultType.ERROR);
    }
  }

  /**
   * Returns an array of descriptions of all faults that match the specified type.
   *
   * @param type The type to filter for.
   * @return An array of description strings.
   */
  private static String[] filteredStrings(Set<Fault> faults, FaultType type) {
    return faults.stream()
        .filter(a -> a.type() == type)
        .map(Fault::toString)
        .toArray(String[]::new);
  }

    /**
   * Formats the name of a spark with its CAN ID.
   *
   * @param spark The spark to find the name of.
   * @return The name of a spark.
   */
  public static String name(CANSparkBase spark) {
    return "Spark [" + spark.getDeviceId() + "]";
  }
}
