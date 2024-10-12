package frc.robot.util.logging;

import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

/**
 * A struct wrapper of the 16-bit faults/sticky faults value from a
 * CANSparkBase,
 * so that each fault can be logged as an individual labeled entry.
 * Unfortunately, they are displayed as `int` values either 0 or 1, not boolean.
 */
public class RevFaults implements StructSerializable {
  public short bitfield = 0;

  public RevFaults() {
    this((short) 0);
  }
  public RevFaults(short bitfield) {
    this.bitfield = bitfield;
  }

  public void update(short bitfield) {
    this.bitfield = bitfield;
  }

  public static final RevFaultsStruct struct = new RevFaultsStruct();

  static class RevFaultsStruct implements Struct<RevFaults> {
    @Override
    public Class<RevFaults> getTypeClass() {
      return RevFaults.class;
    }

    @Override
    public String getTypeString() {
      return "struct:RevFaults";
    }

    @Override
    public int getSize() {
      return 4;
    }

    @Override
    public String getSchema() {
      return "int16 raw;"
          + "bool kBrownout:1;"
          + "bool kOvercurrent:1;"
          + "bool kIWDTReset:1;"
          + "bool kMotorFault:1;"
          + "bool kSensorFault:1;"
          + "bool kStall:1;"
          + "bool kEEPROMCRC:1;"
          + "bool kCANTX:1;"
          + "bool kCANRX:1;"
          + "bool kHasReset:1;"
          + "bool kDRVFault:1;"
          + "bool kOtherFault:1;"
          + "bool kSoftLimitFwd:1;"
          + "bool kSoftLimitRev:1;"
          + "bool kHardLimitFwd:1;"
          + "bool kHardLimitRev:1";
    }

    @Override
    public RevFaults unpack(ByteBuffer bb) {
      short bitfield = bb.getShort();
      // we don't care about the rest but we need to consume them out of the buffer
      bb.getShort();
      return new RevFaults(bitfield);
    }

    @Override
    public void pack(ByteBuffer bb, RevFaults value) {
      bb.putShort(value.bitfield);
      bb.putShort(value.bitfield);
    }
  }
}
