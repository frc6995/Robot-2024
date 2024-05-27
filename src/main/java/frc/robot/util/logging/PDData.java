package frc.robot.util.logging;

import java.nio.ByteBuffer;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.PowerDistributionJNI;
import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/**
 * A utility to log data from the PDP/PDH with as little overhead as possible.
 */
public class PDData implements StructSerializable {

    protected static final record PD(int handle, int module) {
        public int getStickyFaults() {
            return PowerDistributionJNI.getStickyFaultsNative(handle);
        }

        public int getFaults() {
            return PowerDistributionJNI.getFaultsNative(handle);
        }

        public boolean getSwitchableChannel() {
            return PowerDistributionJNI.getSwitchableChannel(handle);
        }

        public double getVoltage() {
            return PowerDistributionJNI.getVoltage(handle);
        }

        public double getTotalCurrent() {
            return PowerDistributionJNI.getTotalCurrent(handle);
        }

        public double getTemperature() {
            return PowerDistributionJNI.getTemperature(handle);
        }

        public void getAllCurrents(double[] outCurrents) {
            PowerDistributionJNI.getAllCurrents(handle, outCurrents);
        }
    }

    protected static final class PowerDistributionFaultsStruct implements Struct<PowerDistributionFaults> {
        @Override
        public Class<PowerDistributionFaults> getTypeClass() {
            return PowerDistributionFaults.class;
        }

        @Override
        public int getSize() {
            return 4; // doing bitfields on a u32
        }

        @Override
        public String getSchema() {
            return "bool Channel0BreakerFault:1; "
                    + "bool Channel1BreakerFault:1; "
                    + "bool Channel2BreakerFault:1; "
                    + "bool Channel3BreakerFault:1; "
                    + "bool Channel4BreakerFault:1; "
                    + "bool Channel5BreakerFault:1; "
                    + "bool Channel6BreakerFault:1; "
                    + "bool Channel7BreakerFault:1; "
                    + "bool Channel8BreakerFault:1; "
                    + "bool Channel9BreakerFault:1; "
                    + "bool Channel10BreakerFault:1; "
                    + "bool Channel11BreakerFault:1; "
                    + "bool Channel12BreakerFault:1; "
                    + "bool Channel13BreakerFault:1; "
                    + "bool Channel14BreakerFault:1; "
                    + "bool Channel15BreakerFault:1; "
                    + "bool Channel16BreakerFault:1; "
                    + "bool Channel17BreakerFault:1; "
                    + "bool Channel18BreakerFault:1; "
                    + "bool Channel19BreakerFault:1; "
                    + "bool Channel20BreakerFault:1; "
                    + "bool Channel21BreakerFault:1; "
                    + "bool Channel22BreakerFault:1; "
                    + "bool Channel23BreakerFault:1; "
                    + "bool Brownout:1; "
                    + "bool CanWarning:1; "
                    + "bool HardwareFault:1; ";
        }

        @Override
        public String getTypeString() {
            return "struct:PowerDistributionFaults";
        }

        @Override
        public void pack(ByteBuffer bb, PowerDistributionFaults value) {
            int packed = 0;
            packed |= value.Channel0BreakerFault ? 1 : 0;
            packed |= value.Channel1BreakerFault ? 1 << 1 : 0;
            packed |= value.Channel2BreakerFault ? 1 << 2 : 0;
            packed |= value.Channel3BreakerFault ? 1 << 3 : 0;
            packed |= value.Channel4BreakerFault ? 1 << 4 : 0;
            packed |= value.Channel5BreakerFault ? 1 << 5 : 0;
            packed |= value.Channel6BreakerFault ? 1 << 6 : 0;
            packed |= value.Channel7BreakerFault ? 1 << 7 : 0;
            packed |= value.Channel8BreakerFault ? 1 << 8 : 0;
            packed |= value.Channel9BreakerFault ? 1 << 9 : 0;
            packed |= value.Channel10BreakerFault ? 1 << 10 : 0;
            packed |= value.Channel11BreakerFault ? 1 << 11 : 0;
            packed |= value.Channel12BreakerFault ? 1 << 12 : 0;
            packed |= value.Channel13BreakerFault ? 1 << 13 : 0;
            packed |= value.Channel14BreakerFault ? 1 << 14 : 0;
            packed |= value.Channel15BreakerFault ? 1 << 15 : 0;
            packed |= value.Channel16BreakerFault ? 1 << 16 : 0;
            packed |= value.Channel17BreakerFault ? 1 << 17 : 0;
            packed |= value.Channel18BreakerFault ? 1 << 18 : 0;
            packed |= value.Channel19BreakerFault ? 1 << 19 : 0;
            packed |= value.Channel20BreakerFault ? 1 << 20 : 0;
            packed |= value.Channel21BreakerFault ? 1 << 21 : 0;
            packed |= value.Channel22BreakerFault ? 1 << 22 : 0;
            packed |= value.Channel23BreakerFault ? 1 << 23 : 0;
            packed |= value.Brownout ? 1 << 24 : 0;
            packed |= value.CanWarning ? 1 << 25 : 0;
            packed |= value.HardwareFault ? 1 << 26 : 0;

            bb.putInt(packed);
        }

        public void pack(ByteBuffer bb, int value) {
            bb.putInt(value);
        }

        @Override
        public PowerDistributionFaults unpack(ByteBuffer bb) {
            int packed = bb.getInt();
            return new PowerDistributionFaults(packed);
        }
    }

    protected static final class PowerDistributionStickyFaultsStruct implements Struct<PowerDistributionStickyFaults> {
        @Override
        public Class<PowerDistributionStickyFaults> getTypeClass() {
            return PowerDistributionStickyFaults.class;
        }

        @Override
        public int getSize() {
            return 4; // doing bitfields on a u32
        }

        @Override
        public String getSchema() {
            return "bool Channel0BreakerFault:1; "
                    + "bool Channel1BreakerFault:1; "
                    + "bool Channel2BreakerFault:1; "
                    + "bool Channel3BreakerFault:1; "
                    + "bool Channel4BreakerFault:1; "
                    + "bool Channel5BreakerFault:1; "
                    + "bool Channel6BreakerFault:1; "
                    + "bool Channel7BreakerFault:1; "
                    + "bool Channel8BreakerFault:1; "
                    + "bool Channel9BreakerFault:1; "
                    + "bool Channel10BreakerFault:1; "
                    + "bool Channel11BreakerFault:1; "
                    + "bool Channel12BreakerFault:1; "
                    + "bool Channel13BreakerFault:1; "
                    + "bool Channel14BreakerFault:1; "
                    + "bool Channel15BreakerFault:1; "
                    + "bool Channel16BreakerFault:1; "
                    + "bool Channel17BreakerFault:1; "
                    + "bool Channel18BreakerFault:1; "
                    + "bool Channel19BreakerFault:1; "
                    + "bool Channel20BreakerFault:1; "
                    + "bool Channel21BreakerFault:1; "
                    + "bool Channel22BreakerFault:1; "
                    + "bool Channel23BreakerFault:1; "
                    + "bool Brownout:1; "
                    + "bool CanWarning:1; "
                    + "bool CanBusOff:1; "
                    + "bool HasReset:1; ";
        }

        @Override
        public String getTypeString() {
            return "struct:PowerDistributionStickyFaults";
        }

        @Override
        public void pack(ByteBuffer bb, PowerDistributionStickyFaults value) {
            int packed = 0;
            packed |= value.Channel0BreakerFault ? 1 : 0;
            packed |= value.Channel1BreakerFault ? 1 << 1 : 0;
            packed |= value.Channel2BreakerFault ? 1 << 2 : 0;
            packed |= value.Channel3BreakerFault ? 1 << 3 : 0;
            packed |= value.Channel4BreakerFault ? 1 << 4 : 0;
            packed |= value.Channel5BreakerFault ? 1 << 5 : 0;
            packed |= value.Channel6BreakerFault ? 1 << 6 : 0;
            packed |= value.Channel7BreakerFault ? 1 << 7 : 0;
            packed |= value.Channel8BreakerFault ? 1 << 8 : 0;
            packed |= value.Channel9BreakerFault ? 1 << 9 : 0;
            packed |= value.Channel10BreakerFault ? 1 << 10 : 0;
            packed |= value.Channel11BreakerFault ? 1 << 11 : 0;
            packed |= value.Channel12BreakerFault ? 1 << 12 : 0;
            packed |= value.Channel13BreakerFault ? 1 << 13 : 0;
            packed |= value.Channel14BreakerFault ? 1 << 14 : 0;
            packed |= value.Channel15BreakerFault ? 1 << 15 : 0;
            packed |= value.Channel16BreakerFault ? 1 << 16 : 0;
            packed |= value.Channel17BreakerFault ? 1 << 17 : 0;
            packed |= value.Channel18BreakerFault ? 1 << 18 : 0;
            packed |= value.Channel19BreakerFault ? 1 << 19 : 0;
            packed |= value.Channel20BreakerFault ? 1 << 20 : 0;
            packed |= value.Channel21BreakerFault ? 1 << 21 : 0;
            packed |= value.Channel22BreakerFault ? 1 << 22 : 0;
            packed |= value.Channel23BreakerFault ? 1 << 23 : 0;
            packed |= value.Brownout ? 1 << 24 : 0;
            packed |= value.CanWarning ? 1 << 25 : 0;
            packed |= value.CanBusOff ? 1 << 26 : 0;
            packed |= value.HasReset ? 1 << 27 : 0;

            bb.putInt(packed);
        }

        public void pack(ByteBuffer bb, int value) {
            bb.putInt(value);
        }

        @Override
        public PowerDistributionStickyFaults unpack(ByteBuffer bb) {
            int packed = bb.getInt();
            return new PowerDistributionStickyFaults(packed);
        }
    }

    public int faults;
    public int stickyFaults;
    public double voltage;
    public double totalCurrent;
    public boolean switchableChannel;
    public double temperature;
    public double[] currents;
    private PD pd;
    private boolean valid = false;

    public PDData(final PD pd) {
        currents = new double[24];
        if (pd != null) {
            valid = true;
            this.pd = pd;
            update();
        }

    }

    public static PDData create(int canId, ModuleType moduleType) {

        int handle;
        int module;
        try {
            handle = PowerDistributionJNI.initialize(canId, moduleType.value);
            module = PowerDistributionJNI.getModuleNumber(handle);

            HAL.report(tResourceType.kResourceType_PDP, module + 1);
            return new PDData(new PD(handle, module));
        } catch (Exception e) {
            DriverStation.reportError("Error initializing power logger: " + e.getMessage(), e.getStackTrace());
            return new PDData(null);
        }
    }

    public PDData update() {
        if (valid) {
            faults = pd.getFaults();
            stickyFaults = pd.getStickyFaults();
            voltage = pd.getVoltage();
            totalCurrent = pd.getTotalCurrent();
            switchableChannel = pd.getSwitchableChannel();
            temperature = pd.getTemperature();
            pd.getAllCurrents(currents);
        }
        return this;
    }

    public static final PDDataStruct struct = new PDDataStruct(new String[] {
        null,
        "frontMidtake",
        "backMidtake",
        "topShooter",
        "bottomShooter",
        "FR Drive",
        "FR Steer",
        "FL Drive",
        "FL Steer",
        null,
        null,
        "BR Drive",
        "BR Steer",
        "BL Drive",
        "BL Steer"
    });

    private static final Struct<double[]> makeChannelStruct(String[] channels) {
        StringBuilder schemaBuilder = new StringBuilder();
        for (int i = 0; i < 24; i++) {
            schemaBuilder.append("double c").append(i);
            if (i < channels.length && channels[i] != null) {
                schemaBuilder.append("_").append(channels[i].replace(" ", "_"));
            }
            schemaBuilder.append(";");
        }
        String schema = schemaBuilder.toString();
        System.out.println(schema);
        return new Struct<double[]>() {

            @Override
            public Class<double[]> getTypeClass() {
                // TODO Auto-generated method stub
                return double[].class;
            }

            @Override
            public String getTypeString() {
                // TODO Auto-generated method stub
                return "struct:PowerDistributionChannels";
            }

            @Override
            public int getSize() {
                // TODO Auto-generated method stub
                return 24 * 8;
            }

            @Override
            public String getSchema() {
                // TODO Auto-generated method stub
                return schema;
            }

            @Override
            public double[] unpack(ByteBuffer bb) {
                double[] values = new double[24];
                for (int i = 0; i < 24; i++) {
                    values[i] = bb.getDouble();
                }
                return values;
            }

            @Override
            public void pack(ByteBuffer bb, double[] value) {
                for (int i = 0; i < 24; i++) {
                    
                    double field = i >= value.length ? value[i] : 0; 
                    bb.putDouble(field);
                }
            }
        };
    }
    protected static final class PDDataStruct implements Struct<PDData> {
        Struct<double[]> channelsStruct;
        public PDDataStruct(String[] channels) {
            channelsStruct = makeChannelStruct(channels);
        }
        public PDDataStruct() {
            this(new String[0]);
        }

        @Override
        public Class<PDData> getTypeClass() {
            return PDData.class;
        }

        @Override
        public int getSize() {
            return 4 + 4 + 8 + 8 + 1 + 8 + channelsStruct.getSize();
        }

        @Override
        public String getSchema() {
            String schema = "PowerDistributionFaults faults; "
                    + "PowerDistributionStickyFaults stickyFaults; "
                    + "double voltage; "
                    + "double totalCurrent; "
                    + "bool switchableChannel; "
                    + "double temperature; "
                    + channelsStruct.getTypeString().substring(7) + " currents;";
            System.out.println(schema);
            return schema;

        }

        @Override
        public String getTypeString() {
            return "struct:PDData";
        }

        @Override
        public void pack(ByteBuffer bb, PDData value) {
            bb.putInt(value.faults);
            bb.putInt(value.stickyFaults);
            bb.putDouble(value.voltage);
            bb.putDouble(value.totalCurrent);
            bb.put((byte) (value.switchableChannel ? 1 : 0));
            bb.putDouble(value.temperature);
            channelsStruct.pack(bb, value.currents);
        }

        @Override
        public PDData unpack(ByteBuffer bb) {
            PDData data = new PDData(null);
            data.faults = bb.getInt();
            data.stickyFaults = bb.getInt();
            data.voltage = bb.getDouble();
            data.totalCurrent = bb.getDouble();
            data.switchableChannel = bb.get() == 1;
            data.temperature = bb.getDouble();
            data.currents = channelsStruct.unpack(bb);
            return data;
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {
                    new PowerDistributionFaultsStruct(),
                    new PowerDistributionStickyFaultsStruct(),
                    channelsStruct
            };
        }
    }
}
