package maniplib;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telemetry extends SubsystemBase {

    public static ManipTelemetry manipVerbosity = ManipTelemetry.HIGH;

    public enum ManipTelemetry {
        /*
         * No telemetry data is sent to dashboard. Including sim.
         */
        NONE,

        /*
         * Only basic telemetry data is sent to dashboard.
         */
        LOW,

        /*
         * All telemetry data is sent to dashboard.
         */
        HIGH
    }

}
