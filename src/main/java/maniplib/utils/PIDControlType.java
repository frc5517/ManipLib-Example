package maniplib.utils;

import com.revrobotics.spark.SparkBase;

public class PIDControlType {

    public static ControlType controlType;

    /**
     * ControlType to run PIDControllers at.
     */
    public enum ControlType {
        /**
         * Position Control.
         */
        POSITION,
        /**
         * Velocity Control.
         */
        VELOCITY
    }

}
