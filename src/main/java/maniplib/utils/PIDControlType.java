package maniplib.utils;

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
