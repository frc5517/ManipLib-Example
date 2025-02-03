package maniplib.utils;

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Rotations;

public class ManipMath {

    public static class Arm {

        /**
         * Convert {@link Angle} into motor {@link Angle}
         *
         * @param measurement Angle, to convert.
         * @return {@link Angle} equivalent to rotations of the motor.
         */
        public static Angle convertAngleToSensorUnits(double armReduction, Angle measurement) {
            return Rotations.of(measurement.in(Rotations) * armReduction);
        }

        /**
         * Convert motor rotations {@link Angle} into usable {@link Angle}
         *
         * @param measurement Motor roations
         * @return Usable angle.
         */
        public static Angle convertSensorUnitsToAngle(double armReduction, Angle measurement) {
            return Rotations.of(measurement.in(Rotations) / armReduction);

        }

    }
}
