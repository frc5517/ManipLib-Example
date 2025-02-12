package maniplib.utils;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

public class ManipMath {

    public static class Arm {

        /**
         * Convert {@link Angle} into motor {@link Angle}
         *
         * @param measurement Angle, to convert.
         * @return {@link Angle} equivalent to rotations of the motor.
         */
        public static Angle convertAngleToSensorUnits(double reduction, Angle measurement) {
            return Rotations.of(measurement.in(Rotations) * reduction);
        }

        /**
         * Convert motor rotations {@link Angle} into usable {@link Angle}
         *
         * @param measurement Motor rotations
         * @return Usable angle.
         */
        public static Angle convertSensorUnitsToAngle(double reduction, Angle measurement) {
            return Rotations.of(measurement.in(Rotations) / reduction);

        }

    }

    public static class Elevator
    {
        /**
         * Convert {@link Distance} into {@link Angle}
         *
         * @param distance Distance, usually Meters.
         * @return {@link Angle} equivalent to rotations of the motor.
         */
        public static Angle convertDistanceToRotations(double drumRadius, double gearing, Distance distance)
        {
            return Rotations.of(distance.in(Meters) /
                    (drumRadius * 2 * Math.PI) *
                    gearing);
        }

        /**
         * Convert {@link Angle} into {@link Distance}
         *
         * @param rotations Rotations of the motor
         * @return {@link Distance} of the elevator.
         */
        public static Distance convertRotationsToDistance(double drumRadius, double gearing, Angle rotations)
        {
            return Meters.of((rotations.in(Rotations) / gearing) *
                    (drumRadius * 2 * Math.PI));
        }
    }
}
