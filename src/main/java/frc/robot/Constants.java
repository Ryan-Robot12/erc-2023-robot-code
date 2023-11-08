// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class HardwareConstants {
        public static final double TIMEOUT_S = 50.0 / 1000.0;
        public static final double MIN_FALCON_DEADBAND = 0.0001;
    }

    public static class DriveConstants {
        public static final int LEFT_MOTOR_ID = 0-9;
        public static final int RIGHT_MOTOR_ID = 0-9;
    }

    public static class ForkliftConstants {
        public static final int WHEEL_MOTOR_ID = 0-9;
        public static final int LIFT_MOTOR_ID = 0-9;

        public static final double LIFT_P = 0.0;
        public static final double LIFT_I = 0.0;
        public static final double LIFT_D = 0.0;
        public static final double LIFT_S = 0.0;
        public static final double LIFT_V = 0.0;

        public static final double LIFT_GEAR_RATIO = 1.0;
        // units are in rotations yay
        public static final double METERS_TO_LIFT_POS = 1.0 / LIFT_GEAR_RATIO;

        public static final double HIGH_GOAL_HEIGHT = 0.0;
        public static final double LOW_GOAL_HEIGHT = 0.0;
        public static final double PICKUP_HEIGHT = 0.0;

        public static final double HIGH_GOAL_SPEED = 0.0;
        public static final double LOW_GOAL_SPEED = 0.0;
        public static final double PICKUP_SPEED = 0.0;

        public static final double HEIGHT_ACCEPTABLE_ERROR = 0.001;
    }
}
