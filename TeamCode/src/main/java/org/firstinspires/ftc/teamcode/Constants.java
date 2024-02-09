package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//This class holds all constants, the reason we do this is so we can import all the constants 'statically.'
// i.e.: To import the drive constants, you would add
// import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.*;

public class Constants {

    /** State for the subsystems that use PIDs (elbow and linear slide). Can be manual, moving to target, or at target. */
    public enum PIDSubsystemState {
        MANUAL,
        MOVING_TO_TARGET,
        AT_TARGET
    }

    public static class AutoConstants {
        public static long DEFAULT_AUTO_WAIT = 250;
    }

    /** Constants for the BoxSubsystem. */
    @Config
    public static class BoxConstants {
        public static final String BOX_SERVO_NAME = "boxServo";

        public static final double MIN_ANGLE = 0;
        public static final double MAX_ANGLE = 270;

        // As the angle increases, the hinge closes
        // 270 degrees is inside the box
        public static final double OPEN_POSITION = 180;
        public static final double CLOSE_POSITION = 240;

        public static final String RED_RIGHT_LED_NAME = "redRight";
        public static final String GREEN_RIGHT_LED_NAME = "greenRight";
        public static final String RED_LEFT_LED_NAME = "redLeft";
        public static final String GREEN_LEFT_LED_NAME = "greenLeft";
    }

    /** Constants for the DistanceSensorSubsystem. */
    public static class DistanceSensorConstants {
        public static final String LEFT_DISTANCE_SENSOR_NAME = "leftDistanceSensor";
        public static final String RIGHT_DISTANCE_SENSOR_NAME = "rightDistanceSensor";

        /** The threshold (in inches) for the distance sensors to be blocked */
        public static double DISTANCE_THRESHOLD = 7;
    }

    /** Constants for the DriveSubsystem. */
    @Config
    public static class DriveConstants {
        public static final String FRONT_LEFT_MOTOR_NAME = "frontLeftMotor";
        public static final String FRONT_RIGHT_MOTOR_NAME = "frontRightMotor";
        public static final String REAR_LEFT_MOTOR_NAME = "rearLeftMotor";
        public static final String REAR_RIGHT_MOTOR_NAME = "rearRightMotor";

        public static final DcMotorSimple.Direction FRONT_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction FRONT_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction REAR_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction REAR_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final boolean FIELD_CENTRIC = true;

        @Deprecated
        public static final double INPUT_MULTIPLIER = 1f;
        /** @noinspection SpellCheckingInspection*/
        public static final double DEADZONE = 0.1;

        public static final String IMU_NAME = "imu";

        public static final double AUTO_DRIVE_SPEED = 0.3;
        public static final double AUTO_STRAFE_SPEED = 0.3;
        public static final double AUTO_TURN_SPEED = 0.5;
        public static double AUTO_HEADING_TOLERANCE = 5;

        public static final double PULSES_PER_MOTOR_REV = 537.7;
        public static final double DRIVE_GEAR_REDUCTION = 1;
        public static final double WHEEL_DIAMETER_INCHES = 5.51181;
        public static final double PULSES_PER_INCH =
                (PULSES_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
        public static int toPulses(double inches) {
            return (int) (inches * PULSES_PER_INCH);
        }
    }

    /** Constants for the DroneSubsystem. */
    public static class DroneConstants {
        public static final String DRONE_SERVO_NAME = "droneServo";

        public static final double MIN_ANGLE = 0;
        public static final double MAX_ANGLE = 270;

        public static final double START_POSITION = 0;
        public static final double RELEASE_ANGLE = 180;
    }

    /** Constants for the ElbowSubsystem. */
    @Config
    public static class ElbowConstants {
        public static final String ELBOW_DC_MOTOR_NAME = "elbowMotor";
        public static final DcMotorSimple.Direction ELBOW_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final String ELBOW_LIMIT_SWITCH_NAME = "elbowLimit";

        public static final double MANUAL_MOTOR_SPEED_MULTIPLIER = 0.8;

        // PID values for the PID controller
        public static  double P = 0.0025, I = 0, D = 0;

        public static  int INTAKE_POSITION = 0;
        public static  int LOW_SCORING_POSITION = 7600;
        @Deprecated
        public static  int AUTO_SCORING_POSITION = 7000;

        public static  int LEVEL_POSITION = 3500;
        public static  int DRIVING_POSITION = 1100;


        public static  int DRONE_LAUNCH_POSITION = 6900;

        // TODO: Find new value
        public static  int MEDIUM_SCORING_POSITION = 10400;
        // TODO: Find new value
        public static  int HIGH_SCORING_POSITION = 12400;

        /** The tolerance for the PID controller for the elbow. */
        public static final double PID_POWER_TOLERANCE = 0.1;
    }

    /** Constants for the IntakeSubsystem. */
    @Config
    public static class IntakeConstants {
        public static final String INTAKE_SERVO_NAME = "intakeServo";
        public static final String INTAKE_MOTOR_NAME = "intakeMotor";

        public static final DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final double MIN_ANGLE = 0;
        public static final double MAX_ANGLE = 270;

        // As the angle increases, the intake moves down, with ~150 being level
        public static final double INTAKE_DOWN_POSITION = 190;
        public static final double INTAKE_DRIVING_POSITION = 140;
        public static final double INTAKE_UP_POSITION = 100;

        public static final double INTAKE_SPEED = 1;
        public static final double OUTTAKE_SPEED = -1;
    }

    /** Constants for the LEDSubsystem. */
    @Config
    public static class LEDConstants {
        public static final String LED_NAME = "LED";
    }

    /** Constants for the LinearSlideSubsystem. */
    @Config
    public static class LinearSlideConstants {
        public static final String SLIDE_MOTOR_NAME = "linearSlideMotor";
        public static final DcMotorSimple.Direction SLIDE_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

        public static final String SLIDE_LIMIT_SWITCH_NAME = "slideLimit";

        // The motor is reversed so we can have positive values higher up, meaning these have to be negative
        public static double SLIDE_MANUAL_POWER_MULTIPLIER = -.8;

        // PID values for the PID controller
        public static  double P = 0.003, I = 0, D = 0;

        // Min and max values for the arm, don't change them
        public static final int MIN_POSITION = 0;
        public static final int MAX_POSITION = 2900;

        public static  int IN_POSITION = MIN_POSITION;
        public static  int LOW_SCORING_POSITION = 2600;
        @Deprecated
        public static  int AUTO_SCORING_POSITION = 2800;

        // TODO: Find new value
        public static  int MEDIUM_SCORING_POSITION = -1; //1870;
        // TODO: Find new value
        public static  int HIGH_SCORING_POSITION = -1; //2800;

        /** The tolerance for the PID controller for the slide. */
        public static final double PID_POWER_TOLERANCE = .3;
    }

    /** Constants for the WinchSubsystem. */
    public static class WinchConstants {
        public static final String WINCH_MOTOR_NAME = "winchMotor";

        public static final DcMotorSimple.Direction WINCH_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

        public static final double WINCH_SPEED = 1;
        public static final double UNWINCH_SPEED = -.5;
    }
}
