package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Scalar;

//This class holds all constants, the reason we do this is so we can import all the constants 'statically.'
// i.e.: To import the drive constants, you would add
// import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.*;

public class Constants {

    public enum PIDSubsystemState {
        MANUAL,
        MOVING_TO_TARGET,
        AT_TARGET
    }

    public static class CameraConstants {
        public static final String CAMERA_NAME = "camera";
        public static final double DETECTION_THRESHOLD = 0.60;
        public static final Scalar LOWER_RED = new Scalar(0, 167, 75);
        public static final Scalar LOWER_BLUE = new Scalar(23, 78, 184);
        public static final Scalar UPPER = new Scalar(255, 255, 255);
    }

    public static class ClawConstants {
        public static final String CLAW_SERVO_NAME = "clawServo";

        public static final double MIN_ANGLE = 0;
        public static final double MAX_ANGLE = 270;


//        public static final double OPEN_POSITION = 0;
//        public static final double CLOSE_POSITION = 50;

        // TODO: Find new value
        public static final double OPEN_POSITION = 270 - 30;
        // TODO: Find new value
        public static final double CLOSE_POSITION = 270 - 15;
        // 270 is almost closed, so the piece on the servo is rotated
        // so going to 270 will be very bad
    }

    public static class DistanceSensorConstants {
        public static final String LEFT_DISTANCE_SENSOR_NAME = "leftDistanceSensor";
        public static final String RIGHT_DISTANCE_SENSOR_NAME = "rightDistanceSensor";

        // TODO: Find value
        public static double DISTANCE_THRESHOLD = -1;
    }

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
        public static final boolean SCALED = false;
        public static final double INPUT_MULTIPLIER = 1f;
        public static final double DEADZONE = 0.1;

        public static final String IMU_NAME = "imu";


        public static final double AUTO_DRIVE_SPEED = 0.5;
        public static final double AUTO_STRAFE_SPEED = -1;
        public static final double TURN_SPEED = -1;

        public static final double SNAP_TARGET = 180;
        public static final double AUTO_SNAP_POWER = 0.8;
        public static final double AUTO_SNAP_TOLERANCE = 1;

        public static final double AUTO_STEP_POWER = 0;
        public static final double AUTO_STEP_TOLERANCE = 50;

        public static final int HALF_STEP_VALUE = -1;
    }

    public static class DroneConstants {
        public static final String DRONE_SERVO_NAME = "droneServo";

        public static final double MIN_ANGLE = 0;
        public static final double MAX_ANGLE = 270;

        // TODO: Find new value
        public static final double START_POSITION = 0;
        // TODO: Find new value
        public static final double RELEASE_ANGLE = 180;
    }

    public static class ElbowConstants {
        public static final String ELBOW_DC_MOTOR_NAME = "elbowMotor";
        public static final DcMotorSimple.Direction ELBOW_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final double MANUAL_MOTOR_SPEED = 0.8;

        // PID values for the PID controller
        // TODO: Find new values
        public static final double P = 0.0019, I = 0, D = 0;

        // TODO: Find new value
        public static final int INTAKE_POSITION = 500;
        // TODO: Find new value
        public static final int DRIVING_POSITION = 2000;

        // TODO: Find new value
        public static final int LOW_SCORING_POSITION = 8400;
        // TODO: Find new value
        public static final int MEDIUM_SCORING_POSITION = 10400;
        // TODO: Find new value
        public static final int HIGH_SCORING_POSITION = 12400;

        // TODO: Find new value
        public static final int LEVEL_POSITION = 4750;
        // TODO: Find new value
        public static final int DRONE_LAUNCH_POSITION = 11000;

//        public static final int TEST_POSITION = 15000;

        // Drone position is ~14000

        // TODO: Find new value
        public static final double POWER_TOLERANCE = 0.1;
    }

    public static class IntakeConstants {
        public static final String INTAKE_SERVO_NAME = "intakeServo";
        public static final String INTAKE_MOTOR_NAME = "intakeMotor";

        public static final DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final double MIN_ANGLE = 0;
        public static final double MAX_ANGLE = 270;

//        // This is around 270 - 175 because the up position is actually around 270
//        public static final double INTAKE_DOWN_POSITION = 270 - 175;
//        public static final double INTAKE_DRIVING_POSITION = 270 - 120;
//        // This is 270 - 45 because the up position is actually around 270
//        public static final double INTAKE_UP_POSITION = 270 - 45;


        // Down position is 270, 0 is back into the robot, ~90 degrees is straight up
        // TODO: Find new value
        public static final double INTAKE_DOWN_POSITION = 270 - 180;
        // TODO: Find new value
        public static final double INTAKE_DRIVING_POSITION = 270 - 120;
        // TODO: Find new value
        public static final double INTAKE_UP_POSITION = 270 - 30; // This is 270 - 45 because the up position is actually around 270

        public static final double INTAKE_SPEED = -0.5;
        public static final double OUTTAKE_SPEED = 0.5;
    }

    public static class LinearSlideConstants {
        public static final String SLIDE_MOTOR_NAME = "linearSlideMotor";
        public static final DcMotorSimple.Direction SLIDE_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

        public static double EXTEND_POWER = .5;
        public static double RETRACT_POWER = -.5;

        // PID values for the PID controller
        // TODO: Find new values
        public static final double P = 0.013, I = 0, D = 0.0003;

        // Min and max values for the arm, don't change them
        // TODO: Find new value
        public static final int MIN_POSITION = 40;
        // TODO: Find new value
        public static final int MAX_POSITION = 3000;

        // TODO: Find new value
        public static final int IN_POSITION = MIN_POSITION;
        // TODO: Find new value
        public static final int LOW_SCORING_POSITION = 1300;
        // TODO: Find new value
        public static final int MEDIUM_SCORING_POSITION = 1870;
        // TODO: Find new value
        public static final int HIGH_SCORING_POSITION = 2800;

        // TODO: Find new value
        public static final double POWER_TOLERANCE = 0.08;
    }

    public static class WinchConstants {
        public static final String WINCH_MOTOR_NAME = "winchMotor";

        public static final DcMotorSimple.Direction WINCH_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final double WINCH_SPEED = .7;
        public static final double UNWINCH_SPEED = -.7;
    }
}
