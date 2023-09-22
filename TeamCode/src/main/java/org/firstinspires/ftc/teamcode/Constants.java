package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.PIDController;

//This class holds all constants, the reason we do this is so we can import all the constants 'statically.'
// i.e.: To import the drive constants, you would add
// import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.*;

public class Constants {
    public static class DriveConstants {
        /* Define all driving constants here i.e:
            - configuration names
            - PID values(gain)
            - booleans(USE_FIELD_CENTRIC)
            - limits(MAX_SPEED_M_S)
        */

        public static final double DEADZONE = 0.1;

        public static final double INPUT_MULTIPLIER = 0.8f;

        public static final String FRONT_LEFT_MOTOR_NAME = "frontLeftMotor";
        public static final String FRONT_RIGHT_MOTOR_NAME = "frontRightMotor";
        public static final String REAR_LEFT_MOTOR_NAME = "rearLeftMotor";
        public static final String REAR_RIGHT_MOTOR_NAME = "rearRightMotor";

        public static final DcMotorSimple.Direction FRONT_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction FRONT_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction REAR_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction REAR_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

        public static final String IMU_NAME = "imu";

        public static final boolean FIELD_CENTRIC = true;
        public static final boolean SCALED = false;

        public static final double SNAP_TARGET = -1;

        public static final double AUTO_DRIVE_SPEED = -1;
        public static final double AUTO_STRAFE_SPEED = -1;
        public static final double TURN_SPEED = -1;

        public static final double AUTO_SNAP_POWER = -1;
        public static final double AUTO_SNAP_TOLERANCE = -1;

        public static final double AUTO_STEP_POWER = -1;
        public static final double AUTO_STEP_TOLERANCE = -1;
    }

    public static class ClawConstants {
        public static final String CLAW_SERVO_NAME = "clawServo";

        public static final double MIN_POSITION = 0;
        public static final double MAX_POSITION = 270;
        public static final double OPEN_POSITION = 0;
        // TODO: Fine tune this value (it is already around the right value)
        public static final double CLOSE_POSITION = 40;
    }

    public static class IntakeConstants {
        public static final String INTAKE_SERVO_NAME = "intakeServo";
        public static final String INTAKE_MOTOR_NAME = "intakeMotor";
        public static final DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final double INTAKE_DOWN_POSITION = -1;
        public static final double INTAKE_UP_POSITION = -1;
        public static final double INTAKE_SPEED = 1;
        public static final double OUTTAKE_SPEED = -1;
    }

    public static class LinearSlideConstants {
        public static final String SLIDE_MOTOR_NAME = "linearSlideMotor";
        public static final DcMotorSimple.Direction SLIDE_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final double EXTEND_POWER = .25;
        public static final double RETRACT_POWER = -.25;

        public static final double PULSES_PER_MOTOR_REV = -1;
        public static final double WHEEL_DIAMETER = -1;
        public static final double GEAR_RATIO = -1;
        public static final double PULSES_PER_INCH = (PULSES_PER_MOTOR_REV * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);

        public static final PIDController SLIDE_PID_CONTROLLER = new PIDController(-1, 0, 0);
        public static final double TARGET_TOLERANCE = 1.0;


    }

    public static class WristConstants {
        public static final String WRIST_SERVO_MOTOR_NAME = " ";
        public static final double WRIST_UP_POSITION = -1;
        public static final double WRIST_DOWN_POSITION = -1;
    }

    public static class ElbowConstants {
        public static final String ELBOW_DC_MOTOR_NAME = "elbowMotor";
        public static final double MOTOR_LIMITATION_OF_ANGLE = -1;
        public static final double MOTOR_ANGLE = -1;
        public static final double MOTOR_POSITION = -1;
        public static final double ARM_POSITION = -1;
        public static final double ARM_LIMITATION_OF_ANGLE = -1;
        public static final double ARM_ANGLE = -1;
    }
}
