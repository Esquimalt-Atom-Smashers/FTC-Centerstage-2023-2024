package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.PIDController;

//This class holds all constants, the reason we do this is so we can import all the constants 'statically.'
public class Constants {
    public static class DriveConstants {
        public static final String FRONT_LEFT_MOTOR_NAME = " ";
        public static final String REAR_LEFT_MOTOR_NAME = " ";
        public static final String FRONT_RIGHT_MOTOR_NAME = " ";
        public static final String REAR_RIGHT_MOTOR_NAME = " ";

        public static final DcMotorSimple.Direction FRONT_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction FRONT_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction REAR_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction REAR_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final boolean FIELD_CENTRIC = true;
        public static final double POWER_MULTIPLIER = 0.80;

        public static final int AUTO_SNAP_TOLERANCE_DEG = 2;
        public static final double AUTO_SNAP_POWER = 0.5;

        public static final int STEPPING_TOLERANCE = 15;
        public static final double STEPPING_POWER = 0.25;

        public static final double TAG_ALIGNMENT_TOLERANCE = 2.0;
    }

    public static class IntakeConstants {
        public static final String INTAKE_MOTOR_NAME = " ";

        public static final DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final double INTAKE_POWER = 0.2;
        public static final double OUTTAKE_POWER = -0.3;
    }

    public static class ElbowConstants {
        public static final String ELBOW_MOTOR_NAME = " ";

        public static final double PULSES_PER_REV = -1;
        public static final double PULSES_PER_DEG = PULSES_PER_REV / 180;

        public static final DcMotorSimple.Direction ELBOW_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final double LIFT_POWER = 0.3;
        public static final double DROP_POWER = -0.2;

        public static final PIDController ELBOW_PID_CONTROLLER = new PIDController(-1, 0, 0);
        public static final double CONTROL_LOOP_TOLERANCE = 1.0;
    }

    public static class LinearSlideConstants {
        public static final String SLIDE_MOTOR_NAME = " ";

        public static final double PULSES_PER_MOTOR_REV = -1;
        public static final double WHEEL_DIAMETER = 4.0;
        public static final double GEAR_RATIO = 1.0;
        public static final double PULSES_PER_INCH = (PULSES_PER_MOTOR_REV * GEAR_RATIO) / (WHEEL_DIAMETER * 3.14159);

        public static final DcMotorSimple.Direction SLIDE_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final double EXTEND_POWER = 0.3;
        public static final double RETRACT_POWER = 0.2;

        public static final PIDController SLIDE_PID_CONTROLLER = new PIDController(-1, 0, 0);
        public static final double CONTROL_LOOP_TOLERANCE = 1.0;
    }

    public static class WristConstants {
        public static final String WRIST_SERVO_NAME = " ";

        public static final Servo.Direction WRIST_SERVO_DIRECTION = Servo.Direction.FORWARD;

        public static final double PICKUP_POSITION = -1;
        public static final double DROP_POSITION = -1;
    }

    public static class ClawConstants {
        public static final String CLAW_SERVO_NAME = " ";

        public static final Servo.Direction CLAW_SERVO_DIRECTION = Servo.Direction.FORWARD;

        public static final double CLOSE_POSITION = -1;
        public static final double OPEN_POSITION = -1;
    }
}
