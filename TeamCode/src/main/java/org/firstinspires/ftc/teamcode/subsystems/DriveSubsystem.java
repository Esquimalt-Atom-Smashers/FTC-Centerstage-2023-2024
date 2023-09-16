package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.*;

import java.util.Arrays;

public class DriveSubsystem {
    //Declare hardware here.
    private final DcMotorEx frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
    private final DcMotorEx[] motors;

    private final BNO055IMU imu;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize the motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_MOTOR_NAME);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_MOTOR_NAME);
        rearLeftMotor = hardwareMap.get(DcMotorEx.class, REAR_LEFT_MOTOR_NAME);
        rearRightMotor = hardwareMap.get(DcMotorEx.class, REAR_RIGHT_MOTOR_NAME);

        motors = new DcMotorEx[]{frontRightMotor, frontLeftMotor, rearLeftMotor, rearRightMotor};

        // Set the direction of the motors
        frontLeftMotor.setDirection(FRONT_LEFT_MOTOR_DIRECTION);
        frontRightMotor.setDirection(FRONT_RIGHT_MOTOR_DIRECTION);
        rearLeftMotor.setDirection(REAR_LEFT_MOTOR_DIRECTION);
        rearRightMotor.setDirection(REAR_RIGHT_MOTOR_DIRECTION);

        // Set the motor modes and zero power behavior
        Arrays.stream(motors).forEach(motor -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize the imu
        imu = hardwareMap.get(BNO055IMU.class, IMU_NAME);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }

    //Define methods that control the bot down here.

    // Main drive method that controls the robot
    public void drive(float forward, float strafe, float turn) {
        if (FIELD_CENTRIC) {
            // Field centric drive
            // TODO: Field centric drive
            double gyroRadians = Math.toRadians(-getHeading());
            double rotateX = strafe * Math.cos(gyroRadians) - forward * Math.sin(gyroRadians);
            double rotateY = strafe * Math.sin(gyroRadians) + forward * Math.sin(gyroRadians);

            frontLeftMotor.setPower(scaleInput(rotateY + rotateX + turn));
            frontRightMotor.setPower(scaleInput(rotateY - rotateX - turn));
            rearLeftMotor.setPower(scaleInput(rotateY - rotateX + turn));
            rearRightMotor.setPower(scaleInput(rotateY + rotateX - turn));
        }
        else {
            // Robot centric drive
            frontLeftMotor.setPower(scaleInput(forward + strafe + turn));
            frontRightMotor.setPower(scaleInput(forward - strafe - turn));
            rearLeftMotor.setPower(scaleInput(forward - strafe + turn));
            rearRightMotor.setPower(scaleInput(forward + strafe - turn));
        }

    }

    // Auto drive
    public void drive() {

    }

    // Auto strafe
    public void strafe() {

    }

    // Used for autonomous driving
    public void driveToPosition() {

    }

    // Stop all of the motors
    public void stop() {
        Arrays.stream(motors).forEach(motor -> motor.setPower(0));
    }

    //Define lower-level methods here. (Methods that are private or work behind the scenes)

    private void setMotorMode(DcMotor.RunMode runMode) {
        Arrays.stream(motors).forEach(motor -> motor.setMode(runMode));
    }

    // Take the input and scale it if needed, while also clipping it to between -1 and 1
    private double scaleInput(double input) {
        if (SCALED) {
            // Take the input (forward, strafe, turn) and scale it so that moving the joystick halfway doesn't use half power
            // Current formula just cubes the input and multiplies it by the multiplier
            return  Range.clip(Math.pow(input, 3) * INPUT_MULTIPLIER, -1, 1);
        }
        else {
            return Range.clip(input * INPUT_MULTIPLIER, -1, 1);
        }
    }

    private double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    //Define getter/setter's here.

    public BNO055IMU getGyro() {
        return imu;
    }
}
