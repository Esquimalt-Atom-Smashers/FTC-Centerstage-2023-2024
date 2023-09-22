package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.*;
import static org.firstinspires.ftc.teamcode.Constants.LinearSlideConstants.PULSES_PER_INCH;

import java.util.Arrays;

public class DriveSubsystem {
    //Declare hardware here.
    private final DcMotorEx frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
    private final DcMotorEx[] motors;

    // TODO: There is a problem with the initialization of the imu
    private final BNO055IMU imu;

    Telemetry telemetry;

    private double snapTarget;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize the motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_MOTOR_NAME);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_MOTOR_NAME);
        rearLeftMotor = hardwareMap.get(DcMotorEx.class, REAR_LEFT_MOTOR_NAME);
        rearRightMotor = hardwareMap.get(DcMotorEx.class, REAR_RIGHT_MOTOR_NAME);

        motors = new DcMotorEx[]{frontRightMotor, frontLeftMotor, rearLeftMotor, rearRightMotor};

        this.telemetry = telemetry;

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
    public void drive(double forward, double strafe, double turn) {
        if (FIELD_CENTRIC) {
//            telemetry.addData("Heading", getHeading());
//            telemetry.update();
            // Field centric drive
            double gyroRadians = Math.toRadians(-getHeading());
            double rotateX = strafe * Math.cos(gyroRadians) - forward * Math.sin(gyroRadians);
            double rotateY = strafe * Math.sin(gyroRadians) + forward * Math.cos(gyroRadians);

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

    public void autoSnap() {
        // Our snap target is a predetermined angle
        snapTarget = SNAP_TARGET;
        if (getNormalizedAngle() >= -90) drive(0, 0, AUTO_SNAP_POWER);
        if (getNormalizedAngle() <= -90) drive(0, 0, -AUTO_SNAP_POWER);
    }

    public boolean isFinishedSnapping() {
        // Checks if we are within the tolerance to auto snap
        return isWithinTolerance(getNormalizedAngle(), snapTarget, AUTO_SNAP_TOLERANCE);
    }

    public void centerWithTag() {

    }

    public boolean isCentered() {
        return false;
    }

    public void halfStepLeft() {
        // Add or subtract the half step value to strafe to the left
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() - TargetPosition.HALF_STEP.getTarget());
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + TargetPosition.HALF_STEP.getTarget());
        rearLeftMotor.setTargetPosition(rearLeftMotor.getCurrentPosition() - TargetPosition.HALF_STEP.getTarget());
        rearRightMotor.setTargetPosition(rearRightMotor.getCurrentPosition() + TargetPosition.HALF_STEP.getTarget());
        drive(0, -AUTO_STEP_POWER, 0);
    }

    public void halfStepRight() {
        // Add or subtract the half step value to strafe to the right
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + TargetPosition.HALF_STEP.getTarget());
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - TargetPosition.HALF_STEP.getTarget());
        rearLeftMotor.setTargetPosition(rearLeftMotor.getCurrentPosition() + TargetPosition.HALF_STEP.getTarget());
        rearRightMotor.setTargetPosition(rearRightMotor.getCurrentPosition() - TargetPosition.HALF_STEP.getTarget());
        drive(0, AUTO_STEP_POWER, 0);
    }

    public boolean isFinishedSteppingLeft() {
        return isWithinTolerance(frontLeftMotor.getCurrentPosition(), frontLeftMotor.getTargetPosition(), AUTO_STEP_TOLERANCE) &&
                isWithinTolerance(frontRightMotor.getCurrentPosition(), frontRightMotor.getTargetPosition(), AUTO_STEP_TOLERANCE) &&
                isWithinTolerance(rearLeftMotor.getCurrentPosition(), rearLeftMotor.getTargetPosition(), AUTO_STEP_TOLERANCE) &&
                isWithinTolerance(rearRightMotor.getCurrentPosition(), rearRightMotor.getTargetPosition(), AUTO_STEP_TOLERANCE);
    }

    public boolean isFinishedSteppingRight() {
        return isWithinTolerance(frontLeftMotor.getCurrentPosition(), frontLeftMotor.getTargetPosition(), AUTO_STEP_TOLERANCE) &&
                isWithinTolerance(frontRightMotor.getCurrentPosition(), frontRightMotor.getTargetPosition(), AUTO_STEP_TOLERANCE) &&
                isWithinTolerance(rearLeftMotor.getCurrentPosition(), rearLeftMotor.getTargetPosition(), AUTO_STEP_TOLERANCE) &&
                isWithinTolerance(rearRightMotor.getCurrentPosition(), rearRightMotor.getTargetPosition(), AUTO_STEP_TOLERANCE);
    }

    // Used for autonomous driving
    public void driveToPosition() {

    }

    // Stop all of the motors
    public void stop() {
        Arrays.stream(motors).forEach(motor -> motor.setPower(0));
    }

    public void setMotorPositions(int position) {
        Arrays.stream(motors).forEach(motor -> motor.setTargetPosition(position));
    }

    public int getNormalizedAngle() {
        // TODO:                                 |
        // TODO: Should this be negative?       \/
        return (int) AngleUnit.normalizeDegrees(imu.getAngularOrientation().firstAngle);
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
            // Otherwise, we just multiply the input by the multiplier
            return Range.clip(input * INPUT_MULTIPLIER, -1, 1);
        }
    }

    private boolean isWithinTolerance(double input, double target, double tolerance) {
        return Math.abs(input - target) <= tolerance;
    }

    private double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    //Define getter/setter's here.

    public BNO055IMU getGyro() {
        return imu;
    }

    enum TargetPosition {
        HALF_STEP(-1);

        private int inches;
        TargetPosition(int inches) {
            this.inches = inches;
        }

        int getTarget() {
            return (int) (inches * PULSES_PER_INCH);
        }
    }
}
