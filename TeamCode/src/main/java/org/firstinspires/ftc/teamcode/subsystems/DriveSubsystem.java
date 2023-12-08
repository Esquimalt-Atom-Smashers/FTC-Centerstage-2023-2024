package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.*;

import java.util.Arrays;

/**
 * A subsystem representing the wheels and gyro of the robot. Uses four {@link DcMotorEx} for the wheels and a {@link BNO055IMU} for the gyro
 */
public class DriveSubsystem extends SubsystemBase {
    // Motors
    private final DcMotorEx frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
    private final DcMotorEx[] motors;

    // Gyro
    private final BNO055IMU imu;

    private double snapTarget;

    public static double forwardTarget;
    private boolean atForwardTarget;

    /**
     * Create a new DriveSubsystem. Initializes the four {@link DcMotorEx} and adds them to an array. Also sets their direction
     * correctly, sets their zero power behaviour to brake, resets their encoders. Initializes the gyro.
     * @param hardwareMap The hardware map of the robot
     */
    public DriveSubsystem(HardwareMap hardwareMap) {
        // Initialize the motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_MOTOR_NAME);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_MOTOR_NAME);
        rearLeftMotor = hardwareMap.get(DcMotorEx.class, REAR_LEFT_MOTOR_NAME);
        rearRightMotor = hardwareMap.get(DcMotorEx.class, REAR_RIGHT_MOTOR_NAME);

        motors = new DcMotorEx[]{frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor};

        // Set the direction of the motors
        frontLeftMotor.setDirection(FRONT_LEFT_MOTOR_DIRECTION);
        frontRightMotor.setDirection(FRONT_RIGHT_MOTOR_DIRECTION);
        rearLeftMotor.setDirection(REAR_LEFT_MOTOR_DIRECTION);
        rearRightMotor.setDirection(REAR_RIGHT_MOTOR_DIRECTION);

        // Set the motor modes and zero power behavior
        Arrays.stream(motors).forEach(motor -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
        resetEncoder();

        // Initialize the imu
        imu = hardwareMap.get(BNO055IMU.class, IMU_NAME);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }

    public void resetEncoder() {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Drives the robot using joystick input.
     * @param forward The amount to move forward
     * @param strafe The amount to move left and right
     * @param turn The amount to turn
     * @param fieldCentric If we want this drive to be field centric. Default is true
     * @param scaled If we want the inputs to be scaled. Default is true
     */
    public void drive(double forward, double strafe, double turn, boolean fieldCentric, boolean scaled) {
        forward = Math.abs(forward) >= DEADZONE ? forward : 0;
        strafe = Math.abs(strafe) >= DEADZONE ? strafe : 0;
        turn = Math.abs(turn) >= DEADZONE ? turn : 0;
        if (fieldCentric) {
            // Field centric drive
            double gyroRadians = Math.toRadians(-getHeading());
            double rotateX = strafe * Math.cos(gyroRadians) - forward * Math.sin(gyroRadians);
            double rotateY = strafe * Math.sin(gyroRadians) + forward * Math.cos(gyroRadians);

            frontLeftMotor.setPower(scaleInput(rotateY + rotateX + turn, scaled));
            frontRightMotor.setPower(scaleInput(rotateY - rotateX - turn, scaled));
            rearLeftMotor.setPower(scaleInput(rotateY - rotateX + turn, scaled));
            rearRightMotor.setPower(scaleInput(rotateY + rotateX - turn, scaled));
        }
        else {
            // Robot centric drive
            frontLeftMotor.setPower(scaleInput(forward + strafe + turn, scaled));
            frontRightMotor.setPower(scaleInput(forward - strafe - turn, scaled));
            rearLeftMotor.setPower(scaleInput(forward - strafe + turn, scaled));
            rearRightMotor.setPower(scaleInput(forward + strafe - turn, scaled));
        }

    }

    /**
     * Drives the robot using joystick input. Uses the default values for field centric and scaling.
     * @param forward The amount to move forward
     * @param strafe The amount to move left and right
     * @param turn The amount to turn
     */
    public void drive(double forward, double strafe, double turn) {
        drive(forward, strafe, turn, FIELD_CENTRIC, SCALED);
    }

    public boolean forward() {
        return frontLeftMotor.getCurrentPosition() < frontLeftMotor.getTargetPosition();
    }

    public void setForwardTarget(int target) {
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + target);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + target);
        rearLeftMotor.setTargetPosition(rearLeftMotor.getCurrentPosition() + target);
        rearRightMotor.setTargetPosition(rearRightMotor.getCurrentPosition() + target);
        forwardTarget = target;
        atForwardTarget = false;
    }


    public void autoSnap() {
        // Our snap target is a predetermined angle
        snapTarget = SNAP_TARGET;
        if (getNormalizedAngle() >= -90) drive(0, 0, AUTO_SNAP_POWER);
        if (getNormalizedAngle() <= -90) drive(0, 0, -AUTO_SNAP_POWER);
    }

    public void autoSnap(Telemetry t) {
        snapTarget = SNAP_TARGET;
        if (getNormalizedAngle() >= -90) {
            t.addData("Would be turning", AUTO_SNAP_POWER);
            drive(0, 0, AUTO_SNAP_POWER);
        };
        if (getNormalizedAngle() <= -90) {
            t.addData("Would be turning", -AUTO_SNAP_POWER);
            drive(0, 0, -AUTO_SNAP_POWER);

        }
    }

    public boolean isFinishedSnapping() {
        // Checks if we are within the tolerance to auto snap
        return isWithinTolerance(getNormalizedAngle(), snapTarget, AUTO_SNAP_TOLERANCE);
    }

    public void halfStepLeft() {
        // Add or subtract the half step value to strafe to the left
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() - HALF_STEP_VALUE);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + HALF_STEP_VALUE);
        rearLeftMotor.setTargetPosition(rearLeftMotor.getCurrentPosition() - HALF_STEP_VALUE);
        rearRightMotor.setTargetPosition(rearRightMotor.getCurrentPosition() + HALF_STEP_VALUE);
        drive(0, -AUTO_STEP_POWER, 0);
    }

    public void halfStepRight() {
        // Add or subtract the half step value to strafe to the right
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + HALF_STEP_VALUE);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - HALF_STEP_VALUE);
        rearLeftMotor.setTargetPosition(rearLeftMotor.getCurrentPosition() + HALF_STEP_VALUE);
        rearRightMotor.setTargetPosition(rearRightMotor.getCurrentPosition() - HALF_STEP_VALUE);
        drive(0, AUTO_STEP_POWER, 0);
    }

    public boolean areMotorsAtPositions() {
        return isWithinTolerance(frontLeftMotor.getCurrentPosition(), frontLeftMotor.getTargetPosition(), AUTO_STEP_TOLERANCE) &&
                isWithinTolerance(frontRightMotor.getCurrentPosition(), frontRightMotor.getTargetPosition(), AUTO_STEP_TOLERANCE) &&
                isWithinTolerance(rearLeftMotor.getCurrentPosition(), rearLeftMotor.getTargetPosition(), AUTO_STEP_TOLERANCE) &&
                isWithinTolerance(rearRightMotor.getCurrentPosition(), rearRightMotor.getTargetPosition(), AUTO_STEP_TOLERANCE);
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

    // Stop all of the motors
    public void stop() {
        Arrays.stream(motors).forEach(motor -> motor.setPower(0));
    }

    public void setMotorPositions(int position) {
        Arrays.stream(motors).forEach(motor -> motor.setTargetPosition(position));
    }

    public int getNormalizedAngle() {
        return (int) AngleUnit.normalizeDegrees(imu.getAngularOrientation().firstAngle);
    }

    public void printData(Telemetry telemetry) {
        telemetry.addLine("--- Drive base ---");
        telemetry.addData("Position", frontLeftMotor.getCurrentPosition());
        telemetry.addData("Power", frontLeftMotor.getPower());
        telemetry.addData("Velocity", frontLeftMotor.getVelocity());
        telemetry.addData("Current (amps)", frontLeftMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Is over current?", frontLeftMotor.isOverCurrent());
    }

    private void setMotorMode(DcMotor.RunMode runMode) {
        Arrays.stream(motors).forEach(motor -> motor.setMode(runMode));
    }

    /**
     * Takes a joystick input and clips it.
     * @param input The input to scale
     * @param isScaled If we want to scale the input
     * @return The input clipped between -1 and 1
     */
    private double scaleInput(double input, boolean isScaled) {
        if (isScaled) {
            // Take the input (forward, strafe, turn) and scale it so that moving the joystick halfway doesn't use half power
            // Current formula just cubes the input and multiplies it by the multiplier
            return  Range.clip(Math.pow(input, 3) * INPUT_MULTIPLIER, -1, 1);
        }
        else {
            // Otherwise, we just multiply the input by the multiplier
            return Range.clip(input * INPUT_MULTIPLIER, -1, 1);
        }
    }

    private double scaleInput(double input) {
        return scaleInput(input, SCALED);
    }

    private boolean isWithinTolerance(double input, double target, double tolerance) {
        return Math.abs(input - target) <= tolerance;
    }

    private double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public BNO055IMU getGyro() {
        return imu;
    }

    private double getAveragePosition() {
        return (double) (
                frontLeftMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition() +
                rearLeftMotor.getCurrentPosition() + rearRightMotor.getCurrentPosition()
                ) / 4;
    }
}
