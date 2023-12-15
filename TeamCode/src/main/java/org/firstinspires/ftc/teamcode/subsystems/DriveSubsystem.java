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

import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.*;

import java.util.Arrays;

/**
 * A subsystem that represents the drive base of the robot. Uses four motors and a gyro to drive.
 *
 * @author Esquimalt Atom Smashers
 */
public class DriveSubsystem extends CustomSubsystemBase {
    /** The DC motors on the robot. */
    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx rearLeftMotor;
    private final DcMotorEx rearRightMotor;
    private final DcMotorEx[] motors;

    /** The built-in IMU(gyro) on the control hub. */
    private final BNO055IMU imu;
    private double offset;

//    private double snapTarget;

//    public static double forwardTarget;
//    private boolean atForwardTarget;

    /**
     * Constructs a new DriveSubsystem.
     *
     * @param hardwareMap The hardware map of the robot
     * @param telemetry The telemetry of the robot
     */
    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_MOTOR_NAME);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_MOTOR_NAME);
        rearLeftMotor = hardwareMap.get(DcMotorEx.class, REAR_LEFT_MOTOR_NAME);
        rearRightMotor = hardwareMap.get(DcMotorEx.class, REAR_RIGHT_MOTOR_NAME);
        motors = new DcMotorEx[]{frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor};
        configureMotors();

        imu = hardwareMap.get(BNO055IMU.class, IMU_NAME);
        configureIMU();
    }

    /** Configure the drive motors by setting their directions and zero power behaviors. */
    private void configureMotors() {
        // Set the direction of the motors
        frontLeftMotor.setDirection(FRONT_LEFT_MOTOR_DIRECTION);
        frontRightMotor.setDirection(FRONT_RIGHT_MOTOR_DIRECTION);
        rearLeftMotor.setDirection(REAR_LEFT_MOTOR_DIRECTION);
        rearRightMotor.setDirection(REAR_RIGHT_MOTOR_DIRECTION);

        // Set the motor modes and zero power behavior
        Arrays.stream(motors).forEach(motor -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
        resetEncoder();
    }

    /** Configure the gyro */
    private void configureIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }

    /** Reset the gyro by setting the offset to the current heading */
    public void resetGyro() {
        offset = getRawHeading();
    }

    /** Reset the encoders on the drive motors */
    public void resetEncoder() {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Drives the robot using joystick input.
     *
     * @param forward The amount to move forward
     * @param strafe The amount to move left and right
     * @param turn The amount to turn
     * @param fieldCentric If we want this drive to be field centric
     * @param scaled If we want the inputs to be scaled
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
     *
     * @param forward The amount to move forward
     * @param strafe The amount to move left and right
     * @param turn The amount to turn
     */
    public void drive(double forward, double strafe, double turn) {
        drive(forward, strafe, turn, FIELD_CENTRIC, SCALED);
    }

//    public boolean forward() {
//        return frontLeftMotor.getCurrentPosition() < frontLeftMotor.getTargetPosition();
//    }

//    public boolean isFinishedSnapping() {
////         Checks if we are within the tolerance to auto snap
//        return isWithinTolerance(getNormalizedAngle(), snapTarget, AUTO_SNAP_TOLERANCE);
//    }

//    public boolean isFinishedSteppingLeft() {
//        return isWithinTolerance(frontLeftMotor.getCurrentPosition(), frontLeftMotor.getTargetPosition(), AUTO_STEP_TOLERANCE) &&
//                isWithinTolerance(frontRightMotor.getCurrentPosition(), frontRightMotor.getTargetPosition(), AUTO_STEP_TOLERANCE) &&
//                isWithinTolerance(rearLeftMotor.getCurrentPosition(), rearLeftMotor.getTargetPosition(), AUTO_STEP_TOLERANCE) &&
//                isWithinTolerance(rearRightMotor.getCurrentPosition(), rearRightMotor.getTargetPosition(), AUTO_STEP_TOLERANCE);
//    }

//    public boolean isFinishedSteppingRight() {
//        return isWithinTolerance(frontLeftMotor.getCurrentPosition(), frontLeftMotor.getTargetPosition(), AUTO_STEP_TOLERANCE) &&
//                isWithinTolerance(frontRightMotor.getCurrentPosition(), frontRightMotor.getTargetPosition(), AUTO_STEP_TOLERANCE) &&
//                isWithinTolerance(rearLeftMotor.getCurrentPosition(), rearLeftMotor.getTargetPosition(), AUTO_STEP_TOLERANCE) &&
//                isWithinTolerance(rearRightMotor.getCurrentPosition(), rearRightMotor.getTargetPosition(), AUTO_STEP_TOLERANCE);
//    }

    /** Stop all of the drive motors */
    public void stopMotors() {
        Arrays.stream(motors).forEach(motor -> motor.setPower(0));
    }

    /**
     * Sets the mode of all the drive motors to the specified run mode.
     *
     * @param runMode The new run mode for the motors
     */
    private void setMotorMode(DcMotor.RunMode runMode) {
        Arrays.stream(motors).forEach(motor -> motor.setMode(runMode));
    }

    /** @return The heading of the robot */
    private double getHeading() {
        return getRawHeading() - offset;
    }

    private double getRawHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    /** @return The normalized angle in degrees */
    public int getNormalizedAngle() {
        return (int) AngleUnit.normalizeDegrees(imu.getAngularOrientation().firstAngle);
    }

    /** Prints data from the motors to the telemetry */
    public void printData() {
        telemetry.addLine("--- Drive base ---");
        telemetry.addData("Position", frontLeftMotor.getCurrentPosition());
        telemetry.addData("Power", frontLeftMotor.getPower());
        telemetry.addData("Velocity", frontLeftMotor.getVelocity());
        telemetry.addData("Current (amps)", frontLeftMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Is over current?", frontLeftMotor.isOverCurrent());
    }

    /**
     * Takes a joystick input and clips it.
     *
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

    /**
     * Checks if a value is close enough to the target.
     *
     * @param input Input value
     * @param target Target value
     * @param tolerance How far away we can be
     * @return Whether the input value is within tolerance away from target
     */
    private boolean isWithinTolerance(double input, double target, double tolerance) {
        return Math.abs(input - target) <= tolerance;
    }
}
