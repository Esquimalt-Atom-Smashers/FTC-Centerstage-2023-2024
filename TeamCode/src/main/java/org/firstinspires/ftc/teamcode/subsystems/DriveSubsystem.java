package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.*;

import java.util.Arrays;
//import java.util.function.DoubleSupplier;

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
    private final BHI260IMU imu; // Counter clockwise is positive
    private double offset;

    private double headingError = 0;

    private enum DriveState {
        MANUAL,
        MOVING_TO_POSITION,
        TURNING_TO_POSITION
    }

    private DriveState driveState = DriveState.MANUAL;

    private double targetHeading;

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

        imu = hardwareMap.get(BHI260IMU.class, IMU_NAME);
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
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(parameters);
        resetGyro();
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
    // TODO: Go through the overloads and check to see if they are needed
    public void drive(double forward, double strafe, double turn, boolean fieldCentric, boolean scaled, double multiplier) {
        forward = Math.abs(forward) >= DEADZONE ? forward : 0;
        strafe = Math.abs(strafe) >= DEADZONE ? strafe : 0;
        turn = Math.abs(turn) >= DEADZONE ? turn : 0;
        multiplier = Range.clip(multiplier, 0, 1);

        if (fieldCentric) {
            // Field centric drive
            double gyroRadians = Math.toRadians(-getHeading());
            double rotateX = strafe * Math.cos(gyroRadians) - forward * Math.sin(gyroRadians);
            double rotateY = strafe * Math.sin(gyroRadians) + forward * Math.cos(gyroRadians);

            frontLeftMotor.setPower(scaleInput(rotateY - rotateX + turn, multiplier, scaled));
            frontRightMotor.setPower(scaleInput(rotateY - rotateX - turn, multiplier, scaled));
            rearLeftMotor.setPower(scaleInput(rotateY + rotateX + turn, multiplier, scaled));
            rearRightMotor.setPower(scaleInput(rotateY + rotateX - turn, multiplier, scaled));
        }
        else {
            // Robot centric drive
            frontLeftMotor.setPower(scaleInput(forward - strafe + turn, multiplier, scaled));
            frontRightMotor.setPower(scaleInput(forward - strafe - turn, multiplier, scaled));
            rearLeftMotor.setPower(scaleInput(forward + strafe + turn, multiplier, scaled));
            rearRightMotor.setPower(scaleInput(forward + strafe - turn, multiplier, scaled));
        }

    }

    /**
     * Drives the robot using joystick input. Uses the default values for field centric and scaling.
     *
     * @param forward The amount to move forward
     * @param strafe The amount to move left and right
     * @param turn The amount to turn left and right
     */
    public void drive(double forward, double strafe, double turn, double speedMultiplier) {
        drive(forward, strafe, turn, FIELD_CENTRIC, SCALED, speedMultiplier);
    }

    public void drive(GamepadEx gamepad, double speedMultiplier) {
        drive(gamepad.getLeftY(), gamepad.getLeftX(), gamepad.getRightX(), FIELD_CENTRIC, SCALED, speedMultiplier);
    }

    // Starts the robot moving forward, you MUST call isFinishedMoving() repeatedly
    // to check if the robot has made it
    public void driveByDistanceAsync(double inches) {
        driveState = DriveState.MOVING_TO_POSITION;
        Arrays.stream(motors).forEach(motor ->
                motor.setTargetPosition(motor.getCurrentPosition() + toPulses(inches)));
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive(AUTO_DRIVE_SPEED, 0, 0, false, false, 1);
    }

    // Moves the robot inches forward and then stops it
    public void driveByDistance(double inches) {
        driveByDistanceAsync(inches);
        while (!isFinishedMoving()) {}
    }

    // Strafe the robot inches, you MUST call isFinishedMoving() repeatedly
    // to check if the robot has made it
    public void strafeByDistanceAsync(double inches) {
        driveState = DriveState.MOVING_TO_POSITION;
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() - toPulses(inches));
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - toPulses(inches));
        rearLeftMotor.setTargetPosition(rearLeftMotor.getCurrentPosition() + toPulses(inches));
        rearRightMotor.setTargetPosition(rearRightMotor.getCurrentPosition() + toPulses(inches));
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive(0, AUTO_STRAFE_SPEED, 0, false, false, 1);
    }

    // Strafes the robot inches and stops it when it gets there
    public void strafeByDistance(double inches) {
        strafeByDistanceAsync(inches);
        while (!isFinishedMoving()) {}
    }

    public void turnAsync(double angle, double speed) {
        driveState = DriveState.TURNING_TO_POSITION;
        targetHeading = getHeading() + angle;
        // Counter clockwise is positive
        // If the angle is positive, we want to turn negative (counterclockwise)
        drive(0, 0, getAutoTurnSpeed(speed), false, false, 1);
    }

    public void turn(double angle) {
        turnAsync(angle, AUTO_TURN_SPEED);
        while (!isFinishedTurning()) {}

        // Wait for a bit
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() <= 500) {}

        angle = targetHeading - getHeading();
        turnAsync(angle, AUTO_TURN_SPEED / 2);
        while (!isFinishedTurning()) {}

        telemetry.addLine("Turning complete! :)");
        telemetry.update();
    }

    public double getAutoTurnSpeed(double speed) {
        double angle = targetHeading - getHeading();
        return angle > 0 ? -speed : speed;
    }

    public double getAutoTurnSpeed() {
        return getAutoTurnSpeed(AUTO_TURN_SPEED);
    }



    private boolean motorsBusy() {
        return frontLeftMotor.isBusy() && frontRightMotor.isBusy() && rearRightMotor.isBusy() && rearLeftMotor.isBusy();
    }

    public boolean isFinishedMoving() {
        if (!motorsBusy()) {
            stopMotors();
            setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveState = DriveState.MANUAL;
            return true;
        }
        return false;
    }

    public boolean isFinishedTurning() {
        if (Math.abs(getHeading() - targetHeading) <= AUTO_HEADING_TOLERANCE) {
            stopMotors();
            driveState = DriveState.MANUAL;
            return true;
        }
        return false;
    }

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
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /** Reset the gyro by setting the offset to the current heading */
    public void resetGyro() {
        imu.resetYaw();
    }

    // TODO: Delete
    /** @return The normalized angle in degrees */
    public int getNormalizedAngle() {
        return -1;
//        return (int) AngleUnit.normalizeDegrees(imu.getAngularOrientation().firstAngle);
    }

    /** Prints data from the motors to the telemetry */
    @Override
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
    private double scaleInput(double input, double multiplier, boolean isScaled) {
        if (isScaled) {
            // Take the input (forward, strafe, turn) and scale it so that moving the joystick halfway doesn't use half power
            // Current formula just cubes the input and multiplies it by the multiplier
            return Range.clip(Math.pow(input, 3) * multiplier, -1, 1);
        }
        else {
            // Otherwise, we just multiply the input by the multiplier
            return Range.clip(input * multiplier, -1, 1);
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
