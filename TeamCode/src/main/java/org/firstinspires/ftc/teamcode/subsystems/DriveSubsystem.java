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

/**
 * A subsystem that represents the drive base of the robot. This includes four motors and a
 * gyro. Uses the four motors and the gyro to drive.
 *
 * @author Esquimalt Atom Smashers
 */
public class DriveSubsystem extends CustomSubsystemBase {
    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx rearLeftMotor;
    private final DcMotorEx rearRightMotor;
    /** The DC motors on the robot. */
    private final DcMotorEx[] motors;

    /** The built-in IMU(gyro) on the control hub. */
    private final BHI260IMU imu;

    private enum DriveState {
        MANUAL,
        MOVING_TO_POSITION,
        TURNING_TO_POSITION
    }

    private DriveState driveState = DriveState.MANUAL;

    private double targetHeading;

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
     * @param fieldCentric If we want these movements to be field centric
     * @param multiplier The speed multiplier
     */
    public void drive(double forward, double strafe, double turn, boolean fieldCentric, double multiplier) {
        // Dead zone for the joysticks
        forward = Math.abs(forward) >= DEADZONE ? forward : 0;
        strafe = Math.abs(strafe) >= DEADZONE ? strafe : 0;
        turn = Math.abs(turn) >= DEADZONE ? turn : 0;
        multiplier = Range.clip(multiplier, 0, 1);

        if (fieldCentric) {
            // Field centric drive
            double gyroRadians = Math.toRadians(-getHeading());
            double fieldCentricStrafe = strafe * Math.cos(gyroRadians) - forward * Math.sin(gyroRadians);
            double fieldCentricDrive = strafe * Math.sin(gyroRadians) + forward * Math.cos(gyroRadians);

            frontLeftMotor.setPower(scaleInput(fieldCentricDrive - fieldCentricStrafe + turn, multiplier));
            frontRightMotor.setPower(scaleInput(fieldCentricDrive - fieldCentricStrafe - turn, multiplier));
            rearLeftMotor.setPower(scaleInput(fieldCentricDrive + fieldCentricStrafe + turn, multiplier));
            rearRightMotor.setPower(scaleInput(fieldCentricDrive + fieldCentricStrafe - turn, multiplier));
        }
        else {
            // Robot centric drive
            frontLeftMotor.setPower(scaleInput(forward - strafe + turn, multiplier));
            frontRightMotor.setPower(scaleInput(forward - strafe - turn, multiplier));
            rearLeftMotor.setPower(scaleInput(forward + strafe + turn, multiplier));
            rearRightMotor.setPower(scaleInput(forward + strafe - turn, multiplier));
        }

    }

    /**
     * Drives the robot using joystick input. Uses the default value for field centric and full power.
     *
     * @param forward The amount to move forward
     * @param strafe The amount to move left and right
     * @param turn The amount to turn left and right
     */
    public void drive(double forward, double strafe, double turn) {
        drive(forward, strafe, turn, FIELD_CENTRIC, 1);
    }

    /**
     * Drives the robot using a gamepad. Uses the default value for field centric.
     *
     * @param gamepad The gamepad controlling the robot
     * @param speedMultiplier The speed multiplier
     */
    public void drive(GamepadEx gamepad, double speedMultiplier) {
        drive(gamepad.getLeftY(), gamepad.getLeftX(), gamepad.getRightX(), FIELD_CENTRIC, speedMultiplier);
    }


    /**
     * Start moving the robot some number of inches forwards/backwards using the encoders. Sets the target position of
     * the motors and starts them moving. {@link #isFinishedMoving()} must be called to check if the robot has made it.
     *
     * @param inches The distance in inches to drive forwards/backwards
     */
    public void driveByDistanceAsync(double inches) {
        driveState = DriveState.MOVING_TO_POSITION;
        Arrays.stream(motors).forEach(motor ->
                motor.setTargetPosition(motor.getCurrentPosition() + toPulses(inches)));
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive(AUTO_DRIVE_SPEED, 0, 0, false, 1);
    }

    /**
     * Drive the robot some number of inches forwards/backwards.
     *
     * @param inches The distance in inches to drive forwards/backwards
     */
    public void driveByDistance(double inches) {
        driveByDistanceAsync(inches);
        while (!isFinishedMoving()) {doNothing("Driving");}
    }

    /**
     * Start moving the robot some number of inches right/left using the encoders. Sets the target position of the
     * motors and starts them moving. {@link #isFinishedMoving()} must be called to check if the robot has made it.
     *
     * @param inches The distance in inches to drive right/left
     */
    public void strafeByDistanceAsync(double inches) {
        driveState = DriveState.MOVING_TO_POSITION;
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() - toPulses(inches));
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - toPulses(inches));
        rearLeftMotor.setTargetPosition(rearLeftMotor.getCurrentPosition() + toPulses(inches));
        rearRightMotor.setTargetPosition(rearRightMotor.getCurrentPosition() + toPulses(inches));
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive(0, AUTO_STRAFE_SPEED, 0, false, 1);
    }

    /**
     * Strafes the robot some number of inches right/left.
     *
     * @param inches The number of inches to strafe right/left
     */
    public void strafeByDistance(double inches) {
        strafeByDistanceAsync(inches);
        while (!isFinishedMoving()) {doNothing("Strafing");}
    }

    /**
     * Turn the robot some angle using the gyro. Sets the target heading and starts moving the motors.
     * {@link #isFinishedTurning()} must be called to check if the robot has made it.
     *
     * @param angle The angle to rotate the robot by
     * @param speed The speed at which the robot rotates (0-1)
     */
    public void turnAsync(double angle, double speed) {
        driveState = DriveState.TURNING_TO_POSITION;
        targetHeading = getHeading() + angle;
        // Counter clockwise is positive
        // If the angle is positive, we want to turn negative (counterclockwise)
        drive(0, 0, getAutoTurnSpeed(speed), false, 1);
    }

    /**
     * Turns the robot some angle using the gyro. Waits half a second and corrects a little bit
     *
     * @param angle The angle to turn the robot by
     */
    public void turn(double angle) {
        turnAsync(angle, AUTO_TURN_SPEED);
        while (!isFinishedTurning()) {doNothing("Turning");}

        // Wait for a bit
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() <= 500) {doNothing("Waiting");}

        angle = targetHeading - getHeading();
        turnAsync(angle, AUTO_TURN_SPEED / 2);
        while (!isFinishedTurning()) {doNothing("Correcting turn");}
    }

    /**
     * Gets the speed we should be turning. Used while turning automatically.
     *
     * @param speed The absolute value of the speed
     * @return What speed we should turn
     */
    public double getAutoTurnSpeed(double speed) {
        double angle = targetHeading - getHeading();
        return angle > 0 ? -speed : speed;
    }

    /** @return What speed we should turn, assumes we are using the default auto turn speed */
    public double getAutoTurnSpeed() {
        return getAutoTurnSpeed(AUTO_TURN_SPEED);
    }

    /** @return True is all of the motors are busy, false otherwise */
    private boolean motorsBusy() {
        return frontLeftMotor.isBusy() && frontRightMotor.isBusy() && rearRightMotor.isBusy() && rearLeftMotor.isBusy();
    }

    /**
     * Checks if we are done driving or strafing, and if we are done, stops the motors and
     * sets the state back to manual.
     *
     * @return True if we are done moving, false otherwise
     */
    public boolean isFinishedMoving() {
        if (!motorsBusy()) {
            stopMotors();
            setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveState = DriveState.MANUAL;
            return true;
        }
        return false;
    }

    /**
     * Checks if we are done turning, and if we are done, stops the motors and sets the
     * state back to manual.
     *
     * @return True if we are done moving, false otherwise
     */
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

    /** Reset the gyro by resetting the yaw. */
    public void resetGyro() {
        imu.resetYaw();
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

    private void doNothing(String str) {
        telemetry.addLine("Waiting, current task: " + str);
        telemetry.update();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    /**
     * Takes a joystick input and clips it.
     *
     * @param input The input to scale
     * @return The input clipped between -1 and 1
     */
    private double scaleInput(double input, double multiplier) {
        return Range.clip(input * multiplier, -1, 1);
    }
}
