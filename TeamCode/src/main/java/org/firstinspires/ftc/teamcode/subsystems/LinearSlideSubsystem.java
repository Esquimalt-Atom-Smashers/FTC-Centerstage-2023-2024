package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Constants.LinearSlideConstants.*;
import org.firstinspires.ftc.teamcode.Constants.PIDSubsystemState;

import java.util.Arrays;

/**
 * A subsystem that represents the motor and PID controller that controls the slide.
 */
public class LinearSlideSubsystem extends SubsystemBase {
    // Motor used in this subsystem
    private final DcMotorEx slideMotor;

    private PIDController controller;
    public static double target;
    private double lastPower;
    private double lastLastPower;

    private PIDSubsystemState state;

    /**
     * Creates a new LinearSlideSubsystem.
     * @param hardwareMap The hardware map of the robot
     */
    public LinearSlideSubsystem(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotorEx.class, SLIDE_MOTOR_NAME);

        slideMotor.setDirection(SLIDE_MOTOR_DIRECTION);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDController(P, I, D);

        state = PIDSubsystemState.MANUAL;
    }

    /**
     * Reset the encoders on the slide motor.
     */
    public void resetEncoder() {
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Extend the slide manually.
     * @param multiplier Speed multiplier
     */
    public void extendManually(double multiplier) {
        state = PIDSubsystemState.MANUAL;
        slideMotor.setPower(EXTEND_POWER * multiplier);
    }

    /**
     * Retract the slide manually.
     * @param multiplier Speed multiplier
     */
    public void retractManually(double multiplier) {
        state = PIDSubsystemState.MANUAL;
        slideMotor.setPower(RETRACT_POWER * multiplier);
    }

    /**
     * Stops the slide motor.
     */
    public void stop() {
        slideMotor.setPower(0);
    }

    /**
     * Checks whether we are at the max extension.
     * @return If we are at the maximum position
     */
    public boolean isMaxExtension() {
        return slideMotor.getCurrentPosition() >= MAX_POSITION;
    }

    /**
     * Checks whether we are at the max retraction.
     * @return If we are at the minimum position
     */
    public boolean isMaxRetraction() {
        return slideMotor.getCurrentPosition() <= MIN_POSITION;
    }

    /**
     * Sets the target position and sets the mode to use PIDs.
     * @param targetPosition The new target position
     */
    public void setTarget(double targetPosition) {
        state = PIDSubsystemState.MOVING_TO_TARGET;
        if (targetPosition < MIN_POSITION || targetPosition > MAX_POSITION) return;
        target = targetPosition;
    }

    /**
     * Runs the PID controllers if we are moving to a target.
     * If we are close enough to the target, get out of PID mode.
     */
    public void runPID() {
        if (state == PIDSubsystemState.MOVING_TO_TARGET) {
            // Calculate how much we need to move the motor by
            controller.setPID(P, I, D);
            int slidePosition = slideMotor.getCurrentPosition();
            double power = controller.calculate(slidePosition, target);
            slideMotor.setPower(power);
            // If the power isn't much, we are about as close to the target as we are going to get,
            // so we don't update anymore
            lastPower = power;
            if (Math.abs(power) <= POWER_TOLERANCE) {
                state = PIDSubsystemState.AT_TARGET;
                lastLastPower = power;
                stop();
            }
        }
    }

    /**
     * Prints data to the provided telemetry.
     * @param telemetry The telemetry to print to
     */
    public void printData(Telemetry telemetry) {
        telemetry.addLine("--- Slide ---");
        telemetry.addData("State", state);
        telemetry.addData("Position", slideMotor.getCurrentPosition());
        telemetry.addData("Target", target);
        telemetry.addData("Power", slideMotor.getPower());
        telemetry.addData("Velocity", slideMotor.getVelocity());
        telemetry.addData("Current (amps)", slideMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Is over current?", slideMotor.isOverCurrent());
    }

    /**
     * Checks if the motor is at the target position.
     * @return Whether the motor is at the target
     */
    public boolean isAtTarget() {
        return state == PIDSubsystemState.AT_TARGET;
    }

    /**
     * @return Returns the low scoring position
     */
    public int getLowScoringPosition() {
        return LOW_SCORING_POSITION;
    }

    /**
     * @return Returns the medium scoring position
     */
    public int getMediumScoringPosition() {
        return MEDIUM_SCORING_POSITION;
    }

    /**
     * @return Returns the high scoring position
     */
    public int getHighScoringPosition() {
        return HIGH_SCORING_POSITION;
    }

    /**
     * @return Returns the minimum position
     */
    public int getInPosition() {
        return IN_POSITION;
    }
}
