package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import static org.firstinspires.ftc.teamcode.Constants.LinearSlideConstants.*;
import org.firstinspires.ftc.teamcode.Constants.PIDSubsystemState;

/**
 * A subsystem that represents the motor that controls the slide.
 *
 * @author Esquimalt Atom Smashers
 */
@Config
public class LinearSlideSubsystem extends CustomSubsystemBase {
    private final DcMotorEx slideMotor;

    private final PIDController controller;
    private static double target = 0;

    private double lastPower;
//    private double lastLastPower;

    private PIDSubsystemState state;

    private ElapsedTime timer;
    private double timeout;

    private final DigitalChannel slideLimitSwitch;

    /**
     * Constructs a new LinearSlideSubsystem.
     *
     * @param hardwareMap The hardware map of the robot
     * @param telemetry The telemetry of the robot
     */
    public LinearSlideSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        slideMotor = hardwareMap.get(DcMotorEx.class, SLIDE_MOTOR_NAME);
        configureSlide();

        controller = new PIDController(P, I, D);

        slideLimitSwitch = hardwareMap.get(DigitalChannel.class, SLIDE_LIMIT_SWITCH_NAME);
        slideLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        state = PIDSubsystemState.MANUAL;
    }

    /** Configure the slide motor by setting the direction and zero power behavior */
    private void configureSlide() {
        slideMotor.setDirection(SLIDE_MOTOR_DIRECTION);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    /** Reset the encoders on the slide motor. */
    public void resetEncoder() {
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Extend the slide manually.
     *
     * @param input Speed input
     */
    public void moveManually(double input) {
        state = PIDSubsystemState.MANUAL;
        if (slideMotor.getCurrentPosition() >= MAX_POSITION && input < 0) {
            stopMotor();
            return;
        }
        if (input > 0 && isLimitSwitchPressed()) {
            stopMotor();
            resetEncoder();
            return;
        }
        slideMotor.setPower(input * SLIDE_MANUAL_POWER_MULTIPLIER);
    }

    /**
     * Retract the slide manually.
     *
     * @param multiplier Speed multiplier
     */
    @Deprecated
    public void retractManually(double multiplier) {
        state = PIDSubsystemState.MANUAL;
        if (isLimitSwitchPressed()) {
            stopMotor();
            resetEncoder();
            return;
        }
        slideMotor.setPower(SLIDE_MANUAL_POWER_MULTIPLIER * multiplier);
    }

    /** Stops the slide motor. */
    public void stopMotor() {
        slideMotor.setPower(0);
    }

    /**
     * Checks whether we are at the max extension.
     *
     * @return If we are at the maximum position
     */
    @Deprecated
    public boolean isMaxExtension() {
        return slideMotor.getCurrentPosition() >= MAX_POSITION;
    }

    /**
     * Checks whether we are at the max retraction.
     *
     * @return If we are at the minimum position
     */
    @Deprecated
    public boolean isMaxRetraction() {
        return slideMotor.getCurrentPosition() <= MIN_POSITION;
    }

    /**
     * Sets the target position and sets the mode to use PIDs.
     *
     * @param targetPosition The new target position
     */
    public void setTarget(double targetPosition, double timeout) {
        state = PIDSubsystemState.MOVING_TO_TARGET;
        if (targetPosition < MIN_POSITION || targetPosition > MAX_POSITION) return;
        target = targetPosition;
        if (timer == null) timer = new ElapsedTime();
        else timer.reset();
        this.timeout = timeout;
    }

    /** Runs the PID controllers if we are moving to a target. If we are close enough to the target, get out of PID mode. */
    public void runPID() {
        if (state == PIDSubsystemState.MOVING_TO_TARGET) {
            if (target == 0) {
                lastPower = -1;
                slideMotor.setPower(-1);
                if (isLimitSwitchPressed() || isTimeoutDone()) {
                    if (isLimitSwitchPressed()) resetEncoder();
                    stopMotor();
                    state = PIDSubsystemState.AT_TARGET;
                    return;
                }
            }
            else {
                if (target < slideMotor.getCurrentPosition() && isLimitSwitchPressed()) {
                    stopMotor();
                    resetEncoder();
                    state = PIDSubsystemState.AT_TARGET;
                    return;
                }
                // Calculate how much we need to move the motor by
                controller.setPID(P, I, D);
                int slidePosition = slideMotor.getCurrentPosition();
                double power = controller.calculate(slidePosition, target);
                slideMotor.setPower(power);
                lastPower = power;
                // If the power isn't much, we are about as close to the target as we are going to get,
                // so we don't update anymore
                // Or, if the timer is over the timeout, we also stop
                if (Math.abs(power) <= POWER_TOLERANCE || isTimeoutDone()) {
                    state = PIDSubsystemState.AT_TARGET;
    //                lastLastPower = power;
                    stopMotor();
                }
            }
        }
    }

    private boolean isTimeoutDone() {
        return timeout > 0 && timer.seconds() >= timeout;
    }

    public int getPosition() {
        return slideMotor.getCurrentPosition();
    }

    /** Prints data from the slide motor. */
    @Override
    public void printData() {
        telemetry.addLine("--- Slide ---");
        telemetry.addData("Slide Position", slideMotor.getCurrentPosition());
        telemetry.addData("Slide last power", lastPower);
        telemetry.addData("Is limit pressed?", isLimitSwitchPressed());
        telemetry.addData("Target", target);
        telemetry.addLine("Comparing " + Math.abs(lastPower) + " and " + POWER_TOLERANCE);
        telemetry.addData("Result", Math.abs(lastPower) <= POWER_TOLERANCE);
//        telemetry.addData("Target", target);
//        telemetry.addData("State", state);
//        telemetry.addData("Power", slideMotor.getPower());
//        telemetry.addData("Velocity", slideMotor.getVelocity());
//        telemetry.addData("Current (amps)", slideMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Is over current?", slideMotor.isOverCurrent());
    }

    /** @return true if the motor is at the target, false otherwise. */
    public boolean isAtTarget() {
        return state == PIDSubsystemState.AT_TARGET;
    }

    /** @return the preset low scoring position */
    public int getLowScoringPosition() {
        return LOW_SCORING_POSITION;
    }

    /** @return the preset medium scoring position */
    public int getMediumScoringPosition() {
        return MEDIUM_SCORING_POSITION;
    }

    /** @return the preset high scoring position */
    public int getHighScoringPosition() {
        return HIGH_SCORING_POSITION;
    }

    /** @return the minimum position of the slide */
    public int getInPosition() {
        return IN_POSITION;
    }

    public int getAutoScoringPosition() {
        return AUTO_SCORING_POSITION;
    }

    private boolean isLimitSwitchPressed() {
        return !slideLimitSwitch.getState();
    }
}
