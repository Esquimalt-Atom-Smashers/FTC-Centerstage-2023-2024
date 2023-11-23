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

public class LinearSlideSubsystem extends SubsystemBase {
    // Motor used in this subsystem
    private final DcMotorEx slideMotor;

    private PIDController controller;
    public static double target;
    private double lastPower;
    private double lastLastPower;

    private PIDSubsystemState state;


    public LinearSlideSubsystem(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotorEx.class, SLIDE_MOTOR_NAME);

        slideMotor.setDirection(SLIDE_MOTOR_DIRECTION);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDController(P, I, D);

        state = PIDSubsystemState.MANUAL;
    }

    public void resetEncoder() {
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void extend() {
        setTarget(OUT_POSITION);
    }

    public void retract() {
        setTarget(IN_POSITION);
    }

    public void testPosition() {
        setTarget(TEST_POSITION);
    }

    public void tiltPosition() {
        setTarget(TILT_POSITION);
    }

    public void extendManually(double multiplier) {
        state = PIDSubsystemState.MANUAL;
        if (isSafeToMove())
            slideMotor.setPower(EXTEND_POWER * multiplier);
        else
            stop();
    }

    public void retractManually(double multiplier) {
        state = PIDSubsystemState.MANUAL;
        if (isSafeToMove())
            slideMotor.setPower(RETRACT_POWER * multiplier);
        else
            stop();
    }

    public void lowScoringPosition() {
        setTarget(LOW_SCORING_POSITION);
    }

    public void mediumScoringPosition() {
        setTarget(MEDIUM_SCORING_POSITION);
    }

    public void highScoringPosition() {
        setTarget(HIGH_SCORING_POSITION);
    }

    public void stop() {
        slideMotor.setPower(0);
    }

    public boolean isMaxExtension() {
        return slideMotor.getCurrentPosition() >= MAX_POSITION;
    }

    public boolean isMaxRetraction() {
        return slideMotor.getCurrentPosition() <= MIN_POSITION;
    }

    public void setTarget(double targetPosition) {
        state = PIDSubsystemState.MOVING_TO_TARGET;
        if (targetPosition < MIN_POSITION || targetPosition > MAX_POSITION) return;
        target = targetPosition;
    }

    public void runPID() {
        if (state == PIDSubsystemState.MOVING_TO_TARGET) {
            // Calculate how much we need to move the motor by
            controller.setPID(P, I, D);
            int slidePosition = slideMotor.getCurrentPosition();
            double power = controller.calculate(slidePosition, target);
            slideMotor.setPower(power);
            // If the power isn't much, we are about as close to the target as we are going to get, don't update anymore
            lastPower = power;
            if (Math.abs(power) <= POWER_TOLERANCE) {
                state = PIDSubsystemState.AT_TARGET;
                lastLastPower = power;
                stop();
            }
        }
    }

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

    public boolean isSafeToMove() {
        return true;
//        return !robot.getElbowSubsystem().isBelowLevel();
    }

    public boolean isAtTarget() {
        return state == PIDSubsystemState.AT_TARGET;
    }
}
