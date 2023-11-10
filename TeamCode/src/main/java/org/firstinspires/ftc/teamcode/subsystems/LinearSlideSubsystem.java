package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    public void extendManually() {
        state = PIDSubsystemState.MANUAL;
        if (isSafeToMove())
            slideMotor.setPower(EXTEND_POWER);
        else
            stop();
    }

    public void retractManually() {
        state = PIDSubsystemState.MANUAL;
        if (isSafeToMove())
            slideMotor.setPower(RETRACT_POWER);
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
        telemetry.addData("Position", slideMotor.getCurrentPosition());
        telemetry.addData("Target", target);
        telemetry.addData("Error", Math.abs(slideMotor.getCurrentPosition() - target));
        telemetry.addData("Last power", lastPower);
        telemetry.addData("Last last power", lastLastPower);
    }

    public boolean isSafeToMove() {
        return true;
//        return !robot.getElbowSubsystem().isBelowLevel();
    }

    public boolean isAtTarget() {
        return state == PIDSubsystemState.AT_TARGET;
    }
}
