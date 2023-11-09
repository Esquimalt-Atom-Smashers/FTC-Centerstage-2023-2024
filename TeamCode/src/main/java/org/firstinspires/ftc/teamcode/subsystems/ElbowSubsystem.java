package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import static org.firstinspires.ftc.teamcode.Constants.ElbowConstants.*;
import org.firstinspires.ftc.teamcode.Constants.PIDSubsystemState;

public class ElbowSubsystem extends SubsystemBase {
    // Motor used for this subsystem
    private final DcMotorEx elbowMotor;

    private final PIDController controller;

    public static double target;
    private double lastPower;

    private PIDSubsystemState state;

    public ElbowSubsystem(HardwareMap hardwareMap) {
        elbowMotor = hardwareMap.get(DcMotorEx.class, ELBOW_DC_MOTOR_NAME);
        elbowMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        controller = new PIDController(P, I, D);

        state = PIDSubsystemState.MANUAL;
    }

    public void intakePosition() {
        setTarget(INTAKE_POSITION);
    }

    public void drivingPosition() {
        setTarget(DRIVING_POSITION);
    }

    public void levelPosition() {
        setTarget(LEVEL_POSITION);
    }

    public void testPosition() {
        setTarget(TEST_POSITION);
    }

    public void tiltPosition() {
        setTarget(TILT_POSITION);
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
        elbowMotor.setPower(0);
    }

    public void raiseManually(double multiplier) {
        state = PIDSubsystemState.MANUAL;
        elbowMotor.setPower(MANUAL_MOTOR_SPEED * multiplier);
    }

    public void lowerManually(double multiplier) {
        state = PIDSubsystemState.MANUAL;
        elbowMotor.setPower(-MANUAL_MOTOR_SPEED * multiplier);
    }

    public void setTarget(double targetPosition) {
        target = targetPosition;
        state = PIDSubsystemState.MOVING_TO_TARGET;
    }

    public void printData(Telemetry telemetry) {
        telemetry.addLine("--- Elbow Subsystem ---");
        telemetry.addData("Position", elbowMotor.getCurrentPosition());
        telemetry.addData("Target", target);
        telemetry.addData("State", state);
        telemetry.addLine("--- ---");
    }

    public void runPID() {
        // If we aren't at the target
        if (state == PIDSubsystemState.MOVING_TO_TARGET)
        {
            // Calculate how much we need to move the motor by
            controller.setPID(P, I, D);
            int elbowPosition = elbowMotor.getCurrentPosition();
            double power = controller.calculate(elbowPosition, target);
            lastPower = power;
            elbowMotor.setPower(power);
            // If the power we are setting is basically none, we are close enough to the target
            if (Math.abs(power) <= POWER_TOLERANCE) {
                state = PIDSubsystemState.AT_TARGET;
            }
        }
    }

    public boolean isBelowLevel() {
        return elbowMotor.getCurrentPosition() <= LEVEL_POSITION;
    }

    public boolean isAtTarget() {
        return state != PIDSubsystemState.MOVING_TO_TARGET;
    }
}
