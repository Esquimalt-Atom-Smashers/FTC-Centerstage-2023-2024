package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.ElbowConstants.*;

public class ElbowSubsystem {
    private final DcMotorEx elbowMotor;

    private PIDController controller;

    public static double target;
    private boolean atTarget = false;
    private double lastPower;

    public  ElbowSubsystem(HardwareMap hardwareMap) {
        elbowMotor = hardwareMap.get(DcMotorEx.class, ELBOW_DC_MOTOR_NAME);
        elbowMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        controller = new PIDController(P, I, D);
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

    public void raiseManually() {
        elbowMotor.setPower(MANUAL_MOTOR_SPEED);
    }

    public void lowerManually() {
        elbowMotor.setPower(-MANUAL_MOTOR_SPEED);
    }

    private void setTarget(double targetPosition) {
        target = targetPosition;
        atTarget = false;
    }

    public void printPosition(Telemetry telemetry) {
        telemetry.addData("Position", elbowMotor.getCurrentPosition());
        telemetry.addData("Target", target);
    }

    public void runPID() {
        // If we aren't at the target
        if (!atTarget)
        {
            // Calculate how much we need to move the motor by
            controller.setPID(P, I, D);
            int elbowPosition = elbowMotor.getCurrentPosition();
            double power = controller.calculate(elbowPosition, target);
            lastPower = power;
            elbowMotor.setPower(power);
            // If the power we are setting is basically none, we are close enough to the target
            atTarget = Math.abs(power) <= POWER_TOLERANCE;
        }
    }

    public boolean isAtTarget() {
        return atTarget;
    }
}
