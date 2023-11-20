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

/**
 * A subsystem that represents the motor that controls the arm.
 */
public class ElbowSubsystem extends SubsystemBase {
    // Motor used for this subsystem
    private final DcMotorEx elbowMotor;

    private final PIDController controller;

    public static double target;
    private double lastPower;

    private PIDSubsystemState state;

    /**
     * Creates a new ElbowSubsystem. Initializes the {@link DcMotorEx} using the provided {@link HardwareMap}. Initializes a PID controller.
     * @param hardwareMap The hardware map of the robot
     */
    public ElbowSubsystem(HardwareMap hardwareMap) {
        elbowMotor = hardwareMap.get(DcMotorEx.class, ELBOW_DC_MOTOR_NAME);
        elbowMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        controller = new PIDController(P, I, D);

        state = PIDSubsystemState.MANUAL;
    }

    /**
     * Sets the target to intake position, used for picking up pixels.
     */
    public void intakePosition() {
        setTarget(INTAKE_POSITION);
    }

    /**
     * Sets the target to driving position, used when driving.
     */
    public void drivingPosition() {
        setTarget(DRIVING_POSITION);
    }

    /**
     * Sets the target to level position.
     */
    public void levelPosition() {
        setTarget(LEVEL_POSITION);
    }

    /**
     * Sets the target to test position, used for testing where position are.
     */
    public void testPosition() {
        setTarget(TEST_POSITION);
    }

    /**
     * Sets the target to tilt position, used to push the pixels against the intake to angle better for the board.
     */
    public void tiltPosition() {
        setTarget(TILT_POSITION);
    }

    /**
     * Sets the target to low scoring position, used for scoring from the first set line and down.
     */
    public void lowScoringPosition() {
        setTarget(LOW_SCORING_POSITION);
    }

    /**
     * Sets the target to medium scoring position, used for scoring from around the second set line to the first.
     */
    public void mediumScoringPosition() {
        setTarget(MEDIUM_SCORING_POSITION);
    }

    /**
     * Sets the target to high scoring position, used for scoring on the highest level.
     */
    public void highScoringPosition() {
        setTarget(HIGH_SCORING_POSITION);
    }

    /**
     * Stop the arm from moving.
     */
    public void stop() {
        elbowMotor.setPower(0);
    }

    /**
     * Raise the arm manually.
     * @param multiplier A multiplier for the speed of the motor
     */
    public void raiseManually(double multiplier) {
        state = PIDSubsystemState.MANUAL;
        elbowMotor.setPower(MANUAL_MOTOR_SPEED * multiplier);
    }

    /**
     * Lower the arm manually
     * @param multiplier A multiplier for the speed of the motor
     */
    public void lowerManually(double multiplier) {
        state = PIDSubsystemState.MANUAL;
        elbowMotor.setPower(-MANUAL_MOTOR_SPEED * multiplier);
    }

    /**
     * Set the target for the PID controller. This will make the robot start moving the arm automatically.
     * @param targetPosition The target position in pulses
     */
    public void setTarget(double targetPosition) {
        target = targetPosition;
        state = PIDSubsystemState.MOVING_TO_TARGET;
    }

    /**
     * Print data to the provided telemtery. Assumes the telemetry will be updated elsewhere.
     * @param telemetry The telemetry to print to
     */
    public void printData(Telemetry telemetry) {
        telemetry.addLine("--- Elbow Subsystem ---");
        telemetry.addData("Position", elbowMotor.getCurrentPosition());
        telemetry.addData("Target", target);
        telemetry.addData("State", state);
    }

    /**
     * Use the PID controller to calculate how fast we should set the motor to. If the motor is moving slow enough,
     * we are close enough and stop moving further.
     */
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

    /**
     * Checks if the arm is below level, for safety concerns with the slide.
     * @return Whether the arm is below level/flat
     */
    public boolean isBelowLevel() {
        return elbowMotor.getCurrentPosition() <= LEVEL_POSITION;
    }

    /**
     * Checks if the motor is at the target.
     * @return Whether the motor is at the target
     */
    public boolean isAtTarget() {
        return state == PIDSubsystemState.AT_TARGET;
    }
}
