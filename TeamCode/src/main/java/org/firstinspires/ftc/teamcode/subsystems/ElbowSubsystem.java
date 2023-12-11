package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Constants.ElbowConstants.*;

import org.firstinspires.ftc.teamcode.Constants.PIDSubsystemState;

/**
 * A subsystem that represents the motor that controls the elbow of the arm.
 *
 * @author Esquimalt Atom Smashers
 */
public class ElbowSubsystem extends SubsystemBase {
    private final DcMotorEx elbowMotor;

    private final PIDController controller;

    public static double target;
//    private double lastPower;

    /** The state the arm is in: (manual, moving-to-target, or at-target) */
    private PIDSubsystemState state;

    /**
     * Constructs an ElbowSubsystem.
     *
     * @param hardwareMap The global hardwareMap.
     */
    public ElbowSubsystem(HardwareMap hardwareMap) {
        elbowMotor = hardwareMap.get(DcMotorEx.class, ELBOW_DC_MOTOR_NAME);
        elbowMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        controller = new PIDController(P, I, D);

        state = PIDSubsystemState.MANUAL;
    }

    /** Resets the encoder on the elbow motor. */
    public void resetEncoder() {
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Stops the elbow motor. */
    public void stopMotor() {
        elbowMotor.setPower(0);
    }

    /**
     * Raises the arm manually.
     *
     * @param multiplier A multiplier for the speed of the motor
     */
    public void raiseManually(double multiplier) {
        state = PIDSubsystemState.MANUAL;
        elbowMotor.setPower(MANUAL_MOTOR_SPEED * multiplier);
    }

    /**
     * Lowers the arm manually
     *
     * @param multiplier A multiplier for the speed of the motor
     */
    public void lowerManually(double multiplier) {
        state = PIDSubsystemState.MANUAL;
        elbowMotor.setPower(-MANUAL_MOTOR_SPEED * multiplier);
    }

    /**
     * Set the target for the PID controller. This will make the robot start moving the arm automatically.
     *
     * @param targetPosition The target position in pulses
     */
    public void setTarget(double targetPosition) {
        target = targetPosition;
        state = PIDSubsystemState.MOVING_TO_TARGET;
    }

    /**
     * Print data to the provided telemetry. Assumes the telemetry will be updated elsewhere.
     *
     * @param telemetry The telemetry to print to
     */
    public void printData(Telemetry telemetry) {
        telemetry.addLine("--- Elbow Subsystem ---");
        telemetry.addData("Position:", elbowMotor.getCurrentPosition());
        telemetry.addData("Target:", target);
        telemetry.addData("State:", state);
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
//            lastPower = power;
            elbowMotor.setPower(power);
            // If the power we are setting is basically none, we are close enough to the target
            if (Math.abs(power) <= POWER_TOLERANCE) {
                state = PIDSubsystemState.AT_TARGET;
            }
        }
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

    public int getDroneLaunchPosition() {
        return DRONE_LAUNCH_POSITION;
    }

    public int getLevelPosition() {
        return LEVEL_POSITION;
    }

    public int getDrivingPosition() {
        return DRIVING_POSITION;
    }

    /** @return the minimum position of the elbow */
    public int getIntakePosition() {
        return INTAKE_POSITION;
    }
}
