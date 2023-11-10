package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static org.firstinspires.ftc.teamcode.Constants.ClawConstants.*;

/**
 * A subsystem representing the claw on the end of the arm. Contains a {@link ServoEx} used to hold pixels.
 */
public class ClawSubsystem extends SubsystemBase {
    private final ServoEx clawServo;

    /**
     * Initializes the {@link ServoEx} using the provided {@link HardwareMap}.
     * @param hardwareMap The hardware map of the robot
     */
    public ClawSubsystem(HardwareMap hardwareMap) {
        clawServo = new SimpleServo(hardwareMap, CLAW_SERVO_NAME, MIN_ANGLE, MAX_ANGLE);
    }

    /**
     * Opens the claw.
     */
    public void openClaw() {
        clawServo.turnToAngle(OPEN_POSITION);
    }

    /**
     * Closes the claw.
     */
    public void closeClaw() {
        clawServo.turnToAngle(CLOSE_POSITION);
    }

    /**
     * Closes the claw tighter than normal, for holding a single pixel.
     */
    public void closeClawSingle() {
        clawServo.turnToAngle(SUPER_CLOSE_POSITION);
    }

    /**
     * Check if the claw is open.
     * @return Whether the claw is open or not
     */
    public boolean isClawOpen() {
        return clawServo.getPosition() == OPEN_POSITION;
    }

    /**
     * Prints data to the provided {@link Telemetry}.
     * @param telemetry The telemetry to print data to.
     */
    public void printPosition(Telemetry telemetry) {
        telemetry.addLine("--- Claw ---");
        telemetry.addData("Angle", clawServo.getAngle());
        telemetry.addData("Angle (degrees)", clawServo.getAngle(AngleUnit.DEGREES));
        telemetry.addData("Position", clawServo.getPosition());
    }
}