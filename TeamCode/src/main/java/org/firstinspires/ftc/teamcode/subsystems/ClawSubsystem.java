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
     * Creates a new ClawSubsystem. Initializes the {@link ServoEx} using the provided {@link HardwareMap}.
     *
     * @param hardwareMap The hardware map of the robot
     */
    public ClawSubsystem(HardwareMap hardwareMap) {
        clawServo = new SimpleServo(hardwareMap, CLAW_SERVO_NAME, MIN_ANGLE, MAX_ANGLE);
    }

    /** Turns the claw to a position which opens it. */
    public void openClaw() {
        clawServo.turnToAngle(OPEN_POSITION);
    }

    /** Turns the claw to a position which closes it. */
    public void closeClaw() {
        clawServo.turnToAngle(CLOSE_POSITION);
    }

    /** Turns the claw to a tighter angle for holding a single pixel. */
    public void closeClawSingle() {
        clawServo.turnToAngle(SUPER_CLOSE_POSITION);
    }

    /**
     * Prints data to the provided {@link Telemetry}.
     *
     * @param telemetry The telemetry to print data to.
     */
    public void printPosition(Telemetry telemetry) {
        telemetry.addLine("--- Claw ---");
        telemetry.addData("Angle", clawServo.getAngle());
        telemetry.addData("Angle (degrees)", clawServo.getAngle(AngleUnit.DEGREES));
        telemetry.addData("Position", clawServo.getPosition());
    }
}