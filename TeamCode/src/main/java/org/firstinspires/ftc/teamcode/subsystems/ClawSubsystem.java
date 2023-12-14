package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static org.firstinspires.ftc.teamcode.Constants.ClawConstants.*;

/**
 * A subsystem that represents the claw on the end of the arm. Uses a servo to pick up and drop pixels.
 *
 * @author Esquimalt Atom Smashers
 */
public class ClawSubsystem extends CustomSubsystemBase {
    private final ServoEx clawServo;

    /**
     * Constructs a new ClawSubsystem.
     *
     * @param hardwareMap The hardware map of the robot
     * @param telemetry The telemetry of the robot
     */
    public ClawSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

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

    /** Prints data from the subsystem. */
    public void printData() {
        telemetry.addLine("--- Claw ---");
        telemetry.addData("Angle", clawServo.getAngle());
        telemetry.addData("Angle (degrees)", clawServo.getAngle(AngleUnit.DEGREES));
        telemetry.addData("Position", clawServo.getPosition());
    }
}