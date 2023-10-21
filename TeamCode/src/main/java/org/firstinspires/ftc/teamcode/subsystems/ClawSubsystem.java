package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static org.firstinspires.ftc.teamcode.Constants.ClawConstants.*;

public class ClawSubsystem extends SubsystemBase {

    private final ServoEx clawServo;

    public ClawSubsystem(HardwareMap hardwareMap) {
        // Initialize the servo
        clawServo = new SimpleServo(hardwareMap, CLAW_SERVO_NAME, MIN_ANGLE, MAX_ANGLE);
    }

    // Set the claw to the open position
    public void openClaw() {
        clawServo.turnToAngle(OPEN_POSITION);
    }

    // Set the claw to the closed position
    public void closeClaw() {
        clawServo.turnToAngle(CLOSE_POSITION);
    }

    // Check if the claw is open
    public boolean isClawOpen() {
        return clawServo.getPosition() == OPEN_POSITION;
    }

    public void toggleClaw() {
        if (isClawOpen()) {
            closeClaw();
        } else {
            openClaw();
        }
    }

    public void printPosition(Telemetry telemetry) {
        telemetry.addData("Claw servo angle", clawServo.getAngle());
        telemetry.addData("Claw position", clawServo.getPosition());
        telemetry.addData("Angle in degrees", clawServo.getAngle(AngleUnit.DEGREES));
    }

    public ServoEx getClawServo()
    {
        return clawServo;
    }
}