package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.ArmConstants.*;

public class ClawSubsystem {

    private final Servo clawServo;

    public ClawSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        clawServo = hardwareMap.servo.get(CLAW1_SERVO_MOTOR_NAME);
    }

    public void openClaw() {
        clawServo.setPosition(OPEN_POSITION);
    }

    public void closeClaw() {
        clawServo.setPosition(CLOSE_POSITION);
    }

    public boolean clawOpen() {
        return clawServo.getPosition() == OPEN_POSITION;
    }

    public void toggleClaw() {
        if (clawOpen()) {
            closeClaw();
        } else {
            openClaw();
        }
    }

    public Servo getClawServo()
    {
        return clawServo;
    }
}