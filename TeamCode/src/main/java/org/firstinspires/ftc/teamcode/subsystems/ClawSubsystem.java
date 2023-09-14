package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.ClawConstants.*;

public class ClawSubsystem {
    private final Servo claw;
    private final Telemetry telemetry;

    public ClawSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        claw = hardwareMap.get(Servo.class, CLAW_SERVO_NAME);

        claw.setDirection(CLAW_SERVO_DIRECTION);

        this.telemetry = telemetry;
    }

    public void close() {
        claw.setPosition(CLOSE_POSITION);
    }

    public void open() {
        claw.setPosition(OPEN_POSITION);
    }
}
