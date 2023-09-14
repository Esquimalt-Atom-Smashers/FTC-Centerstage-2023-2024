package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.WristConstants.*;

public class WristSubsystem {
    private final Servo wrist;
    private final Telemetry telemetry;

    public WristSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        wrist = hardwareMap.get(Servo.class, WRIST_SERVO_NAME);

        wrist.setDirection(WRIST_SERVO_DIRECTION);

        this.telemetry = telemetry;
    }

    public void pickup() {
        wrist.setPosition(PICKUP_POSITION);
    }

    public void dropoff() {
        wrist.setPosition(DROP_POSITION);
    }
}
