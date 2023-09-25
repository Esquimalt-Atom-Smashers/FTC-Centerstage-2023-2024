package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.WristConstants.*;

public class WristSubsystem
{

    // One servo
    // Method to rotate to 0 degrees
    // Method to rotate to 60 degrees

    private Servo wristServo;

    public WristSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        wristServo = hardwareMap.servo.get(WRIST_SERVO_NAME);
    }
    public void upWrist()
    {
        wristServo.setPosition(WRIST_UP_POSITION);
    }
    public void downWrist()
    {
        wristServo.setPosition(WRIST_DOWN_POSITION);
    }
    public Servo getWristServo()
    {
        return wristServo;
    }

}