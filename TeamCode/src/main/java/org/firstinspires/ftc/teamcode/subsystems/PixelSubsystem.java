package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.ClawConstants.CLAW_SERVO_NAME;
import static org.firstinspires.ftc.teamcode.Constants.ClawConstants.MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ClawConstants.MIN_POSITION;

import static org.firstinspires.ftc.teamcode.Constants.PixelConstants.*;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PixelSubsystem {
    // One ServoEx

    // Method to rotate to a set position
    // Method to rotate back to start position

    private final ServoEx pixelServo;

    public PixelSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize the servo
        pixelServo = new SimpleServo(hardwareMap, CLAW_SERVO_NAME, MIN_POSITION, MAX_POSITION);
    }

    public void rotateToPosition()
    {
        pixelServo.rotateBy(ROTATE_BY_POSITION);
    }

    public void rotateToStartPosition()
    {
        pixelServo.setPosition(START_POSITION);
    }
}
