package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.PixelConstants.*;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PixelSubsystem {
    // One ServoEx

    // Method to rotate to a set position
    // Method to rotate back to start position

    private final ServoEx pixelServo;

    public PixelSubsystem(HardwareMap hardwareMap) {
        // Initialize the servo
        pixelServo = new SimpleServo(hardwareMap, PIXEL_SERVO_NAME, MIN_ANGLE, MAX_ANGLE);
    }

    public void placePixel() {
        pixelServo.rotateBy(PLACE_POSITION);
    }

    public void startPosition() {
        pixelServo.setPosition(START_POSITION);
    }
}
