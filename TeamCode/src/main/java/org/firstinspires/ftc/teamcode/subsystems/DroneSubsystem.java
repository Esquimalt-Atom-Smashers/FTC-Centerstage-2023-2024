package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.DroneConstants.*;

public class DroneSubsystem {
    // One ServoEx
    // Method to rotate the servo
    // Method to stop the servo
    // 0 -> 100% back
    // .25 -> 50% back
    // .5 -> 0% power
    // 0.75 -> 50% forward
    // 1 -> 100% forward
    // Program the servo to continuous mode (use the servo programmer)

    private final ServoEx droneServo;

    public DroneSubsystem(HardwareMap hardwareMap) {
        // Initialize the servo
        droneServo = new SimpleServo(hardwareMap, DRONE_SERVO_NAME, MIN_ANGLE, MAX_ANGLE);
    }

    public void forward() {
        droneServo.turnToAngle(FORWARD);
    }

    public void backward() {
        droneServo.turnToAngle(BACKWARD);
    }

    public void stop() {
        droneServo.turnToAngle(STOP);
    }
}
