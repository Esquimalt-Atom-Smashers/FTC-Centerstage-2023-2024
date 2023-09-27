package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

    public DroneSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize the servo
        droneServo = new SimpleServo(hardwareMap, DRONE_SERVO_NAME, MIN_POSITION, MAX_POSITION);
    }

    public void rotateServoForward()
    {
        droneServo.turnToAngle(FORWARD_DEGREE);
    }

    public void stopServo()
    {
        droneServo.turnToAngle(STOP_DEGREE);
    }
    public void rotateServoBackward()
    {
        droneServo.turnToAngle(BACKWARD_DEGREE);
    }
}
