package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.DroneConstants.*;

/**
 * A subsystem that represents the servo on the arm that releases the paper drone.
 */
public class DroneSubsystem extends SubsystemBase {
    private final ServoEx droneServo;

    /**
     * Creates a new DroneSubsystem. Initializes the {@link ServoEx} using the provided {@link HardwareMap}.
     * @param hardwareMap The hardware map of the robot
     */
    public DroneSubsystem(HardwareMap hardwareMap) {
        droneServo = new SimpleServo(hardwareMap, DRONE_SERVO_NAME, MIN_ANGLE, MAX_ANGLE);
    }

    /** Turns the servo to release the drone. */
    public void release() {
        droneServo.turnToAngle(RELEASE_ANGLE);
    }

    /** Turns the servo to the starting position. */
    public void startPosition() {
        droneServo.turnToAngle(START_POSITION);
    }
}
