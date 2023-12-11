package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.DroneConstants.*;

/**
 * A subsystem that represents the servo on the arm that releases the paper drone.
 *
 * @author Esquimalt Atom Smashers
 */
public class DroneSubsystem extends SubsystemBase {
    private final ServoEx droneServo;

    /**
     * Constructs a DroneSubsystem.
     *
     * @param hardwareMap The global hardwareMap.
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
