package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.DroneConstants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A subsystem that represents the servo on the arm. Uses the servo to hold and release
 * an elastic to shoot the drone.
 *
 * @author Esquimalt Atom Smashers
 */
public class DroneSubsystem extends CustomSubsystemBase {
    private final ServoEx droneServo;

    /**
     * Constructs a DroneSubsystem.
     *
     * @param hardwareMap The hardware map of the robot
     * @param telemetry The telemetry of the robot
     */
    public DroneSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

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

    /** Prints data from the subsystem */
    @Override
    public void printData() {
        telemetry.addLine("--- Drone ---");
    }
}
