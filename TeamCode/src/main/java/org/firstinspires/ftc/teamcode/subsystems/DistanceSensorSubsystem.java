package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.Constants.DistanceSensorConstants.*;

/**
 * A subsystem that represents the two distance sensors on the sides of the robot.
 *
 * @author Esquimalt Atom Smashers
 */
public class DistanceSensorSubsystem extends CustomSubsystemBase {
    private final DistanceSensor leftDistanceSensor;
    private final DistanceSensor rightDistanceSensor;

    /**
     * Constructs a new DistanceSensorSubsystem.
     *
     * @param hardwareMap The hardware map of the robot
     * @param telemetry The telemetry of the robot
     */
    public DistanceSensorSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, LEFT_DISTANCE_SENSOR_NAME);
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, LEFT_DISTANCE_SENSOR_NAME);
    }

    /** @return If the left sensor is blocked by something. */
    public boolean isLeftBlocked() {
        return leftDistanceSensor.getDistance(DistanceUnit.INCH) <= DISTANCE_THRESHOLD;
    }

    /** @return If the right sensor is blocked by something. */
    public boolean isRightBlocked() {
        return rightDistanceSensor.getDistance(DistanceUnit.INCH) <= DISTANCE_THRESHOLD;
    }

    /** Prints data from the distance sensors. */
    public void printData() {
        telemetry.addData("Right blocked", isRightBlocked());
        telemetry.addData("Left blocked", isLeftBlocked());
    }
}
