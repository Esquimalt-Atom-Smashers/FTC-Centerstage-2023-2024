package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.LED;

import static org.firstinspires.ftc.teamcode.Constants.BoxConstants.*;

/**
 * A subsystem that represents the box release door on the end of the arm.
 * Uses a servo to open and close the door and LEDs to indicate when open/closed.
 *
 * @author Esquimalt Atom Smashers
 */
public class BoxReleaseSubsystem extends CustomSubsystemBase {
    private final ServoEx boxReleaseServo;

    // Left and right side of robot LED channels for red and green LEDs
    private final LED redRightLED;
    private final LED greenRightLED;
    private final LED redLeftLED;
    private final LED greenLeftLED;

    //public boolean boxOpen = false;

    /**
     * Constructs a new boxReleaseSubsystem.
     *
     * @param hardwareMap The hardware map of the robot
     * @param telemetry The telemetry of the robot
     */
    public BoxReleaseSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        boxReleaseServo = new SimpleServo(hardwareMap, BOX_SERVO_NAME, MIN_ANGLE, MAX_ANGLE);

        redLeftLED = hardwareMap.led.get("redLeft");
        greenLeftLED = hardwareMap.led.get("greenLeft");
        redRightLED = hardwareMap.led.get("redRight");
        greenRightLED = hardwareMap.led.get("greenRight");

    }

    /** Turns the servo for the box release to a position which opens it. */
    public void openBox() {
        boxReleaseServo.turnToAngle(OPEN_POSITION);
        //boxOpen = true;

        // When open, LEDs are red as warning to driver/operator
        greenLeftLED.enable(false);
        redLeftLED.enable(true);
        greenRightLED.enable(false);
        redRightLED.enable(true);
    }

    /** Turns the box release to a position which closes it. */
    public void closeBox() {
        boxReleaseServo.turnToAngle(CLOSE_POSITION);
        //boxOpen = false;

        //When closed, LEDs are green to indicate normal operation
        greenLeftLED.enable(true);
        redLeftLED.enable(false);
        greenRightLED.enable(true);
        redRightLED.enable(false);
    }

    /** Prints data from the subsystem. */
    public void printData() {
        telemetry.addLine("--- Box Release ---");
        telemetry.addData("Angle", boxReleaseServo.getAngle());
        telemetry.addData("Angle (degrees)", boxReleaseServo.getAngle(AngleUnit.DEGREES));
        telemetry.addData("Position", boxReleaseServo.getPosition());
    }
}