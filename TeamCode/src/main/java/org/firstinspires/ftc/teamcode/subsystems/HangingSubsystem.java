package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.WinchConstants.*;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A subsystem that represents the motor that controls the winch.
 *
 * @author Esquimalt Atom Smashers
 */
public class HangingSubsystem extends CustomSubsystemBase {
    private final DcMotorEx winchMotor;
    private final ServoEx hookServo;

    private enum ServoState {
        LEVEL,
        MANUAL
    }

    ServoState servoState = ServoState.LEVEL;

    /**
     * Constructs a new WinchSubsystem
     *
     * @param hardwareMap The hardware map of the robot
     * @param telemetry The telemetry of the robot
     */
    public HangingSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        winchMotor = hardwareMap.get(DcMotorEx.class, WINCH_MOTOR_NAME);
        configureMotor();

        hookServo = new SimpleServo(hardwareMap, HOOK_SERVO_NAME, MIN_ANGLE, MAX_ANGLE);
    }

    /** Configure the winch motor by setting the direction and zero power behavior */
    private void configureMotor() {
        winchMotor.setDirection(WINCH_MOTOR_DIRECTION);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Start moving the winch motor to pull ourselves up. */
    public void winch() {
        winchMotor.setPower(WINCH_SPEED);
    }

    /** Start moving the winch motor to let ourselves down. */
    public void unwinch() {
        winchMotor.setPower(UNWINCH_SPEED);
    }

    /** Stop the winch motor. */
    public void stopMotor() { winchMotor.setPower(0); }

    public void raiseServo() {
        hookServo.setPosition(UP_POSITION);
        servoState = ServoState.MANUAL;
    }

    public void lowerServo() {
        hookServo.setPosition(DOWN_POSITION);
        servoState = ServoState.LEVEL;
    }

    public void levelServo(ElbowSubsystem elbowSubsystem) {
        if (servoState == ServoState.LEVEL) {
            hookServo.setPosition(convertPosition(elbowSubsystem.getPosition()));
        }
    }

    private double convertPosition(double value) {
        // TODO: I have no idea how to convert the value, more testing needed
        telemetry.addData("Value", value);

        return 0;
//        return Range.clip(value, 0, 270);
    }

    /** Prints data from the subsystem */
    @Override
    public void printData() {

    }
}
