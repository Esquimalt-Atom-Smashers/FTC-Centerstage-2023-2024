package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.WinchConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A subsystem that represents the motor that controls the winch.
 *
 * @author Esquimalt Atom Smashers
 */
public class WinchSubsystem extends CustomSubsystemBase {
    private final DcMotorEx winchMotor;

    /**
     * Constructs a new WinchSubsystem
     *
     * @param hardwareMap The hardware map of the robot
     * @param telemetry The telemetry of the robot
     */
    public WinchSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        winchMotor = hardwareMap.get(DcMotorEx.class, WINCH_MOTOR_NAME);
        configureMotor();
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

    /** Prints data from the subsystem */
    @Override
    public void printData() {

    }
}
