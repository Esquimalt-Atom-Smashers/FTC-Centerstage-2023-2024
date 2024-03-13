package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A subsystem that represents the servo and motor on the intake. Uses the servo to raise and
 * lower the intake and the motor to spin it.
 *
 * @author Esquimalt Atom Smashers
 */
public class IntakeSubsystem extends CustomSubsystemBase {
    private final DcMotorEx intakeMotor;
    private final ServoEx intakeServo;

    /**
     * Constructs an IntakeSubsystem.
     *
     * @param hardwareMap The hardware map of the robot
     * @param telemetry The telemetry of the robot
     */
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        intakeServo = new SimpleServo(hardwareMap, INTAKE_SERVO_NAME, MIN_ANGLE, MAX_ANGLE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);
        configureIntake();
    }

    // TODO: Swap the direction of the motor (there might be some swapping of commands in other files)
    /** Configure the intake motor by setting the direction and zero power behavior */
    private void configureIntake() {
        intakeMotor.setDirection(INTAKE_MOTOR_DIRECTION);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Sets the intake position to the 'down' position. */
    public void downPosition() {
        intakeServo.turnToAngle(INTAKE_DOWN_POSITION);
    }

    /** Sets the intake position to the 'driving' position. */
    public void mediumPosition() {
        intakeServo.turnToAngle(INTAKE_DRIVING_POSITION);
    }

    /** Sets the intake position to the 'up' position. */
    public void upPosition() {
        intakeServo.turnToAngle(INTAKE_UP_POSITION);
    }

    /** Starts intaking. */
    public void intake() {
        intakeMotor.setPower(INTAKE_SPEED);
    }

    /**
     * Intake at a set speed.
     *
     * @param speed Speed to intake at
     */
    public void intake(double speed) {
        intakeMotor.setPower(speed);
    }

    /** Start outtaking. */
    public void outtake() {
        intakeMotor.setPower(OUTTAKE_SPEED);
    }

    /** Stops the intake motor. */
    public void stopMotor() {
        intakeMotor.setPower(0);
    }

    /** Prints data from the subsystem */
    @Override
    public void printData() {
        telemetry.addLine("--- Intake ---");
        telemetry.addData("Servo pos", intakeServo.getPosition());
    }
}
