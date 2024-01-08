package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A subsystem that represents the servo and motor that control the intake.
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
