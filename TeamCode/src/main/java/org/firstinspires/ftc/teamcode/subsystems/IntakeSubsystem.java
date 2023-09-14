package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.*;

public class IntakeSubsystem {
    private final DcMotorEx intakeMotor;
    private final Telemetry telemetry;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR_NAME);

        intakeMotor.setDirection(INTAKE_MOTOR_DIRECTION);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.telemetry = telemetry;
    }

    public void intake() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void outtake() {
        intakeMotor.setPower(OUTTAKE_POWER);
    }
}
