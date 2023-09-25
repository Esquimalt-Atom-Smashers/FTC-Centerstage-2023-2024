package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.ElbowConstants.*;

@Config
public class ElbowSubsystem {
    private final DcMotorEx elbowMotor;

    private PIDController controller;

    public static double target;

    private final Telemetry telemetry;

    public  ElbowSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        elbowMotor = hardwareMap.get(DcMotorEx.class, ELBOW_DC_MOTOR_NAME);
        elbowMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        controller = new PIDController(P, I, D);

        this.telemetry = telemetry;
    }

    public void spin() {
        elbowMotor.setPower(0.8);
    }

    public void counterSpin() {
        elbowMotor.setPower(-0.8);
    }

    public void stop() {
        elbowMotor.setPower(0);
    }

    public void run() {
        controller.setPID(P, I, D);
        int elbowPosition = elbowMotor.getCurrentPosition();
        double power = controller.calculate(elbowPosition, target);
        elbowMotor.setPower(power);
        telemetry.addData("pos ", elbowPosition);
        telemetry.addData("Target ", target);
        telemetry.update();
    }
}
