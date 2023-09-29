package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.LinearSlideConstants.*;

import java.util.Arrays;

public class LinearSlideSubsystem {
    private final DcMotorEx slideMotor;

    private double[] positions = new double[]{IN_POSITION, TEST_POSITION, OUT_POSITION};

    private PIDController controller;

    public static double target;
    private boolean atTarget;

    private final Telemetry telemetry;


    public LinearSlideSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        slideMotor = hardwareMap.get(DcMotorEx.class, SLIDE_MOTOR_NAME);

        slideMotor.setDirection(SLIDE_MOTOR_DIRECTION);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        controller = new PIDController(P, I, D);
        this.telemetry = telemetry;
    }

    public void extend() {
        target = OUT_POSITION;
    }

    public void retract() {
        target = IN_POSITION;
    }

    public void testPosition() {
        target = TEST_POSITION;
    }

    public void extendManually() {
        slideMotor.setPower(EXTEND_POWER);
    }

    public void retractManually() {
        slideMotor.setPower(RETRACT_POWER);
    }

    public void stop() {
        slideMotor.setPower(0);
    }

    public boolean isMaxExtension() {
        return slideMotor.getCurrentPosition() >= MAX_POSITION;
    }

    public boolean isMaxRetraction() {
        return slideMotor.getCurrentPosition() <= MIN_POSITION;
    }

    public void runPID() {
        controller.setPID(P, I, D);
        int slidePosition = slideMotor.getCurrentPosition();
        double power = controller.calculate(slidePosition, target);
        slideMotor.setPower(power);
        atTarget = power <= POWER_TOLERANCE;
    }

    public void nextPosition() {
//        if (position == TargetPosition.HIGH) return;
//        // Set our position to the next position
//        setPosition(positions[Arrays.asList(positions).indexOf(position) + 1]);
        if (target == positions[positions.length - 1]) return;
        target = positions[Arrays.asList(positions).indexOf(target) + 1];
    }

    public void prevPosition() {
//        if (position == TargetPosition.DEFAULT) return;
//        // Set our position to the previous position
//        setPosition(positions[Arrays.asList(positions).indexOf(position) - 1]);
        if (target == positions[0]) return;
        target = positions[Arrays.asList(positions).indexOf(target) - 1];
    }

    // TODO: Make a isAtTarget method
    public boolean isAtTarget() {
        return atTarget;
    }
}
