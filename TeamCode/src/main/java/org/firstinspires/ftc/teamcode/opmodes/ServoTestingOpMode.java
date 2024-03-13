package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

@Config
@TeleOp(name="Servo Testing", group="Testing")
public class ServoTestingOpMode extends LinearOpMode {
    public static double lowerTarget = 0;
    public static double higherTarget = 15;

    @Override
    public void runOpMode() throws InterruptedException {
        ServoEx servo = new SimpleServo(hardwareMap, Constants.DroneConstants.DRONE_SERVO_NAME, 0, 270);
        GamepadEx gamepad = new GamepadEx(gamepad1);

        telemetry.addLine("This op mode is used to test a servo.");
        telemetry.addLine("While enabled, press A and B to switch between the lower target and higher target.");
        telemetry.addLine("To change the targets, you can use ftc dashboard, to change the servo you must change the servo name in code.");
        telemetry.update();


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad.getButton(GamepadKeys.Button.A))
                servo.turnToAngle(lowerTarget);
            if (gamepad.getButton(GamepadKeys.Button.B))
                servo.turnToAngle(higherTarget);
            telemetry.addData("Position", servo.getPosition());
            telemetry.update();
        }
    }
}
