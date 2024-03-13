package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;

@TeleOp(name = "LED Testing", group = "Testing")
public class LEDTestingOpMode extends LinearOpMode {
    RevBlinkinLedDriver.BlinkinPattern currentPattern;

    @Override
    public void runOpMode() throws InterruptedException {
        currentPattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        GamepadEx gamepad = new GamepadEx(gamepad1);
        LEDSubsystem ledSubsystem = new LEDSubsystem(hardwareMap, telemetry);
        ledSubsystem.setLights(currentPattern);

        telemetry.addLine("This op mode is used for testing the LEDs");
        telemetry.addLine("While enabled, you can use the d-pad to rotate through the different light patterns.");
        telemetry.addLine("Left for previous, right for next, up for back 10, down for next 10.");
        telemetry.addLine("You can see the current pattern printed out on the telemetry.");
        telemetry.update();

        waitForStart();

        new Trigger(() -> gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)).whenActive(() -> {
            currentPattern = currentPattern.next();
            ledSubsystem.setLights(currentPattern);
        });

        new Trigger(() -> gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)).whenActive(() -> {
            currentPattern = currentPattern.previous();
            ledSubsystem.setLights(currentPattern);
        });

        new Trigger(() -> gamepad.getButton(GamepadKeys.Button.DPAD_UP)).whenActive(() -> {
            for (int i = 0; i < 10; i++) {
                currentPattern = currentPattern.previous();
            }
            ledSubsystem.setLights(currentPattern);
        });

        new Trigger(() -> gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)).whenActive(() -> {
            for (int i = 0; i < 10; i++) {
                currentPattern = currentPattern.next();
            }
            ledSubsystem.setLights(currentPattern);
        });

        while (opModeIsActive() && !isStopRequested()) {

            CommandScheduler.getInstance().run();
            telemetry.addData("Current Pattern", currentPattern);
            telemetry.update();
        }
    }
}
