package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutonomousController;

@Autonomous(name = "BlueDownstage", group = "Auto")
public class AutoBlueDownstage extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousController autonomousController = new AutonomousController(this, true, false, true);

        telemetry.addLine("This autonomous mode is used for when we start on the blue alliance, farther from the backdrop.");
        telemetry.addLine("In autonomous, this mode drives forward, places a purple pixel on the correct spike mark, drives all the way to the backdrop, and places a yellow pixel in the correct spot.");
        telemetry.addLine("To finish, it rotates to face away from the drivers for field-centric to work properly.");
        telemetry.update();

        waitForStart();
        autonomousController.start();

        while (opModeIsActive() && !isStopRequested()) {
            autonomousController.run();
        }
    }
}
