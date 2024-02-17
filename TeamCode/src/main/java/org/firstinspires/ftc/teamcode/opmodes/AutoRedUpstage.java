package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutonomousController;

@Autonomous(name = "RedUpstage", group = "Auto")
public class AutoRedUpstage extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousController autonomousController = new AutonomousController(this, false, true, true);

        telemetry.addLine("This autonomous mode is used for when we start on the red alliance, closer from the backdrop.");
        telemetry.addLine("In autonomous, this mode drives forward, places a purple pixel on the correct spike mark, drives to the backdrop, and places a yellow pixel in the correct spot.");
        telemetry.addLine("To finish, it rotates to face away from the drivers for field-centric to work properly, then drives into the corner to let other robots access the backdrop");
        telemetry.update();

        waitForStart();
        autonomousController.start();

        while (opModeIsActive() && !isStopRequested()) {
            autonomousController.run();
        }
    }
}
