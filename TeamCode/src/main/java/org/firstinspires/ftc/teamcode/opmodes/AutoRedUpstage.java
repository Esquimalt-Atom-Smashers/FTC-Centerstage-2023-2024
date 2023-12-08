package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutonomousController;

@Autonomous(name = "RedUpstage", group = "Auto")
public class AutoRedUpstage extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousController autonomousController = new AutonomousController(this, false, true, true);
        waitForStart();
        autonomousController.start();
        while (opModeIsActive() && !isStopRequested()) {
            autonomousController.run();
        }
    }
}
