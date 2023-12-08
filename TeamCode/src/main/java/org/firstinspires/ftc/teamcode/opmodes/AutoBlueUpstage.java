package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutonomousController;

@Autonomous(name = "BlueUpstage", group = "Auto")
public class AutoBlueUpstage extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousController autonomousController = new AutonomousController(this, true, true, true);
        waitForStart();
        autonomousController.start();
        while (opModeIsActive() && !isStopRequested()) {
            autonomousController.run();
        }
    }
}
