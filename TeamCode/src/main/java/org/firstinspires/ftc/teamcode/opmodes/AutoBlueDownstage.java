package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutonomousController;
import org.firstinspires.ftc.teamcode.auto.NewAutonomousController;

@Autonomous(name = "BlueDownstage", group = "Auto")
public class AutoBlueDownstage extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        NewAutonomousController autonomousController = new NewAutonomousController(this, true, false, false);
        waitForStart();
        autonomousController.start();
        while (opModeIsActive() && !isStopRequested()) {
            autonomousController.run();
        }
    }
}
