package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutonomousController;
import org.firstinspires.ftc.teamcode.auto.NewAutonomousController;

@Autonomous(name = "RedDownstage", group = "Auto")
public class AutoRedDownstage extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        NewAutonomousController autonomousController = new NewAutonomousController(this, false, false, false);
        waitForStart();
        autonomousController.start();
        while (opModeIsActive() && !isStopRequested()) {
            autonomousController.run();
        }
    }
}

// :]
// -Jake
