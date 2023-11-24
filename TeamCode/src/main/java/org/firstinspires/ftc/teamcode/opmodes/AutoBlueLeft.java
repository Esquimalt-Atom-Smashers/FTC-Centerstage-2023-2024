package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutonomousController;

@Autonomous(group = "auto")
public class AutoBlueLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousController autonomousController = new AutonomousController(this);
        waitForStart();
        if (isStopRequested()) return;
        autonomousController.blueLeft();

//        AutonomousController autonomousController = new AutonomousController(this);
//        autonomousController.setSettings(true, true, true);
//        autonomousController.initializeCommands();
//        waitForStart();
//        autonomousController.startStateMachine();
//        while (opModeIsActive() && !isStopRequested()) {
//            autonomousController.runStateMachine();
//        }
    }
}
