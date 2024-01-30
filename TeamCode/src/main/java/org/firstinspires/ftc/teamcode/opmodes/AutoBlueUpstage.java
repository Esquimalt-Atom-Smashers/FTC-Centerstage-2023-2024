package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.NewAutonomousController;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Autonomous(name = "BlueUpstage", group = "Auto")
public class AutoBlueUpstage extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        NewAutonomousController autonomousController = new NewAutonomousController(this, true, true, true);

        waitForStart();
        autonomousController.start();

        while (opModeIsActive() && !isStopRequested()) {
            autonomousController.run();
        }
    }
}
