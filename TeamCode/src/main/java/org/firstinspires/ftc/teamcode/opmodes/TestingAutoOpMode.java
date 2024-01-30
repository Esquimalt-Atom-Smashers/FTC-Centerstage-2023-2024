package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.StrafeCommand;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(group = "auto")
public class TestingAutoOpMode extends LinearOpMode {

    double waitTime = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);

        waitForStart();

        new SequentialCommandGroup(
                new DriveCommand(driveSubsystem, 10),
                new WaitCommand(250),
                new StrafeCommand(driveSubsystem, -10),
                new WaitCommand(250),
//                new TurnCommand(driveSubsystem, 90),
//                new WaitCommand(250),
                new DriveCommand(driveSubsystem, -10),
                new WaitCommand(250),
                new StrafeCommand(driveSubsystem, 10),
                new WaitCommand(250)
//                new TurnCommand(driveSubsystem, -90)
        ).schedule();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
        }
    }

}
