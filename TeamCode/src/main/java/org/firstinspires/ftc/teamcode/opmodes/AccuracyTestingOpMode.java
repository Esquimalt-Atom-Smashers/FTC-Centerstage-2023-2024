package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.StrafeCommand;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(group = "auto")
public class AccuracyTestingOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);

        waitForStart();

        new SequentialCommandGroup(
                new DriveCommand(driveSubsystem, 48),
                new WaitCommand(500),
                new DriveCommand(driveSubsystem, -24),
                new WaitCommand(500),
                new DriveCommand(driveSubsystem, 24),
                new WaitCommand(500),
                new DriveCommand(driveSubsystem, -24),
                new WaitCommand(500)
        ).schedule();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
        }
        telemetry.addLine("The robot should be one tile forward from where it started");
        telemetry.update();
    }
}
