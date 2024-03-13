package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.MoveCommand;
import org.firstinspires.ftc.teamcode.commands.StrafeCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(group = "auto")
public class TestingAutoOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);

        telemetry.addLine("This op mode is used to test the autonomous driving.");
        telemetry.addLine("When you start this, it will drive and strafe in a square.");
        telemetry.update();

        waitForStart();
        new SequentialCommandGroup(
                new MoveCommand(driveSubsystem, MoveCommand.MovementType.DRIVE, 10),
                new MoveCommand(driveSubsystem, MoveCommand.MovementType.STRAFE, -10),
                new MoveCommand(driveSubsystem, MoveCommand.MovementType.DRIVE, -10),
                new MoveCommand(driveSubsystem, MoveCommand.MovementType.STRAFE, 10)
        ).schedule();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
        }
    }

}
