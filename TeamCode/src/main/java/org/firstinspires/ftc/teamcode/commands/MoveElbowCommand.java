package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

public class MoveElbowCommand extends CommandBase {
    private final ElbowSubsystem elbowSubsystem;
    private final double target;

    public MoveElbowCommand(ElbowSubsystem subsystem, double armPosition) {
        elbowSubsystem = subsystem;
        target = armPosition;
        addRequirements(elbowSubsystem);
    }

    @Override
    public void initialize() {
        elbowSubsystem.setTarget(target);
    }

    @Override
    public void execute() {
        elbowSubsystem.runPID();
    }

    @Override
    public void end(boolean interrupted) {
        elbowSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return elbowSubsystem.isAtTarget();
    }
}
