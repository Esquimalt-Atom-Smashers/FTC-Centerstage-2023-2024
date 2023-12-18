package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

public class MoveSlideCommand extends CommandBase {
    private final LinearSlideSubsystem linearSlideSubsystem;
    private final double target;

    public MoveSlideCommand(LinearSlideSubsystem subsystem, double slidePosition) {
        linearSlideSubsystem = subsystem;
        target = slidePosition;
        addRequirements(linearSlideSubsystem);
    }

    @Override
    public void initialize() {
        if (target == -1) {
            cancel();
            return;
        }
        linearSlideSubsystem.setTarget(target, 5.0);
    }

    @Override
    public void execute() {
        linearSlideSubsystem.runPID();
    }

    @Override
    public void end(boolean interrupted) {
        linearSlideSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return linearSlideSubsystem.isAtTarget();
    }
}
