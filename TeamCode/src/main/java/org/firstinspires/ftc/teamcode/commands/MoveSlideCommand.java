package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

public class MoveSlideCommand extends CommandBase {
    private final LinearSlideSubsystem linearSlideSubsystem;
    private final double target;
    private final Telemetry telemetry;
    private ElapsedTime timer;

    public MoveSlideCommand(LinearSlideSubsystem subsystem, double slidePosition, Telemetry telemetry) {
        this.telemetry = telemetry;
        linearSlideSubsystem = subsystem;
        target = slidePosition;
        addRequirements(linearSlideSubsystem);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        linearSlideSubsystem.setTarget(target);
    }

    @Override
    public void execute() {
        telemetry.addData("LinearSlideCommand", timer.milliseconds());
        linearSlideSubsystem.runPID();
    }

    @Override
    public boolean isFinished() {
//        return timer.milliseconds() >= 5000;
        return linearSlideSubsystem.isAtTarget();
    }
}
