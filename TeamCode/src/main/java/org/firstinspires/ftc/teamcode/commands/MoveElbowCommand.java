package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

public class MoveElbowCommand extends CommandBase {
    private final ElbowSubsystem elbowSubsystem;
    private final double target;
    private final Telemetry telemetry;
    private ElapsedTime timer;

    public MoveElbowCommand(ElbowSubsystem subsystem, double armPosition, Telemetry telemetry) {
        this.telemetry = telemetry;
        elbowSubsystem = subsystem;
        target = armPosition;
        addRequirements(elbowSubsystem);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        elbowSubsystem.setTarget(target);
    }

    @Override
    public void execute() {
        telemetry.addData("MoveElbowCommand", timer.milliseconds());
        elbowSubsystem.runPID();
    }

    @Override
    public boolean isFinished() {
//        return timer.milliseconds() >= 5000;
        return elbowSubsystem.isAtTarget();
    }
}
