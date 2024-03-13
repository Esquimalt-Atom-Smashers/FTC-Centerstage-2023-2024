package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;

/**
 * Command that moves the elbow to a set position using the PID controller.
 */
public class MoveElbowCommand extends CommandBase {
    private final ElbowSubsystem elbowSubsystem;
    private final double target;

    /**
     * Creates a new command that moves the elbow to a specified position using the PID controller.
     *
     * @param elbowSubsystem Reference to the elbowSubsystem
     * @param armPosition The target position of the arm in pulses
     */
    public MoveElbowCommand(ElbowSubsystem elbowSubsystem, double armPosition) {
        this.elbowSubsystem = elbowSubsystem;
        target = armPosition;
        addRequirements(this.elbowSubsystem);
    }

    @Override
    public void initialize() {
        if (target == -1) {
            cancel();
            return;
        }
        elbowSubsystem.setTarget(target, 5.0);
    }

    @Override
    public void execute() {
        elbowSubsystem.runPID();
    }

    @Override
    public void end(boolean interrupted) {
        elbowSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return elbowSubsystem.isAtTarget();
    }
}
