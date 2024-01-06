package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.MoveElbowCommand;
import org.firstinspires.ftc.teamcode.commands.MoveSlideCommand;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

@TeleOp(name = "Testing PID", group = "Testing")
public class PIDTestingOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this, true, false, false);

        waitForStart();
        Trigger moveTrigger = new Trigger(() -> gamepad1.dpad_up);
        moveTrigger.whenActive(new SequentialCommandGroup(
                new MoveElbowCommand(robot.getElbowSubsystem(), robot.getElbowSubsystem().getLowScoringPosition()),
                new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getLowScoringPosition())
        ));

        Trigger secondTrigger = new Trigger(() -> gamepad1.dpad_down);
        secondTrigger.whenActive(new SequentialCommandGroup(
                new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getInPosition()),
                new MoveElbowCommand(robot.getElbowSubsystem(), robot.getElbowSubsystem().getDrivingPosition())
        ));

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.a) {
                robot.getElbowSubsystem().setTarget(ElbowSubsystem.target, 5.0);
                robot.getElbowSubsystem().runPID();
            }
            else robot.getElbowSubsystem().stopMotor();

            if (gamepad1.b) {
                telemetry.addLine("Moving slide");
                robot.getLinearSlideSubsystem().setTarget(LinearSlideSubsystem.target, 5.0);
                robot.getLinearSlideSubsystem().runPID();
            }
            else robot.getLinearSlideSubsystem().stopMotor();

            CommandScheduler.getInstance().run();

            robot.getElbowSubsystem().printData();
            telemetry.addData("Elbow target", ElbowSubsystem.target);
            robot.getLinearSlideSubsystem().printData();
            telemetry.addData("Slide target", LinearSlideSubsystem.target);
            telemetry.update();
        }
    }
}
