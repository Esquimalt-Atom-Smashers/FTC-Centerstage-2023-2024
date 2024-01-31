package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.MoveElbowCommand;
import org.firstinspires.ftc.teamcode.commands.MoveSlideCommand;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

@Config
@TeleOp(name = "Testing PID", group = "Testing")
public class PIDTestingOpMode extends LinearOpMode {
    public static int slideTarget = 0;
    public static int elbowTarget = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(this, true, false);

        Gamepad gamepad = gamepad2;


        waitForStart();

        Trigger moveTrigger = new Trigger(() -> gamepad.y);
//        moveTrigger.whenActive(new SequentialCommandGroup(
////                new MoveElbowCommand(robot.getElbowSubsystem(), robot.getElbowSubsystem().getLowScoringPosition()),
//                new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getLowScoringPosition())
//        ));
        moveTrigger.whenActive(new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getLowScoringPosition()));

        Trigger secondTrigger = new Trigger(() -> gamepad.x);
//        secondTrigger.whenActive(new SequentialCommandGroup(
//                new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getInPosition())
////                new MoveElbowCommand(robot.getElbowSubsystem(), robot.getElbowSubsystem().getDrivingPosition())
//        ));
        secondTrigger.whenActive(new MoveSlideCommand(robot.getLinearSlideSubsystem(), robot.getLinearSlideSubsystem().getInPosition()));

        robot.getElbowSubsystem().setTarget(robot.getElbowSubsystem().getPosition(), 5.0);
        robot.getLinearSlideSubsystem().setTarget(robot.getLinearSlideSubsystem().getPosition(), 5.0);
        CommandScheduler.getInstance().cancelAll();

        boolean aPressed = false;
        boolean bPressed = false;

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad.a) {
                if (!aPressed) robot.getElbowSubsystem().setTarget(elbowTarget, 5.0);
                robot.getElbowSubsystem().runPID();
            }
            else robot.getElbowSubsystem().stopMotor();
            aPressed = gamepad.a;

            if (gamepad.b) {
                if (!bPressed) robot.getLinearSlideSubsystem().setTarget(slideTarget, 5.0);
                robot.getLinearSlideSubsystem().runPID();
            }
            else robot.getLinearSlideSubsystem().stopMotor();
            bPressed = gamepad.b;

            CommandScheduler.getInstance().run();

            robot.getElbowSubsystem().printData();
//            telemetry.addData("Elbow target", ElbowSubsystem.target);
            robot.getLinearSlideSubsystem().printData();
//            telemetry.addData("Slide target", LinearSlideSubsystem.target);
            telemetry.update();
        }
    }
}
