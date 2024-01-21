package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.NewAutonomousController;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Autonomous(name = "BlueUpstage", group = "Auto")
public class AutoBlueUpstage extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();

    private enum SpikeMark {
        LEFT,
        MIDDLE,
        RIGHT
    }


    // TODO: Make this beautiful!
    @Override
    public void runOpMode() throws InterruptedException {
        NewAutonomousController autonomousController = new NewAutonomousController(this, true, true, true);

        waitForStart();
        autonomousController.start();

//        SpikeMark spikeMarkPosition;
//        intakeSubsystem.upPosition();
//        driveSubsystem.driveByDistanceAsync(30);
//        waitForSeconds(1);
//        if (distanceSensorSubsystem.isLeftBlocked()) {
//            spikeMarkPosition = SpikeMark.LEFT;
//            driveSubsystem.turn(90);
//            waitForSeconds(0.25);
//            driveSubsystem.strafeByDistance(5);
//            waitForSeconds(0.25);
//            driveSubsystem.driveByDistanceAsync(-2);
//            waitForSeconds(0.25);
//        } else if (distanceSensorSubsystem.isRightBlocked()) {
//            spikeMarkPosition = SpikeMark.RIGHT;
//            driveSubsystem.driveByDistanceAsync(-4);
//            waitForSeconds(0.25);
//            driveSubsystem.turn(-90);
//            waitForSeconds(0.25);
//            driveSubsystem.driveByDistanceAsync(-2);
//            waitForSeconds(0.25);
//        } else {
//            spikeMarkPosition = SpikeMark.MIDDLE;
//            driveSubsystem.driveByDistanceAsync(-7.5);
//            waitForSeconds(0.25);
//        }
//
//        intakeSubsystem.downPosition();
//        waitForSeconds(0.5);
//        intakeSubsystem.intake(0.5);
//        waitForSeconds(0.25);
//        intakeSubsystem.stopMotor();
//        waitForSeconds(0.25);
//        intakeSubsystem.upPosition();
//
//         TODO: Get the rest of these movement to the backdrop
//        if (spikeMarkPosition == SpikeMark.RIGHT) {
//            driveSubsystem.driveByDistanceAsync(-4);
//            waitForSeconds(0.25);
//            driveSubsystem.turn(180);
//            waitForSeconds(0.25);
//            driveSubsystem.driveByDistanceAsync(22);
//            waitForSeconds(0.25);
//        }
//
//         TODO: Get the arm able to move
//
//         TODO: Get the movement to the corner
//


        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Spike mark", autonomousController.getSpikeMark());
            autonomousController.run();
        }
    }
}
