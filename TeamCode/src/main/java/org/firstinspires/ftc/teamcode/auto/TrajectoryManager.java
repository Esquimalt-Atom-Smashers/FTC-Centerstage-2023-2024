package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class TrajectoryManager {
    private final AutonomousController autonomousController;
    private final SampleMecanumDrive drive;
    private final AutonomousController.SpikeMark spikeMarkPosition;

    public TrajectoryManager(AutonomousController autonomousController) {
        this.autonomousController = autonomousController;
        this.drive = autonomousController.getDriveBase();
        this.spikeMarkPosition = autonomousController.getSpikeMarkPosition();
    }

    /**
     * Constructs and returns the trajectory sequence used to drive to the correct spike mark.
     *
     * @return The built trajectory sequence.
     */
    public TrajectorySequence getDriveToSpikeMarksTrajectory() {
        if (spikeMarkPosition == AutonomousController.SpikeMark.LEFT)
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(24)
                    .turn(Math.toRadians(90))
                    .strafeRight(5)
                    .back(2)
                    .build();
        else if (spikeMarkPosition == AutonomousController.SpikeMark.MIDDLE)
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(25)
                    .strafeLeft(8)
                    .build();
        else
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(24)
                    .turn(Math.toRadians(-90))
                    .strafeLeft(5)
                    .back(2)
                    .build();
    }

    /**
     * Constructs and returns the trajectory sequence used to drive away from the spike marks after placing the purple pixel.
     *
     * @return The built trajectory sequence.
     */
    public TrajectorySequence getDriveFromSpikeMarksTrajectory() {
        if (spikeMarkPosition == AutonomousController.SpikeMark.LEFT)
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeLeft(30)
                    .build();
        else if (spikeMarkPosition == AutonomousController.SpikeMark.MIDDLE)
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeRight(6)
                    .back(11)
                    .build();
        else
            return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeRight(30)
                    .build();
    }

    /**
     * Constructs and returns the trajectory sequence used to drive to the backdrop.
     *
     * @return The built trajectory sequence.
     */
    public TrajectorySequence getDriveToBackdropTrajectory() {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(46, 35 * autonomousController.getPositionMultiplier(), Math.toRadians(0)))
                .strafeRight(spikeMarkPosition == AutonomousController.SpikeMark.LEFT ? -6 : spikeMarkPosition == AutonomousController.SpikeMark.MIDDLE ? 0.1 : 9)
                .build();
    }

    /**
     * Constructs and returns the trajectory sequence used to drive to the corner.
     *
     * @return The built trajectory sequence.
     */
    public TrajectorySequence getDriveToCornerTrajectory() {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(3)
                .turn(Math.toRadians(autonomousController.isBlueAlliance() ? -90 : 90))
                .lineTo(new Vector2d(49, 60 * autonomousController.getPositionMultiplier()))
                .build();
    }
}
