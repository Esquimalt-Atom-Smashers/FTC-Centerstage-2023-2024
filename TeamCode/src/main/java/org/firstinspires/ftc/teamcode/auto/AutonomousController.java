package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.CommandManager;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * A controller that controls the drive base and arm during autonomous.
 */
public class AutonomousController {

    enum State {
        PICKING_UP_PIXELS,
        MOVING_TO_SPIKE_MARKS,
        MOVING_FROM_SPIKE_MARKS,
        PLACING_PURPLE,
        MOVING_TO_BACKDROP,
        PLACING_YELLOW,
        HIDING,
        IDLE
    }

    enum SpikeMark {
        LEFT,
        RIGHT,
        MIDDLE
    }

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private final Robot robot;
    private final CommandManager commandManager;
    private final TrajectoryManager trajectoryManager;

    // Our starting function
    private Pose2d startPosition;

    private final SampleMecanumDrive drive;

    // Camera things
    private OpenCVPipeline pipeline;
    private OpenCvWebcam webcam;

    // Settings for controlling what we do
    private boolean isBlueAlliance;
    private boolean isUpstage;
    private boolean isPlacingYellow;

    // Operators used to easily convert positions and angles between red and blue
    private float positionMultiplier;
    private float rotationalOffset;

    // Our state for our state machine
    private State currentState = State.IDLE;

    // The position of the spike mark
    private SpikeMark spikeMarkPosition;

    private WaitCommand currentCommand;

    /**
     * Creates a new AutonomousController, initializing the subsystems using the HardwareMap from the provided LinearOpMode.
     *
     * @param opMode The LinearOpMode that created the controller.
     * @param isBlueAlliance If we are on the blue alliance.
     * @param isUpstage If we are upstage (closer to the backdrop).
     * @param isPlacingYellow Whether we want to place the yellow pixel on the backdrop.
     */
    public AutonomousController(LinearOpMode opMode, boolean isBlueAlliance, boolean isUpstage, boolean isPlacingYellow) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        setSettings(isBlueAlliance, isUpstage, isPlacingYellow);

        drive = new SampleMecanumDrive(hardwareMap);
        robot = new Robot(opMode, true, true);
        commandManager = new CommandManager(robot);
        trajectoryManager = new TrajectoryManager(this);

        startOpenCV();
    }

    /**
     * Creates a webcam and asynchronously opens an open cv pipeline. Waits until the camera is ready
     */
    private void startOpenCV(){
        updateStatus("Not Ready (Starting OpenCVPipeline)...");
        pipeline = new OpenCVPipeline();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, Constants.CameraConstants.CAMERA_NAME), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }
            @Override
            public void onError(int errorCode)
            {
            }
        });
        while (!pipeline.cameraReady) updateStatus("Not Ready (Waiting for camera)...\nDo not press stop!");
        updateStatus("Ready");
    }

    /**
     * Sets the settings for the AutonomousController, controlling where we start and if we want to place a yellow pixel on the backdrop
     *
     * @param isBlue If we are on the blue alliance.
     * @param isUpstage If we are upstage (closer to the backdrop).
     * @param isPlacingYellow Whether we want to place the yellow pixel on the backdrop.
     */
    public void setSettings(boolean isBlue, boolean isUpstage, boolean isPlacingYellow) {
        this.isBlueAlliance = isBlue;
        this.isUpstage = isUpstage;
        this.isPlacingYellow = isPlacingYellow;

        positionMultiplier = isBlueAlliance ? 1 : -1;
        rotationalOffset = isBlueAlliance ? 0 : 180;
        startPosition = new Pose2d(isUpstage ? 11.5 : -35.5, 62 * positionMultiplier, Math.toRadians(isBlue ? 270 : 90));
    }

    /**
     * Starts the state machine by starting the first command and setting the state to picking up pixels.
     */
    public void start() {
        currentState = State.PICKING_UP_PIXELS;
        drive.setPoseEstimate(startPosition);
        currentCommand = scheduleCommand(commandManager.getAutoSetupCommand());
    }

    /**
     * Runs the state machine. Switches states if needed, runs commands, updates the drive, and updating the telemetry
     */
    public void run() {
        switch (currentState) {
            // Picking up the pixel from the ground as well as waiting for the camera to be ready
            case PICKING_UP_PIXELS:
                if (canContinue() && pipeline.cameraReady) {
                    int gameElementPosition = pipeline.findGameElement(isBlueAlliance);
                    spikeMarkPosition = gameElementPosition == -1 ? SpikeMark.LEFT : gameElementPosition == 0 ? SpikeMark.MIDDLE : SpikeMark.RIGHT;
                    webcam.closeCameraDevice();
                    currentState = State.MOVING_TO_SPIKE_MARKS;
                    drive.followTrajectorySequenceAsync(trajectoryManager.getDriveToSpikeMarksTrajectory());
                }
                break;
            // Carrying the pixels, moving to the center of the spike marks
            case MOVING_TO_SPIKE_MARKS:
                if (canContinue()) {
                    currentState = State.PLACING_PURPLE;
                    currentCommand = scheduleCommand(commandManager.getAutoPlacePurpleCommand());
                }
                break;
            // Placing the purple pixel on the spike mark using the intake
            case PLACING_PURPLE:
                if (canContinue()) {
                    // TODO: Make a way to always turn the correct direction at the end of auto
                    if (isUpstage && isPlacingYellow) {
                        currentState = State.MOVING_FROM_SPIKE_MARKS;
                        currentCommand = scheduleCommand(commandManager.getAutoMoveArmCommand());
                        drive.followTrajectorySequenceAsync(trajectoryManager.getDriveFromSpikeMarksTrajectory());
                    }
                    else currentState = State.IDLE;
                }
                break;
            // Moving out of the way of the pixel and game element
            case MOVING_FROM_SPIKE_MARKS:
                if (canContinue()) {
                    currentState = State.MOVING_TO_BACKDROP;
                    drive.followTrajectorySequenceAsync(trajectoryManager.getDriveToBackdropTrajectory());
                }
                break;
            // Moving to the correct spot to place the yellow pixel
            case MOVING_TO_BACKDROP:
                if (canContinue()) {
                    currentState = State.PLACING_YELLOW;
                    currentCommand = scheduleCommand(commandManager.getAutoPlaceYellowCommand());
                }
                break;
            // Placing the yellow pixel on the backstage
            case PLACING_YELLOW:
                if (canContinue()) {
                    currentState = State.HIDING;
                    drive.followTrajectorySequenceAsync(trajectoryManager.getDriveToCornerTrajectory());
                }
                break;
            // Moving to the corner to take up the least amount of room
            case HIDING:
                if (canContinue()) {
                    currentState = State.IDLE;
                    // We are done, idle
                }
                break;
            case IDLE:
                break;
        }

        CommandScheduler.getInstance().run();
        drive.update();
        telemetry.addData("Spike mark position", spikeMarkPosition);
        telemetry.addData("State", currentState);
        telemetry.update();

    }

    /**
     * Schedules a command to be run. Adds a 1 ms wait commands to the end of the provided command and returns it so you can see when it is done.
     * (Calling isFinished on a SequentialCommandGroup doesn't seem to work properly)
     *
     * @param command The command to be scheduled.
     * @return The wait command at the end of the sequential command.
     */
    private WaitCommand scheduleCommand(Command command) {
        WaitCommand waitCommand = new WaitCommand(1);
        new SequentialCommandGroup(command, waitCommand).schedule();
        return waitCommand;
    }

    private boolean canContinue() {
        return currentCommand.isFinished() && !drive.isBusy();
    }

    private void updateStatus(String text) {
        telemetry.addData("Status", text);
        telemetry.update();
    }

    public SampleMecanumDrive getDriveBase() {
        return drive;
    }

    public SpikeMark getSpikeMarkPosition() {
        return spikeMarkPosition;
    }

    public float getPositionMultiplier() {
        return positionMultiplier;
    }

    public boolean isBlueAlliance() {
        return isBlueAlliance;
    }

//    private void lineUpWithAprilTag(){
//        CameraSubsystem cameraSubsystem = new CameraSubsystem(hardwareMap);
//        updateStatus("Lining up with april tag ID: " + aprilTagID);
//
//        // Get to the correct distance away from the backdrop
//        do {
//            // Re-check distance to april tag
//            cameraSubsystem.detect();
//            aprilTagForwardDistance = cameraSubsystem.getDistance(aprilTagID);
//
//            telemetry.addData("Distance", aprilTagForwardDistance);
//
//            if (aprilTagForwardDistance != -1) {
//                // Move robot closer to april tag
//                if (aprilTagForwardDistance < aprilTagForwardTarget) {
//                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(aprilTagForwardTarget - aprilTagForwardDistance).build());
//                }
//                if (aprilTagForwardDistance > aprilTagForwardTarget) {
//                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(aprilTagForwardDistance - aprilTagForwardTarget).build());
//                }
//            }
//            telemetry.update();
//
//        } while (Math.abs(aprilTagForwardDistance - aprilTagForwardTarget) >= 0.25); // +-1/4" tolerance
//
////         Line up with april tag
//        do {
//            // Re-check lateral distance to april tag
//            aprilTagLateralDistance = cameraSubsystem.getLateralDistance(aprilTagID);
////
//            // Move robot closer to april tag
//            if (aprilTagLateralDistance < 0) {
//                telemetry.addData("Should be going left", aprilTagLateralDistance);
//                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(-aprilTagLateralDistance).build());
//            }
//            if (aprilTagLateralDistance > 0) {
//                telemetry.addData("Should be going right", aprilTagLateralDistance);
//                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(aprilTagLateralDistance).build());
//            }
//            telemetry.update();
//        } while (Math.abs(aprilTagLateralTarget - aprilTagLateralDistance) >= 1); // +-1" tolerance
//    }
}
