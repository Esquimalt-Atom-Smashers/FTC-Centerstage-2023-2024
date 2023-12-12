package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.CommandManager;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WinchSubsystem;

public class Robot {
    // OpMode
    private final OpMode opMode;

    // Gamepads
    private final GamepadEx driverGamepad;
    private final GamepadEx operatorGamepad;

    //Define subsystems here.
    private final DriveSubsystem driveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ElbowSubsystem elbowSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final LinearSlideSubsystem linearSlideSubsystem;
    private final CameraSubsystem cameraSubsystem;
    private final DroneSubsystem droneSubsystem;
    private final WinchSubsystem winchSubsystem;

    private final boolean manualMode;

    private final CommandManager commandManager;

    enum DriveState {
        DRIVER_CONTROLLED,
        SNAPPING,
        DETECTING_TAG,
        CENTERING_TAG,
        STEPPING_LEFT,
        STEPPING_RIGHT
    }

    public enum ScoringState {
        STARTING,
        INTAKE,
        LOADING_PIXELS,
        DRIVING,
        MANUAL,
        SHOOTING_DRONE
    }

    private DriveState driveState = DriveState.DRIVER_CONTROLLED;
    private ScoringState scoringState = ScoringState.STARTING;

    /**
     * Initializes all gamepads, subsystems, and if necessary, initializes the commands through {@link #bindCommands()}
     *
     * @param opMode The opMode that created the Robot
     * @param manualMode Whether we are using commands or not, default is false
     * @param resetEncoders Whether we reset the encoders on the slide, elbow and drive subsystems
     */
    public Robot(OpMode opMode, boolean manualMode, boolean resetEncoders) {
        this.opMode = opMode;
        this.manualMode = manualMode;

        // Initialize the gamepads
        driverGamepad = new GamepadEx(opMode.gamepad1);
        operatorGamepad = new GamepadEx(opMode.gamepad2);

        // Initialize the subsystems
        driveSubsystem = new DriveSubsystem(opMode.hardwareMap);
        intakeSubsystem = new IntakeSubsystem(opMode.hardwareMap);
        elbowSubsystem = new ElbowSubsystem(opMode.hardwareMap);
        clawSubsystem = new ClawSubsystem(opMode.hardwareMap);
        linearSlideSubsystem = new LinearSlideSubsystem(opMode.hardwareMap);
        cameraSubsystem = new CameraSubsystem(opMode.hardwareMap);
        droneSubsystem = new DroneSubsystem(opMode.hardwareMap);
        winchSubsystem = new WinchSubsystem(opMode.hardwareMap);

        commandManager = new CommandManager(this);

        if (!manualMode) bindCommands();
        if (resetEncoders) resetEncoders();
    }

    public Robot(OpMode opMode) {
        this(opMode, false, true);
    }

    /**
     * Binds the ftclib commands that control the robot.
     */
    private void bindCommands() {
        // Default commands for individual subsystems:

        // ClawSubsystem
        clawSubsystem.setDefaultCommand(commandManager.getDefaultClawCommand());

        Trigger clawTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.Y) && scoringState == ScoringState.DRIVING);
        clawTrigger.whenActive(commandManager.getOpenClawCommand());

        // DriveSubsystem
        driveSubsystem.setDefaultCommand(commandManager.getDefaultDriveCommand());

        // DroneSubsystem
        Trigger droneLaunchModeTrigger = new Trigger(() -> isPressed(driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        droneLaunchModeTrigger.whenActive(commandManager.getDroneModeCommand());

        Trigger droneLaunchTrigger = new Trigger(() -> isPressed(driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) && scoringState == ScoringState.SHOOTING_DRONE);
        droneLaunchTrigger.whenActive(commandManager.getDroneLaunchCommand());

        Trigger cancelDroneModeTrigger = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.B) && scoringState == ScoringState.SHOOTING_DRONE);
        cancelDroneModeTrigger.whenActive(commandManager.getDroneCancelCommand());

        // ElbowSubsystem
        elbowSubsystem.setDefaultCommand(commandManager.getDefaultElbowCommand());

        // LinearSlideSubsystem
        linearSlideSubsystem.setDefaultCommand(commandManager.getDefaultSlideCommand());

        // WinchSubsystem
        winchSubsystem.setDefaultCommand(commandManager.getDefaultWinchCommand());

        // Controls for command groups
        Trigger intakeTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.A));
        intakeTrigger.whenActive(commandManager.getIntakeModeCommand());

        Trigger cancelIntakeTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.B) && scoringState == ScoringState.INTAKE);
        cancelIntakeTrigger.whenActive(commandManager.getIntakeCancelCommand());

        // Press X when in intake mode to pick up pixels
        Trigger pickPixelsTrigger = new Trigger(() -> scoringState == ScoringState.INTAKE && operatorGamepad.getButton(GamepadKeys.Button.X));
        pickPixelsTrigger.whenActive(commandManager.getPickupPixelsCommand());

        // Press left dpad while in driving mode to move the arm to low preset scoring position
        Trigger lowScoringPositionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_LEFT) && scoringState == ScoringState.DRIVING);
        lowScoringPositionTrigger.whenActive(commandManager.getLowScoringPositionCommand());

        // Press up dpad while in driving mode to move the arm to medium scoring position
        Trigger mediumScoringPositionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_UP) && scoringState == ScoringState.DRIVING);
        mediumScoringPositionTrigger.whenActive(commandManager.getMediumScoringPositionCommand());

        // Press right dpad while in driving mode to move the arm to high scoring position
        Trigger highScoringPositionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) && scoringState == ScoringState.DRIVING);
        highScoringPositionTrigger.whenActive(commandManager.getHighScoringPositionCommand());

        // Press down dpad while in driving mode to move the arm to home position (used while driving)
        Trigger homePositionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_DOWN) && scoringState == ScoringState.DRIVING);
        homePositionTrigger.whenActive(commandManager.getHomePostionCommand());
    }

    public void resetEncoders() {
        linearSlideSubsystem.resetEncoder();
        elbowSubsystem.resetEncoder();
        driveSubsystem.resetEncoder();
    }

    /**
     * Schedule any commands that run at the start of teleop mode
     */
    public void start() {
        commandManager.getSetupCommand().schedule();
    }

    /**
     * Run the robot, including calling CommandScheduler.getInstance().run(), which polls the gamepad inputs and performs scheduled commands.
     * Also prints data from the subsystems and calls telemetry.update()
     */
    public void run() {
        // Run the command scheduler, which polls the gamepad inputs, and performs the commands created in initCommands
        if (!manualMode)
            CommandScheduler.getInstance().run();

        // Print data from subsystems
//        opMode.telemetry.addData("Scoring state", scoringState);
//        opMode.telemetry.addData("Driving state", driveState);
//        elbowSubsystem.printData(opMode.telemetry);
        driveSubsystem.printData(opMode.telemetry);
        linearSlideSubsystem.printData(opMode.telemetry);
        opMode.telemetry.update();
    }

    /**
     * Drives the robot using the gamepad inputs, only used in manual mode
     */
//    private void driveLoop() {
//        switch (driveState) {
//            case DRIVER_CONTROLLED:
//                driveSubsystem.drive(driverGamepad.getLeftY(), driverGamepad.getLeftX(), driverGamepad.getRightX());
//                if (driverGamepad.getButton(GamepadKeys.Button.A)) {
////                    driveSubsystem.autoSnap();
////                    driveState = DriveState.SNAPPING;
//                }
//                break;
//            case SNAPPING:
//                if (driveSubsystem.isFinishedSnapping()) {
//                    driveState = DriveState.DETECTING_TAG;
//                }
//                break;
//            case DETECTING_TAG:
//                break;
//            case CENTERING_TAG:
////                if (driveSubsystem.isCentered()) {
////                    instructionExecutor.getNextInstruction();
////                }
//                break;
//            case STEPPING_LEFT:
//                if (driveSubsystem.isFinishedSteppingLeft()) {
//                    driveState = DriveState.DRIVER_CONTROLLED;
//                }
//                break;
//            case STEPPING_RIGHT:
//                if (driveSubsystem.isFinishedSteppingRight()) {
//                    driveState = DriveState.DRIVER_CONTROLLED;
//                }
//                break;
//            default:
//                driveState = DriveState.DRIVER_CONTROLLED;
//        }
//    }


    /**
     * Controls the elbow, intake, slide, claw, drone and drive subsystem manually, without any commands running or PID controllers.
     */
    public void runManually() {

//        driveLoop();

        driveSubsystem.drive(driverGamepad.getLeftY(), driverGamepad.getLeftX(), driverGamepad.getRightX());

        // Elbow Subsystem (operator)
        // X -> Raise, Y -> Lower, Moving the left joystick from bottom to top increases the speed
        if (operatorGamepad.getButton(GamepadKeys.Button.X)) elbowSubsystem.raiseManually((operatorGamepad.getLeftY() + 1) / 2);
        else if (operatorGamepad.getButton(GamepadKeys.Button.Y)) elbowSubsystem.lowerManually((operatorGamepad.getLeftY() + 1) / 2);
        else elbowSubsystem.stopMotor();
        elbowSubsystem.printData(opMode.telemetry);

        // Intake Subsystem (operator)
        // Left -> Intake, Right -> Outtake, Up -> Move intake up, Down -> Move intake down
        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_UP)) intakeSubsystem.mediumPosition();
        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) intakeSubsystem.downPosition();
        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) intakeSubsystem.intake();
        else if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)) intakeSubsystem.outtake();
        else intakeSubsystem.stopMotor();

        // Linear Slide Subsystem (operator)
        // Right bumper -> Extend, Left bumper -> Retract, Moving the left joystick from bottom to top increases the speed
        if (operatorGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) linearSlideSubsystem.extendManually((operatorGamepad.getLeftY() + 1) / 2);
        else if (operatorGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) linearSlideSubsystem.retractManually((operatorGamepad.getLeftY() + 1) / 2);
        else linearSlideSubsystem.stopMotor();
        linearSlideSubsystem.printData(opMode.telemetry);

        // Claw Subsystem (operator)
        // A -> Open, B -> Close
        if (operatorGamepad.getButton(GamepadKeys.Button.A)) clawSubsystem.openClaw();
        if (operatorGamepad.getButton(GamepadKeys.Button.START)) clawSubsystem.closeClawSingle();
        if (operatorGamepad.getButton(GamepadKeys.Button.B)) clawSubsystem.closeClaw();
        clawSubsystem.printPosition(opMode.telemetry);

        // Drone subsystem (driver)
        // Right bumper -> Release, Left bumper -> Go to start position
        if (driverGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) droneSubsystem.release();
        if (driverGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) droneSubsystem.startPosition();

        // Winch subsystem (driver)
        // A -> Winch, B -> Unwinch
        if (driverGamepad.getButton(GamepadKeys.Button.A)) winchSubsystem.winch();
        else if (driverGamepad.getButton(GamepadKeys.Button.B)) winchSubsystem.unwinch();
        else winchSubsystem.stopMotor();

        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_UP)) linearSlideSubsystem.resetEncoder();

        linearSlideSubsystem.printData(opMode.telemetry);
        opMode.telemetry.update();

    }

    public void setScoringState(ScoringState newState) {
        scoringState = newState;
    }

    public GamepadEx getOperatorGamepad() {
        return operatorGamepad;
    }

    public GamepadEx getDriverGamepad() {
        return driverGamepad;
    }

    public ClawSubsystem getClawSubsystem() {
        return clawSubsystem;
    }

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    public DroneSubsystem getDroneSubsystem() {
        return droneSubsystem;
    }

    public ElbowSubsystem getElbowSubsystem() {
        return elbowSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    public LinearSlideSubsystem getLinearSlideSubsystem() {
        return linearSlideSubsystem;
    }

    public WinchSubsystem getWinchSubsystem() {
        return winchSubsystem;
    }

    /**
     * Checks if a input from the controller is outside the dead zone
     * @param controllerInput the input from the controller, for example gamepad.left_bumper or gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
     * @return whether the input from the controller has passed the dead zone
     */
    private boolean isPressed(double controllerInput) {
        return Math.abs(controllerInput) >= Constants.DriveConstants.DEADZONE;
    }
}