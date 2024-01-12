package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.CommandManager;
import org.firstinspires.ftc.teamcode.subsystems.BoxReleaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CustomSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WinchSubsystem;

/**
 * Represents all of the subsystems that make up the robot
 *
 * @author Esquimalt Atom Smashers
 */
public class Robot {
    /** The op mode that created the robot */
    private final OpMode opMode;

    /** The gamepad used by the driver to drive the robot around and open/close box*/
    private final GamepadEx driverGamepad;
    /** The gamepad used by the operator to control the arm */
    private final GamepadEx operatorGamepad;

    /** The box release door of the robot */
    private final BoxReleaseSubsystem boxReleaseSubsystem;
    /** The distance sensors of the robot */
    private final DistanceSensorSubsystem distanceSensorSubsystem;
    /** The drive base of the robot */
    private final DriveSubsystem driveSubsystem;
    /** The drone holder and launcher of the robot */
    private final DroneSubsystem droneSubsystem;
    /** The elbow of the robot */
    private final ElbowSubsystem elbowSubsystem;
    /** The intake of the robot */
    private final IntakeSubsystem intakeSubsystem;
    /** The linear slide of the robot */
    private final LinearSlideSubsystem linearSlideSubsystem;
    /** The winch of the robot */
    private final WinchSubsystem winchSubsystem;

    private final boolean manualMode;

    private final CommandManager commandManager;

    //private DriveState driveState = DriveState.DRIVER_CONTROLLED;
    private ScoringState scoringState = ScoringState.STARTING;

    /* enum DriveState {
        DRIVER_CONTROLLED,
        SNAPPING,
        DETECTING_TAG,
        CENTERING_TAG,
        STEPPING_LEFT,
        STEPPING_RIGHT
    }*/

    public enum ScoringState {
        STARTING,
        INTAKE,
        LOADING_PIXELS,
        DRIVING,
        MANUAL,
        SHOOTING_DRONE
    }

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
        boxReleaseSubsystem = new BoxReleaseSubsystem(opMode.hardwareMap, opMode.telemetry);
        distanceSensorSubsystem = new DistanceSensorSubsystem(opMode.hardwareMap, opMode.telemetry);
        driveSubsystem = new DriveSubsystem(opMode.hardwareMap, opMode.telemetry);
        droneSubsystem = new DroneSubsystem(opMode.hardwareMap, opMode.telemetry);
        elbowSubsystem = new ElbowSubsystem(opMode.hardwareMap, opMode.telemetry);
        intakeSubsystem = new IntakeSubsystem(opMode.hardwareMap, opMode.telemetry);
        linearSlideSubsystem = new LinearSlideSubsystem(opMode.hardwareMap, opMode.telemetry);
        winchSubsystem = new WinchSubsystem(opMode.hardwareMap, opMode.telemetry);

        commandManager = new CommandManager(this);

        if (!manualMode) bindCommands();
        if (resetEncoders) resetEncoders();
    }

    /** Binds the ftclib commands that control the robot. */
    private void bindCommands() {
        // --- BoxReleaseSubsystem ---
        // Press driver B to open the box
        Trigger openBoxTrigger = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.B) && scoringState == ScoringState.DRIVING);
        openBoxTrigger.whenActive(commandManager.getOpenBoxCommand());

        // Press driver A to close the box
        Trigger closeBoxTrigger = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.A) && scoringState == ScoringState.DRIVING);
        closeBoxTrigger.whenActive(commandManager.getCloseBoxCommand());

        // --- DriveSubsystem ---
        driveSubsystem.setDefaultCommand(commandManager.getDefaultDriveCommand());

        Trigger resetGyroTrigger = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.BACK));
        resetGyroTrigger.whenActive(commandManager.getResetGyroCommand());

        // Press driver left dpad while in driving mode to snap to left side of field
        Trigger snapLeftTrigger = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.DPAD_LEFT) && scoringState == ScoringState.DRIVING);
        snapLeftTrigger.whenActive(commandManager.getSnapLeftCommand());

        // Press driver up dpad while in driving mode to snap to forward (far edge) side of field
        Trigger snapUpTrigger = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.DPAD_UP) && scoringState == ScoringState.DRIVING);
        snapUpTrigger.whenActive(commandManager.getSnapUpCommand());

        // Press driver right dpad while in driving mode to snap to right side of field
        Trigger snapRightTrigger = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) && scoringState == ScoringState.DRIVING);
        snapRightTrigger.whenActive(commandManager.getSnapRightCommand());

        // Press driver down dpad while in driving mode to snap to back (near edge) side of field
        Trigger snapDownTrigger = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.DPAD_DOWN) && scoringState == ScoringState.DRIVING);
        snapDownTrigger.whenActive(commandManager.getSnapDownCommand());

        // --- DroneSubsystem ---
        // Press operator A to raise the arm and enter shooting drone mode
        Trigger droneLaunchModeTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.A));
        droneLaunchModeTrigger.whenActive(commandManager.getDroneModeCommand());

        // Press operator B to launch the drone
        Trigger droneLaunchTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.B) && scoringState == ScoringState.SHOOTING_DRONE);
        droneLaunchTrigger.whenActive(commandManager.getDroneLaunchCommand());

        // Returns launch servo to start position using operator button Y, does not move arm
        Trigger cancelDroneModeTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.Y) && scoringState == ScoringState.SHOOTING_DRONE);
        cancelDroneModeTrigger.whenActive(commandManager.getDroneCancelCommand());

        // --- ElbowSubsystem ---
        elbowSubsystem.setDefaultCommand(commandManager.getDefaultElbowCommand());

        // --- LinearSlideSubsystem ---
        linearSlideSubsystem.setDefaultCommand(commandManager.getDefaultSlideCommand());

        // --- WinchSubsystem ---
        winchSubsystem.setDefaultCommand(commandManager.getDefaultWinchCommand());

        // --- IntakeSubsystem ---
        // Press operator RT to enter intake mode, hold operator dpad up to outtake
        Trigger intakeTrigger = new Trigger(() -> isPressed(operatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) && scoringState != ScoringState.INTAKE);
        intakeTrigger.whenActive(commandManager.getIntakeModeCommand());

        // Press operator RT to pickup the pixels while the arm is down
        Trigger pickupPixelsTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) && scoringState == ScoringState.INTAKE);
        pickupPixelsTrigger.whenActive(commandManager.getPickupPixelsCommand());

        // Press operator RB to exit intake mode as well as canceling the pickup command if it's running (as an emergency stop)
//        Trigger cancelIntakeTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER));
//        cancelIntakeTrigger.whenActive(commandManager.getIntakeCancelCommand());

        // --- Complex Commands (commands that use more than one subsystem) ---
        // Press operator left dpad while in driving mode to move the arm to low preset scoring position
        Trigger lowScoringPositionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_LEFT) && scoringState == ScoringState.DRIVING);
        lowScoringPositionTrigger.whenActive(commandManager.getLowScoringPositionCommand());

        // Press operator up dpad while in driving mode to move the arm to medium scoring position
        Trigger mediumScoringPositionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_UP) && scoringState == ScoringState.DRIVING);
        mediumScoringPositionTrigger.whenActive(commandManager.getMediumScoringPositionCommand());

        // Press operator right dpad while in driving mode to move the arm to high scoring position
        Trigger highScoringPositionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) && scoringState == ScoringState.DRIVING);
        highScoringPositionTrigger.whenActive(commandManager.getHighScoringPositionCommand());

        // Press operator down dpad while in driving mode to move the arm to home position (used while driving)
        Trigger homePositionTrigger = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_DOWN) && scoringState == ScoringState.DRIVING);
        homePositionTrigger.whenActive(commandManager.getHomePostionCommand());
    }

    /** Resets the encoders on the subsystems that use them. */
    public void resetEncoders() {
        linearSlideSubsystem.resetEncoder();
        elbowSubsystem.resetEncoder();
        driveSubsystem.resetEncoder();
    }

    /** Schedule any commands that run at the start of teleop mode. */
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

        opMode.telemetry.addData("Scoring state", scoringState);
        distanceSensorSubsystem.printData();
        opMode.telemetry.update();
    }

    /**
     * Drives the robot using the gamepad inputs, only used in manual mode
     */
    public void startManual() {
        droneSubsystem.startPosition();
    }

    /**
     * Controls the elbow, intake, slide, box, drone and drive subsystem manually, without any commands running or PID controllers.
     */
    public void runManually() {
        driveSubsystem.drive(driverGamepad, isPressed(driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) ? 0.3 : 1);

        if (driverGamepad.getButton(GamepadKeys.Button.BACK)) driveSubsystem.resetGyro();

        // Elbow Subsystem (operator)
        // Move left joystick up to move the arm up, down to move down
        if (isPressed(operatorGamepad.getLeftY())) elbowSubsystem.moveManually(operatorGamepad.getLeftY());
        else elbowSubsystem.stopMotor();

        // Linear Slide Subsystem (operator)
        // Move right joystick up to move slide out, down to move in
        if (isPressed(operatorGamepad.getRightY())) linearSlideSubsystem.moveManually(operatorGamepad.getRightY());
        else linearSlideSubsystem.stopMotor();

        // Intake Subsystem (operator)
        // Up -> Up, Right -> Medium, Down -> Down, left trigger -> intake, right trigger -> outtake
        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_UP)) intakeSubsystem.upPosition();
        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)) intakeSubsystem.mediumPosition();
        if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) intakeSubsystem.downPosition();

        if (operatorGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) intakeSubsystem.outtake();
        else if (isPressed(operatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))) intakeSubsystem.intake();
        else intakeSubsystem.stopMotor();

        // Box release Subsystem (driver)
        // B -> Open/Red, A -> Close/Green
        if (driverGamepad.getButton(GamepadKeys.Button.B)) boxReleaseSubsystem.openBox();
        if (driverGamepad.getButton(GamepadKeys.Button.A)) boxReleaseSubsystem.closeBox();
/*        if (driverGamepad.getButton(GamepadKeys.Button.A)) {
            if (boxReleaseSubsystem.boxOpen){
                boxReleaseSubsystem.closeBox();
            } else {
                boxReleaseSubsystem.openBox();
            }
        }*/

        // Drone subsystem (operator)
        // B -> Release, A-> Go to start position with elbow up, Y -> Go to start position for servo only
        if (operatorGamepad.getButton(GamepadKeys.Button.B)) droneSubsystem.release();
        if (operatorGamepad.getButton(GamepadKeys.Button.A)) {
            droneSubsystem.startPosition();
            elbowSubsystem.moveManually(Constants.ElbowConstants.DRONE_LAUNCH_POSITION);
            elbowSubsystem.stopMotor();
        }
        if (operatorGamepad.getButton(GamepadKeys.Button.Y)) droneSubsystem.startPosition();

        // Winch subsystem (operator)
        // left trigger -> Winch in, left bumper -> Winch out
        if (isPressed(operatorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))) winchSubsystem.unwinch();
        else if (operatorGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) winchSubsystem.winch();
        else winchSubsystem.stopMotor();

        if (operatorGamepad.getButton(GamepadKeys.Button.BACK)) resetEncoders();

        if (operatorGamepad.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) boxReleaseSubsystem.disableLights();

        elbowSubsystem.printData();
        linearSlideSubsystem.printData();
        distanceSensorSubsystem.printData();
        boxReleaseSubsystem.printData();
        intakeSubsystem.printData();
        opMode.telemetry.update();
    }

    public void setScoringState(ScoringState newState) {
        scoringState = newState;
    }

    public ScoringState getScoringState() {
        return scoringState;
    }

    public GamepadEx getOperatorGamepad() {
        return operatorGamepad;
    }

    public GamepadEx getDriverGamepad() {
        return driverGamepad;
    }

    public BoxReleaseSubsystem getBoxReleaseSubsystem() {
        return boxReleaseSubsystem;
    }

    public DistanceSensorSubsystem getDistanceSensorSubsystem() {
        return distanceSensorSubsystem;
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
     *
     * @param controllerInput the input from the controller, for example gamepad.left_bumper or gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
     * @return whether the input from the controller has passed the dead zone
     */
    public boolean isPressed(double controllerInput) {
        return Math.abs(controllerInput) >= Constants.DriveConstants.DEADZONE;
    }
}