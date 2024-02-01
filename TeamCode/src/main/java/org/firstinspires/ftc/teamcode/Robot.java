package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.CommandManager;
import org.firstinspires.ftc.teamcode.controller.CustomController;
import org.firstinspires.ftc.teamcode.subsystems.BoxReleaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
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
    /** The LED's on the side of the robot */
    private final LEDSubsystem ledSubsystem;

    private final boolean manualMode;

    private final CommandManager commandManager;

    private RobotState state = RobotState.DRIVING;

    private final CustomController customDriverController;
    private final CustomController customOperatorController;

    public enum RobotState {
        DRIVING,
        INTAKE,
        LOADING_PIXELS,
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
        ledSubsystem = new LEDSubsystem(opMode.hardwareMap, opMode.telemetry);

        commandManager = new CommandManager(this);
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();

        customDriverController = new CustomController(driverGamepad);
        customOperatorController = new CustomController(operatorGamepad);

        if (!manualMode) bindCommands();
        if (resetEncoders) resetEncoders();
    }

    /** Binds the ftclib commands that control the robot. */
    private void bindCommands() {
        customDriverController
                .onAPressed().whenActive(commandManager.getHomePostionCommand())
                .onRightPressed().whenActive(commandManager.getSnapRightCommand())
                .onLeftPressed().whenActive(commandManager.getSnapLeftCommand())
                .onUpPressed().whenActive(commandManager.getSnapUpCommand())
                .onDownPressed().whenActive(commandManager.getSnapDownCommand())
                .onBackPressed().whenActive(commandManager.getResetGyroCommand());

        customOperatorController
                .onAPressed().toggleWhenActive(commandManager.getIntakeModeCommand(), commandManager.getPickupPixelsCommand())
                .onBPressed().toggleWhenActive(commandManager.getOuttakeCommand(), commandManager.getIntakeCommand())
                .onXPressed().whenActive(commandManager.getOpenBoxCommand())
                .onYPressed().and(() -> isState(RobotState.SHOOTING_DRONE)).whenActive(commandManager.getDroneCancelCommand())
                .onLeftPressed().whenActive(commandManager.getLowScoringPositionCommand())
                .onUpPressed().whenActive(commandManager.getMediumScoringPositionCommand())
                .onRightPressed().whenActive(commandManager.getHighScoringPositionCommand())
                .onDownPressed().whenActive(commandManager.getHomePostionCommand())
                .onRightBumperPressed().whenActive(commandManager.getDroneModeCommand())
                .onRightTrigger(0.1).and(() -> isState(RobotState.SHOOTING_DRONE)).whenActive(commandManager.getDroneLaunchCommand());

        // --- DriveSubsystem ---
        driveSubsystem.setDefaultCommand(commandManager.getDefaultDriveCommand());

        // --- ElbowSubsystem ---
        elbowSubsystem.setDefaultCommand(commandManager.getDefaultElbowCommand());

        // --- LinearSlideSubsystem ---
        linearSlideSubsystem.setDefaultCommand(commandManager.getDefaultSlideCommand());

        // --- WinchSubsystem ---
        winchSubsystem.setDefaultCommand(commandManager.getDefaultWinchCommand());
    }

    /** Resets the encoders on the subsystems that use them. */
    public void resetEncoders() {
        linearSlideSubsystem.resetEncoder();
        elbowSubsystem.resetEncoder();
        driveSubsystem.resetEncoder();
    }

    /** Schedule any commands that run at the start of teleop mode. */
    public void start() {
//        commandManager.getSetupCommand().schedule();
    }

    /**
     * Run the robot, including calling CommandScheduler.getInstance().run(), which polls the gamepad inputs and performs scheduled commands.
     * Also prints data from the subsystems and calls telemetry.update()
     */
    public void run() {
        // Run the command scheduler, which polls the gamepad inputs, and performs the commands created in initCommands
        if (!manualMode)
            CommandScheduler.getInstance().run();

        opMode.telemetry.addData("Right Y", operatorGamepad.getRightY());
        opMode.telemetry.addData("Scoring state", state);
        elbowSubsystem.printData();
        linearSlideSubsystem.printData();
        distanceSensorSubsystem.printData();
        opMode.telemetry.update();
    }

    public void startManual() {

    }

    /**
     * Controls the elbow, intake, slide, box, drone and drive subsystem manually, without any commands running or PID controllers.
     */
    public void runManually() {
        driveSubsystem.drive(driverGamepad.getLeftY(), driverGamepad.getLeftX(), driverGamepad.getRightX(), !isPressed(driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)), false, isPressed(driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) ? 0.3 : 1);

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
        if (operatorGamepad.getButton(GamepadKeys.Button.B) && !operatorGamepad.getButton(GamepadKeys.Button.START)) boxReleaseSubsystem.openBox();
        if (operatorGamepad.getButton(GamepadKeys.Button.A) && !operatorGamepad.getButton(GamepadKeys.Button.START)) boxReleaseSubsystem.closeBox();
/*        if (driverGamepad.getButton(GamepadKeys.Button.A)) {
            if (boxReleaseSubsystem.boxOpen){
                boxReleaseSubsystem.closeBox();
            } else {
                boxReleaseSubsystem.openBox();
            }
        }*/

        // Drone subsystem (operator)
        // X -> Release, Y -> Go to start position
        if (operatorGamepad.getButton(GamepadKeys.Button.X)) droneSubsystem.release();
        if (operatorGamepad.getButton(GamepadKeys.Button.Y)) droneSubsystem.startPosition();

        // Winch subsystem (operator)
        // left trigger -> Winch in, left bumper -> Winch out
        if (isPressed(operatorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))) winchSubsystem.unwinch();
        else if (operatorGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) winchSubsystem.winch();
        else winchSubsystem.stopMotor();

        if (operatorGamepad.getButton(GamepadKeys.Button.BACK)) resetEncoders();

        if (operatorGamepad.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) boxReleaseSubsystem.disableLights();

        opMode.telemetry.addData("Gyro heading: ",  driveSubsystem.getHeading());


        elbowSubsystem.printData();
        linearSlideSubsystem.printData();
        distanceSensorSubsystem.printData();
        boxReleaseSubsystem.printData();
        intakeSubsystem.printData();
        opMode.telemetry.update();
    }

    public void setState(RobotState newState) {
        state = newState;
    }

    public RobotState getState() {
        return state;
    }

    public boolean isState(RobotState state) {
        return this.state == state;
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

    public LEDSubsystem getLedSubsystem() {
        return ledSubsystem;
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