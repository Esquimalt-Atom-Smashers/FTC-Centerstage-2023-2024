package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static org.firstinspires.ftc.teamcode.Constants.ClawConstants.*;

public class ClawSubsystem {

    private final ServoEx clawServo;
//    private final Servo clawServo;
    Telemetry telemetry;

    public ClawSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        clawServo = new SimpleServo(hardwareMap, CLAW_SERVO_NAME, MIN_POSITION, MAX_POSITION);
//        clawServo = hardwareMap.servo.get(CLAW_SERVO_NAME);
        this.telemetry = telemetry;

    }

    public void openClaw() {
        clawServo.turnToAngle(OPEN_POSITION);
//        clawServo.setPosition(OPEN_POSITION);
    }

    public void closeClaw() {
        clawServo.turnToAngle(CLOSE_POSITION);
//        clawServo.setPosition(CLOSE_POSITION);
    }

    private int count;
    public void printPosition() {
        count++;
        telemetry.addData("Count ", count);
        telemetry.addData("Claw servo angle ", clawServo.getAngle());
        telemetry.addData("Claw position ", clawServo.getPosition());
        telemetry.addData("Angle in degrees ", clawServo.getAngle(AngleUnit.DEGREES));
        telemetry.update();
    }

    public boolean clawOpen() {
        return clawServo.getPosition() == OPEN_POSITION;
    }

    public void toggleClaw() {
        if (clawOpen()) {
            closeClaw();
        } else {
            openClaw();
        }
    }

    public ServoEx getClawServo()
    {
        return clawServo;
    }
}