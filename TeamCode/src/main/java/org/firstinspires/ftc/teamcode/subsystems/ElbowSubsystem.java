package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.teamcode.Constants.LinearSlideConstants.PULSES_PER_INCH;
import static org.firstinspires.ftc.teamcode.Constants.WristConstants.WRIST_SERVO_MOTOR_NAME;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.ElbowConstants.*;

import android.os.Debug;

public class ElbowSubsystem {

    // One DcMotorEx
    // Method to spin the motor clockwise
    // Method to spin the motor counter clockwise
    // Method to set the arm to a specific position
    // Logic to limit the motor from going too high
    // Lower level method using PIDF to accurately set the position of the motor

    public DcMotorEx elbowMotor;

    public ElbowSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        elbowMotor = hardwareMap.get(DcMotorEx.class, ELBOW_DC_MOTOR_NAME);
    }
    public void spinMotorClockwise(){

    }

    public void spinMotorCounterClockwise(){

    }

    public void setArmPosition(){

    }

    public void setMotorPosition(){
        elbowMotor.setTargetPosition((int)MOTOR_POSITION);
    }

    public enum TargetMotorPosition {
        // The number of inches
        DEFAULT(0),
        LOW(-1),
        MEDIUM(-1),
        HIGH(-1);

        private int inches;

        TargetMotorPosition(int inches) {
            this.inches = inches;
        }
        int getTarget() {
            return (int) (inches * PULSES_PER_INCH);
        }
    }

    public enum TargetArmPosition {
        // The number of inches
        DEFAULT(0),
        LOW(-1),
        MEDIUM(-1),
        HIGH(-1);

        private int inches;

        TargetArmPosition(int inches) {
            this.inches = inches;
        }
        int getTarget() {
            return (int) (inches * PULSES_PER_INCH);
        }
    }
}
