package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.WristConstants.WRIST_SERVO_MOTOR_NAME;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.ElbowConstants.*;

public class ElbowSubsystem {
    // One DcMotorEx
    // Method to spin the motor clockwise
    // Method to spin the motor counter clockwise
    // Method to set the arm to a specific position
    // Logic to limit the motor from going too high
    // Lower level method using PIDF to accurately set the position of the motor

    public DcMotor elbowMotor;

    public ElbowSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        elbowMotor = hardwareMap.dcMotor.get(ELBOW_DC_MOTOR_NAME);
    }
    public void spinMotorClockwise(){

    }

    public void spinMotorCounterClockwise(){

    }

    public void setArmPosition(){

    }

    public void setMotorPosition(){

    }
}
