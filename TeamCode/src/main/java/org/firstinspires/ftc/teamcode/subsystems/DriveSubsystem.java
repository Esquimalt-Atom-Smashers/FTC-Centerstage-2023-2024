package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// import android.os.Build;

// import com.qualcomm.hardware.bosch.BNO055IMU;
// import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.util.Range;

// import org.firstinspires.ftc.robotcore.external.Predicate;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// import java.nio.channels.ScatteringByteChannel;
// import java.util.Arrays;

// import teamcode.internal.util.EncoderConstants;


public class DriveSubsystem {
    //Declare hardware here.
    private final DcMotor frontRightMotor, frontLeftMotor, rearRightMotor, rearLeftMotor;

    //Initialize hardware here
    public DriveSubsystem(HardwareMap hardwareMap, Telemetry, telemetry) {
        // Initialize the motors
        frontRightMotor = harwareMap.dcMotor.get(Constants.DriveConstants.FRONT_RIGHT_MOTOR);
        frontLeftMotor = harwareMap.dcMotor.get(Constants.DriveConstants.FRONT_LEFT_MOTOR);
        rearRightMotor = harwareMap.dcMotor.get(Constants.DriveConstants.REAR_RIGHT_MOTOR);
        rearLeftMotor = harwareMap.dcMotor.get(Constants.DriveConstants.REAR_LEFT_MOTOR);
    }

    //Define methods that control the bot down here.

    public void drive(float forward, float strafe, float turn)
    {
        if (Constants.DriveConstants.FIELD_CENTRIC) {
            // Field centric drive
            // TODO: Field centric drive
        }
        else {
            // Normal drive (probably not going to be used)
            // TODO: (low) Robot centric drive
        }
    }

    //Define lower-level methods here. (Methods that are private or work behind the scenes)

    // Take the input (forward, strafe, turn) and scale it so that moving the joystick halfway doesn't use half power
    private float scaleInput(float input)
    {
        // TODO: Create a scaling formula
        return input
    }

    //Define getter/setter's here.
    
}
