package org.firstinspires.ftc.teamcode;

//This class holds all constants, the reason we do this is so we can import all the constants 'statically.'
public class Constants {
    public static class DriveConstants {
        /* Define all driving constants here i.e:
            - configuration names
            - PID values(gain)
            - booleans(USE_FIELD_CENTRIC)
            - limits(MAX_SPEED_M_S)
        */
        public static String FRONT_LEFT_MOTOR_NAME = " ";
        public static String REAR_LEFT_MOTOR_NAME = " ";
        public static String FRONT_RIGHT_MOTOR_NAME = " ";
        public static String REAR_RIGHT_MOTOR_NAME = " ";

        public static boolean FIELD_CENTRIC = false;
        public static boolean SCALED = false;

        public static float AUTO_DRIVE_SPEED;
        public static float AUTO_STRAFE_SPEED;
        public static float TURN_SPEED;
    }

    public static class ArmConstants 
    {
        public static String CLAW1_SERVO_MOTOR_NAME = " ";
        public static String CLAW2_SERVO_MOTOR_NAME = " ";
    }

}
