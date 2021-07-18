package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class OtherConstants {
    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_GEAR_RATIO = 1;

    public static double iOn = .8;
    public static double iOff = 0;

    public static double brOn = .7;
    public static double brOff = 0;


    //different shooter speeds so that we can change just this instead of changing it in every auton/teleop
//    public static double shooterSpeed1 = ;
//    public static double shooterSpeed2 = ;
//    public static double shooterSpeed3 = ;
//    public static double shooterSpeed4 = ;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(45, 0, 0, 25);
    public static PIDFCoefficients MOTOR_VELO_PID_2 = new PIDFCoefficients(45, 0, 0, 25);

    public static double lastKf = 17.7;
    public static double lastKf_2 = 17.7;
}
