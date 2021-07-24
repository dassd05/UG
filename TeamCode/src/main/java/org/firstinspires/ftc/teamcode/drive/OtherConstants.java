package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.opencv.core.Point;

public class OtherConstants {
    public static final double MOTOR_TICKS_PER_REV = 28;
    public static final double MOTOR_GEAR_RATIO = 1;

    public static final double iOn = .8;
    public static final double iOff = 0;

    public static final double brOn = .7;
    public static final double brOff = 0;

    public static final int shooterOff = 0;

    //Ring Detecting Constants for Left Side
    public static final int LEFT_4_THRESHOLD = 145;
    public static final int LEFT_1_THRESHOLD = 132;

    public static final Point LEFT_REGION1_TOPLEFT_ANCHOR_POINT = new Point(1000, 780);

    public static final int LEFT_REGION_WIDTH = 280;
    public static final int LEFT_REGION_HEIGHT = 180;


    //todo:
    // make different shooter speeds so that we can change just this instead of changing it in
    // every auton
    public static final double shooterAtStack = 2665;
    public static final double shooterAfterIntake = 2590;
    public static final double shooterZeroRing = 2620;
//    public static double shooterSpeed1 = ;
//    public static double shooterSpeed2 = ;
//    public static double shooterSpeed3 = ;
//    public static double shooterSpeed4 = ;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(150, 0, 0, 25);
    public static PIDFCoefficients MOTOR_VELO_PID_2 = new PIDFCoefficients(150, 0, 0, 25);

    public static double lastKf = 17.7;
    public static double lastKf_2 = 17.7;
}
