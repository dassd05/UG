package org.firstinspires.ftc.teamcode.drive.advanced;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotFunctions {
    /*
    TODO:
     rename this
     maybe catch nullpointerexception when a device isn't configured/initialized properly
     maybe make this "robot" as base for all opmodes
     add functions for driving, shooting, wobble goal, etc.
     set privacy of the fields and methods here
     make documentation for this (and everything)
     */

    private DcMotorEx frontLeft, backLeft, backRight, frontRight, frontShoot, backShoot;
    private DcMotorSimple intake1, intake2;
//    private List<DcMotorEx> motors;
    Servo liftServo, wobbleClawServo, wobbleArmServo, shootFlicker;
    Servo[] servos;
//    private BNO055IMU imu;


    public RobotFunctions(HardwareMap hardwareMap) {
//        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
//        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
//        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
//        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        frontShoot = hardwareMap.get(DcMotorEx.class, "frontShoot");
        backShoot = hardwareMap.get(DcMotorEx.class, "backShoot");

        intake1 = hardwareMap.get(DcMotorSimple.class, "intake1");
        intake2 = hardwareMap.get(DcMotorSimple.class, "intake2");

//        motors = Arrays.asList(frontLeft, backLeft, backRight, frontRight);

        liftServo = hardwareMap.get(Servo.class, "liftServo");
        wobbleClawServo = hardwareMap.get(Servo.class, "wobbleClawServo");
        wobbleArmServo = hardwareMap.get(Servo.class, "wobbleArmServo");
        shootFlicker = hardwareMap.get(Servo.class, "shootFlicker");


        servos = new Servo[]{liftServo, wobbleClawServo, wobbleArmServo, shootFlicker};
    }

    public void grabWobbleGoal() {
        wobbleClawServo.setPosition(0.8);
        sleep(700);
    }
    public void releaseWobbleGoal() {
        wobbleClawServo.setPosition(0.3);
        sleep(700);
    }
    public void raiseWobbleArm() {
        wobbleArmServo.setPosition(0.8);
        sleep(1400);
    }
    public void dropOffWobbleArm() {
        wobbleArmServo.setPosition(0.5);
        sleep(700);
    }
    public void lowerWobbleArm() {
        wobbleArmServo.setPosition(0.3);
        sleep(700);
    }

    public void prepareShooter() {
        frontShoot.setPower(0.7); // dk power yet
        backShoot.setPower(0.7);
    }
    public void shoot() {
        // MUST MAKE ASYNCHRONOUSLY TO MAIN PROGRAM
        shootFlicker.setPosition(0.45);
        shootFlicker.setPosition(0.1);
    }

    public void raiseLift() {
        liftServo.setPosition(0.08);
        sleep(1500);
    }
    public void lowerLift() {
        liftServo.setPosition(0.58);
        sleep(1500);
    }

    public void startIntake() {
        intake1.setPower(0.5); // dk power yet
        intake2.setPower(0.5);
    }
    public void pauseIntake() {
        intake1.setPower(0);
        intake2.setPower(0);
    }

    // DK abt this
    // maybe change float to double
    // idk abt other runtopoint algo
    public void runToPoint(float x, float y) {

    }

    // idk abt this either
    // not sure if pose2d is right
    // also not sure abt other localization funcs
    // adjust as necessary
//    public Pose2d getPosition() {
//
//    }

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
