package org.firstinspires.ftc.teamcode.drive.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

/**
 * Example opmode demonstrating how to hand-off the pose from your autonomous opmode to your teleop
 * by passing the data through a static class.
 * <p>
 * This is required if you wish to read the pose from odometry in teleop and you run an autonomous
 * sequence prior. Without passing the data between each other, teleop isn't completely sure where
 * it starts.
 * <p>
 * This example runs the same paths used in the SplineTest tuning opmode. After the trajectory
 * following concludes, it simply sets the static value, `PoseStorage.currentPose`, to the latest
 * localizer reading.
 * However, this method is not foolproof. The most immediate problem is that the pose will not be
 * written to the static field if the opmode is stopped prematurely. To work around this issue, you
 * need to continually write the pose to the static field in an async trajectory follower. A simple
 * example of async trajectory following can be found at
 * https://www.learnroadrunner.com/advanced.html#async-following
 * A more advanced example of async following can be found in the AsyncFollowingFSM.java class.
 * <p>
 * The other edge-case issue you may want to cover is saving the pose value to disk by writing it
 * to a file in the event of an app crash. This way, the pose can be retrieved and set even if
 * something disastrous occurs. Such a sample has not been included.
 */
@Autonomous(group = "advanced")
public class OneRing extends LinearOpMode {

    private DcMotorEx frontShoot, backShoot;
    private Servo wobbleClawServo, wobbleArmServo;
    private Servo liftServo, shootFlicker;
    private DcMotor intake1, intake2;

    double integralf = 0;
    double integralb = 0;

    public static PIDCoefficients pidConstsf = new PIDCoefficients(0.4, 0, 83);
    public static PIDCoefficients pidConstsb = new PIDCoefficients(0.4, 0, 181);

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void runOpMode() throws InterruptedException {

        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");

        backShoot = hardwareMap.get(DcMotorEx.class, "backShoot");
        frontShoot = hardwareMap.get(DcMotorEx.class, "frontShoot");

        frontShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftServo = hardwareMap.servo.get("liftServo");
        wobbleClawServo = hardwareMap.servo.get("wobbleClawServo");
        wobbleArmServo = hardwareMap.servo.get("wobbleArmServo");
        shootFlicker = hardwareMap.servo.get("shootFlicker");

        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        backShoot.setDirection(DcMotorSimple.Direction.REVERSE);
        frontShoot.setDirection(DcMotorSimple.Direction.REVERSE);
        // Declare your drive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-50, -2, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        wobbleArmServo.setPosition(1);
        sleep(5000);
        wobbleClawServo.setPosition(.9);

        liftServo.setPosition(.08);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));

            // Example spline path from SplineTest.java
            // Make sure the start pose matches with the localizer's start pose
            Trajectory traj = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(45, 45), 0)
                    .build();

            drive.followTrajectory(traj);

            // Transfer the current pose to PoseStorage so we can use it in TeleOp
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
    }
    double lastErrorf = 0;
    double lastErrorb = 0;

    public void runShooterMotors(double targetVelocity) {
        PIDTimer.reset();

        double currentVelocityf = frontShoot.getVelocity();

        double errorf = currentVelocityf - targetVelocity;

        double changeInErrorf = lastErrorf - errorf;
        integralf += -errorf * PIDTimer.time();
        double derivativef = changeInErrorf / PIDTimer.time();

        double Pf = pidConstsf.p * -errorf;
        double If = pidConstsf.i * integralf;
        double Df = pidConstsf.d * derivativef;

        frontShoot.setVelocity(Pf + If + Df + targetVelocity);

        lastErrorf = errorf;

        double currentVelocityb = backShoot.getVelocity();

        double errorb = currentVelocityb - targetVelocity;

        double changeInErrorb = lastErrorb - errorb;
        integralb += -errorb * PIDTimer.time();
        double derivativeb = changeInErrorb / PIDTimer.time();

        double Pb = pidConstsb.p * -errorb;
        double Ib = pidConstsb.i * integralb;
        double Db = pidConstsb.d * derivativeb;

        backShoot.setVelocity(Pb + Ib + Db + targetVelocity);

        lastErrorb = errorb;
    }

    public void shoot() {
        sleep(100);
        shootFlicker.setPosition(0.1);
        sleep(100);
        shootFlicker.setPosition(0.45);
    }
    public void wobbleUp () {
        wobbleClawServo.setPosition(.9);
        sleep(500);
        wobbleArmServo.setPosition(.5);
    }
    public void wobbleDown () {
        wobbleArmServo.setPosition(.03);
        sleep(300);
        wobbleClawServo.setPosition(.5);
    }
}
