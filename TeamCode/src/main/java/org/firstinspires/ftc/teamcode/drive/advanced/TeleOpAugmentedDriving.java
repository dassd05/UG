package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.*;

/**
 * This opmode demonstrates how one can augment driver control by following Road Runner arbitrary
 * Road Runner trajectories at any time during teleop. This really isn't recommended at all. This is
 * not what Trajectories are meant for. A path follower is more suited for this scenario. This
 * sample primarily serves as a demo showcasing Road Runner's capabilities.
 * <p>
 * This bot starts in driver controlled mode by default. The player is able to drive the bot around
 * like any teleop opmode. However, if one of the select buttons are pressed, the bot will switch
 * to automatic control and run to specified location on its own.
 * <p>
 * If A is pressed, the bot will generate a splineTo() trajectory on the fly and follow it to
 * targetA (x: 45, y: 45, heading: 90deg).
 * <p>
 * If B is pressed, the bot will generate a lineTo() trajectory on the fly and follow it to
 * targetB (x: -15, y: 25, heading: whatever the heading is when you press B).
 * <p>
 * If Y is pressed, the bot will turn to face 45 degrees, no matter its position on the field.
 * <p>
 * Pressing X will cancel trajectory following and switch control to the driver. The bot will also
 * cede control to the driver once trajectory following is done.
 * <p>
 * The following may be a little off with this method as the trajectory follower and turn
 * function assume the bot starts at rest.
 * <p>
 * This sample utilizes the SampleMecanumDriveCancelable.java class.
 */

@TeleOp(group = "advanced")
public class TeleOpAugmentedDriving extends LinearOpMode {
    enum Mode {
        INTAKING,
        TELEOP_SHOOTING,
        ENDGAME
    }

    Mode currentMode = Mode.INTAKING;

    private DcMotorEx frontShoot, backShoot;
    private Servo wobbleClawServo, wobbleArmServo;
    private Servo liftServo, shootFlicker;
    private DcMotor intake1, intake2;

    double integralf = 0;
    double integralb = 0;

    public static PIDCoefficients pidConstsf = new PIDCoefficients(0.5, 0, 25.0);
    public static PIDCoefficients pidConstsb = new PIDCoefficients(0.45, 0, 35.0);

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

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading() - Math.toRadians(90));

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() * .7,
                            input.getY() * .7,
                            (-gamepad1.right_stick_x * .7)
                    )
            );

            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));

            switch (currentMode) {
                case INTAKING:
                    frontShoot.setPower(0);
                    backShoot.setPower(0);

                    liftServo.setPosition(.58);

                    intake1.setPower(.65);
                    intake2.setPower(.65);
                    break;
                case TELEOP_SHOOTING:

                    Trajectory shooting = drive.trajectoryBuilder(poseEstimate)
                            .lineToLinearHeading(new Pose2d(20, 20, Math.toRadians(0))) // need to fix coordinate
                            .addTemporalMarker(0, () -> {
                                intake1.setPower(0);
                                intake2.setPower(0);
                                liftServo.setPosition(.08);

                                runShooterMotors(1200); //need to test shooter power stuff
                            })
                            .build(); //need to fix the coordinates

                    drive.followTrajectory(shooting);
                    //runShooterMotors(1200);

                    if (gamepad1.x && drive.isBusy() || gamepad1.a && drive.isBusy())
                        drive.cancelFollowing();
                    break;
                case ENDGAME:
                    Trajectory endgame1 = drive.trajectoryBuilder(poseEstimate)
                            .strafeRight(24) // need to fix coordinate
                            .addTemporalMarker(0, () -> {
                                intake1.setPower(0);
                                intake2.setPower(0);
                                liftServo.setPosition(.08);

                                runShooterMotors(1200); //need to test shooter power stuff
                            })
                            .build(); //need to fix the coordinates

                    Trajectory endgame2 = drive.trajectoryBuilder(endgame1.end())
                            .strafeRight(10) // need to fix coordinate
                            .addTemporalMarker(0, () -> {
                            })
                            .build(); //need to fix the coordinates

                    Trajectory endgame3 = drive.trajectoryBuilder(endgame2.end())
                            .strafeRight(10) // need to fix coordinate
                            .addTemporalMarker(0, () -> {
                            })
                            .build(); //need to fix the coordinates

                    drive.followTrajectory(endgame1);
                    sleep(100);
                    shoot();
                    sleep(200);

                    drive.followTrajectory(endgame2);
                    sleep(100);
                    shoot();
                    sleep(200);

                    drive.followTrajectory(endgame3);
                    sleep(100);
                    shoot();
                    sleep(200);

                    currentMode = Mode.INTAKING;

                    if (gamepad1.x && drive.isBusy() || gamepad1.y && drive.isBusy())
                        drive.cancelFollowing();
                    break;
                default:
                    // this should *NEVER* happen
                    String errMsg = "LINE 49, 126, 135, switch statement on currentMode reached default case somehow";
                    telemetry.addData("ERROR", errMsg);
                    telemetry.update();
                    throw new InternalError(errMsg);
            }

            if (gamepad1.x) currentMode = Mode.INTAKING;
            if (gamepad1.y) currentMode = Mode.TELEOP_SHOOTING;
            if (gamepad1.a) currentMode = Mode.ENDGAME;

            if (gamepad1.b) {
                shoot();
            }

            // lifting wobble goal
            if (gamepad1.dpad_up) {
                wobbleClawServo.setPosition(.8);
                sleep(700);
                wobbleArmServo.setPosition(.8);
            }

            // setting arm down
            if (gamepad1.dpad_down) {
                wobbleArmServo.setPosition(.3);
                wobbleClawServo.setPosition(.3);
            }

            // dropping off wobble goal
            if (gamepad1.dpad_right) {
                wobbleArmServo.setPosition(.5);
                sleep(1000);
                wobbleClawServo.setPosition(.3);
            }

            telemetry.update();
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
}
