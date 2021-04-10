package org.firstinspires.ftc.teamcode.drive.auton;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;

import java.util.Arrays;

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
    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_MAX_RPM = 5400;
    public static double MOTOR_GEAR_RATIO = 1;

    public static boolean RUN_USING_ENCODER = true;
    public static boolean DEFAULT_GAINS = false;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private VoltageSensor batteryVoltageSensor;

    //private DcMotorEx frontShoot, backShoot;
    private Servo wobbleClawServo, wobbleArmServo;
    private Servo /*liftServo,*/ shootFlicker;
    //private DcMotor intake1, intake2;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(45, 0, 0, 25);
    public static PIDFCoefficients MOTOR_VELO_PID_2 = new PIDFCoefficients(45, 0, 0, 25); // fix this

    public static double lastKf = 16.7;
    public static double lastKf_2 = 16.7; // fix this

    double lastVoltage = 0;

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private DcMotorEx frontShoot, backShoot;

    @Override
    public void runOpMode() throws InterruptedException {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        DcMotorEx frontShoot = hardwareMap.get(DcMotorEx.class, "frontShoot");
        frontShoot.setDirection(DcMotorSimple.Direction.REVERSE);
        frontShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MotorConfigurationType motorConfigurationType = frontShoot.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        frontShoot.setMotorType(motorConfigurationType);

        DcMotorEx backShoot = hardwareMap.get(DcMotorEx.class, "backShoot");
        backShoot.setDirection(DcMotorSimple.Direction.REVERSE);
        backShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MotorConfigurationType motorConfigurationType2 = backShoot.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        backShoot.setMotorType(motorConfigurationType2);

        if (RUN_USING_ENCODER)
            frontShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else
            frontShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (RUN_USING_ENCODER)
            backShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else
            backShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shootFlicker = hardwareMap.get(Servo.class, "shootFlicker");

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        /*frontShoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,
                MOTOR_VELO_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));*/
        setPIDFCoefficients(frontShoot, MOTOR_VELO_PID);

        /*backShoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID_2.p, MOTOR_VELO_PID_2.i, MOTOR_VELO_PID_2.d,
                MOTOR_VELO_PID_2.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));*/
        setPIDFCoefficients2(backShoot, MOTOR_VELO_PID_2);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.update();
        telemetry.clearAll();

        /*intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");
        */
        //liftServo = hardwareMap.servo.get("liftServo");
        wobbleClawServo = hardwareMap.servo.get("wobbleClawServo");
        wobbleArmServo = hardwareMap.servo.get("wobbleArmServo");
        shootFlicker = hardwareMap.servo.get("shootFlicker");
        /*
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        */

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        Pose2d startPose = new Pose2d(-50, -2, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        sleep(200);
        shootFlicker.setPosition(0.4);
        sleep(100);
        shootFlicker.setPosition(0.1);

        wobbleArmServo.setPosition(1);
        sleep(5000);
        wobbleClawServo.setPosition(.07); // need to change

        //liftServo.setPosition(.17);

        waitForStart();

        if (isStopRequested()) return;
        while (/*opModeIsActive() && */!isStopRequested()) {

            if (lastKf_2 != MOTOR_VELO_PID_2.f) {
                MOTOR_VELO_PID_2.f = lastKf_2 * 12 / batteryVoltageSensor.getVoltage();
                lastKf_2 = MOTOR_VELO_PID_2.f;
            }

            if (lastKf != MOTOR_VELO_PID.f) {
                MOTOR_VELO_PID.f = lastKf * 12 / batteryVoltageSensor.getVoltage();
                lastKf = MOTOR_VELO_PID.f;
            }

            setPIDFCoefficients2(backShoot, MOTOR_VELO_PID_2);
            setPIDFCoefficients(frontShoot, MOTOR_VELO_PID);

            lastVoltage = batteryVoltageSensor.getVoltage();

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));

            /*Trajectory traj0 = drive.trajectoryBuilder(startPose)
                    .addTemporalMarker(0, () -> {
                        runShooterMotors(2000);
                        telemetry.addData("power1", frontShoot.getPower());
                        telemetry.addData("velo1", frontShoot.getVelocity());
                        telemetry.update();
                    })
                    .lineToLinearHeading(new Pose2d(-25, -7, Math.toRadians(5)))
                    .addDisplacementMarker(() -> {
                        sleep(150);
                        shootFlicker.setPosition(0.1);
                        sleep(100);
                        shootFlicker.setPosition(0.45);
                        sleep(400);
                        shootFlicker.setPosition(0.1);
                        sleep(100);
                        shootFlicker.setPosition(0.45);
                        sleep(400);
                        shootFlicker.setPosition(0.1);
                        sleep(100);
                        shootFlicker.setPosition(0.45);
                        sleep(170);
                        frontShoot.setPower(0);
                        backShoot.setPower(0);
                        liftServo.setPosition(.72);
                        sleep(450);

                        intake1.setPower(.8);
                        intake2.setPower(.8);
                    })
                    .build();

            Trajectory traj01 = drive.trajectoryBuilder(traj0.end())
                    .lineToLinearHeading(new Pose2d(0, -7, 0))
                    .addDisplacementMarker(() -> {
                        sleep(600);
                        intake1.setPower(0);
                        intake2.setPower(0);
                        liftServo.setPosition(.17);
                        sleep(450);
                    })
                    .build();
            Trajectory traj02 = drive.trajectoryBuilder(traj01.end())
                    .addTemporalMarker(0, () -> {
                        runShooterMotors(0.1);
                    })
                    .lineToLinearHeading(new Pose2d(-25, -7, Math.toRadians(5)))
                    .addDisplacementMarker(() -> {
                        sleep(150);
                        shootFlicker.setPosition(0.1);
                        sleep(100);
                        shootFlicker.setPosition(0.45);
                        sleep(270);
                        frontShoot.setPower(0);
                        backShoot.setPower(0);
                        liftServo.setPosition(.72);
                        sleep(450);
                    })
                    .build();

            /*Trajectory traj1 = drive.trajectoryBuilder(traj01.end())
                    .addTemporalMarker(0, () -> {
                        runShooterMotors(2860);
                    })
                    .splineToConstantHeading(new Vector2d(3, 15), 0)
                    .addDisplacementMarker(() -> {
                        shootFlicker.setPosition(0.1);
                        sleep(100);
                        shootFlicker.setPosition(0.45);
                    })
                    .build();

            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                    .lineToLinearHeading(new Pose2d(3, 22.5, Math.toRadians(0)))
                    .addDisplacementMarker(() -> {
                        sleep(100);
                        shootFlicker.setPosition(0.1);
                        sleep(100);
                        shootFlicker.setPosition(0.45);
                    })
                    .build();

            Trajectory traj3 = drive.trajectoryBuilder(traj02.end())
                    .lineToLinearHeading(new Pose2d(3, 28, Math.toRadians(0)))
                    .addDisplacementMarker(() -> {
                        sleep(100);
                        shootFlicker.setPosition(0.1);
                        sleep(100);
                        shootFlicker.setPosition(0.45);
                        sleep(150);
                    })
                    .build();

            Trajectory traj4 = drive.trajectoryBuilder(traj02.end())
                    .addTemporalMarker(0, () -> {
                        frontShoot.setPower(0);
                        backShoot.setPower(0);
                        liftServo.setPosition(.72);
                    })
                    .lineToLinearHeading(new Pose2d(50, 12, Math.toRadians(90)))
                    .addDisplacementMarker(() -> {
                        sleep(100);
                        wobbleDown();
                        /*wobbleArmServo.setPosition(.3);
                        sleep(700);
                        wobbleClawServo.setPosition(.5);
                        sleep(350);*/
                    //})
                    /*.build();

            Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                    .lineToLinearHeading(new Pose2d(-26.5, -12, Math.toRadians(0)))
                    .addTemporalMarker(1.5, () -> {
                        wobbleArmServo.setPosition(0);
                    })
                    .addDisplacementMarker(() -> {
                        wobbleUp();
                    })
                    .build();

            Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                    .lineToLinearHeading(new Pose2d(42, 17, Math.toRadians(90)))
                    .addDisplacementMarker(() -> {
                        wobbleDown();
                    })
                    .build();

            Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                    .lineToLinearHeading(new Pose2d(42, 20, Math.toRadians(90)))
                    .build();
            Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                    .lineToLinearHeading(new Pose2d(24, 20, Math.toRadians(0)))
                    .build();

            drive.followTrajectory(traj0);
            //high goal
            sleep(700);
            drive.followTrajectory(traj01);
            sleep(200);
            //picking up ring
            drive.followTrajectory(traj02);
            /*drive.followTrajectory(traj1);
            //sleep(400);
            //shoot();
            drive.followTrajectory(traj2);
            //shoot();
            drive.followTrajectory(traj3);
            //shoot();
            */
            /*drive.followTrajectory(traj4);
            wobbleArmServo.setPosition(.5);
            sleep(100);
            //wobble goal
            sleep(300);
            drive.followTrajectory(traj5);
            //picking up 2nd wobble goal
            drive.followTrajectory(traj6);
            wobbleArmServo.setPosition(.5);
            sleep(100);
            //dropping off the 2nd wobble goal
            drive.followTrajectory(traj7);
            drive.followTrajectory(traj8);
            //white line

            // Transfer the current pose to PoseStorage so we can use it in TeleOp
            PoseStorage.currentPose = drive.getPoseEstimate();
            sleep(10000);

            //break;*/
            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    //.splineToConstantHeading(new Vector2d(-45, -2), 0)
                    .addTemporalMarker(0, () -> {
                        setVelocity(frontShoot, 2700);
                        setVelocity(backShoot, 2700);
                    })
                    .splineToConstantHeading(new Vector2d(3, 13), 0)
                    .addDisplacementMarker(() -> {
                        sleep(100);
                        shootFlicker.setPosition(0.45);
                        sleep(170);
                        shootFlicker.setPosition(0.1);
                    })
                    .build();

            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                    .lineToLinearHeading(new Pose2d(3, 21.5, Math.toRadians(0)))
                    .addDisplacementMarker(() -> {
                        sleep(100);
                        shootFlicker.setPosition(0.45);
                        sleep(170);
                        shootFlicker.setPosition(0.1);
                    })
                    .build();

            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .lineToLinearHeading(new Pose2d(3, 29, Math.toRadians(0)))
                    .addDisplacementMarker(() -> {
                        sleep(100);
                        shootFlicker.setPosition(0.45);
                        sleep(170);
                        shootFlicker.setPosition(0.1);
                        sleep(150);
                    })
                    .build();
            Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                    .addTemporalMarker(0, () -> {
                        frontShoot.setPower(0);
                        backShoot.setPower(0);
                        //liftServo.setPosition(.63);
                    })
                    .lineToLinearHeading(new Pose2d(44, 0, Math.toRadians(270))) //change pose
                    .addDisplacementMarker(() -> {
                        sleep(250);
                        wobbleArmServo.setPosition(.44);
                        sleep(900);
                        wobbleClawServo.setPosition(.51); //need to change position and time
                        sleep(500);
                        //wobbleUp();
                        /*wobbleArmServo.setPosition(.3);
                        sleep(700);
                        wobbleClawServo.setPosition(.5);
                        sleep(350);*/
                    })
                    .build();
            Trajectory traj5_0 = drive.trajectoryBuilder(traj4.end())
                    /*.addTemporalMarker(1.5, () -> {
                        wobbleDown();
            })*/
                    .lineToLinearHeading(new Pose2d(44, 4, Math.toRadians(270)))
                    //.lineToLinearHeading(new Pose2d(-26.6, -12, Math.toRadians(0)))
                    /*.addTemporalMarker(1.5, () -> {
                        wobbleArmServo.setPosition(0);
                    })*/
                    .build();

            Trajectory traj5 = drive.trajectoryBuilder(traj5_0.end())
                    .lineToLinearHeading(new Pose2d(-26.6, -17, Math.toRadians(180)))
                    //.lineToLinearHeading(new Pose2d(-26.6, -12, Math.toRadians(0)))
                    /*.addTemporalMarker(1.5, () -> {
                        wobbleArmServo.setPosition(0);
                    })*/
                    .addDisplacementMarker(() -> {
                        sleep(250);
                        wobbleUp();
                    })
                    .build();

            Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                    .lineToLinearHeading(new Pose2d(32, 12, Math.toRadians(270))) //change pose
                    .addDisplacementMarker(() -> {
                        sleep(250);
                        wobbleArmServo.setPosition(.44);
                        sleep(350);
                        wobbleClawServo.setPosition(.51); //need to change position and time
                        sleep(200);
                    })
                    .build();

            Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                    .lineToLinearHeading(new Pose2d(24, 8, 0))
                    .addDisplacementMarker(() -> {
                        sleep(250);
                        wobbleUp();
                    })
                    .build();

        drive.followTrajectory(traj1);
        //sleep(400);
        //shoot();
        drive.followTrajectory(traj2);
        //shoot();
        drive.followTrajectory(traj3);
        //shoot();
        //
        drive.followTrajectory(traj4);
            /*wobbleArmServo.setPosition(.5);
            sleep(100);*/
        //wobble goal
        //sleep(300);
            drive.followTrajectory(traj5_0);
        drive.followTrajectory(traj5);
        //picking up 2nd wobble goal
        drive.followTrajectory(traj6);
        //dropping off the 2nd wobble goal
        drive.followTrajectory(traj7);
        //white line
        //58, 63 2 x wobble 0 15,8 y wobble 0
        //82, 88 x wobble 1 10, 18 7wobble 1

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();
        break;
        }
    }
    public void shoot() {
        shootFlicker.setPosition(0.4);
        sleep(100);
        shootFlicker.setPosition(0.1);
    }
    public void wobbleUp () {
        wobbleClawServo.setPosition(.07); // need to change position and time
        sleep(700);
        wobbleArmServo.setPosition(.8);
    }
    public void wobbleDown () {
        wobbleArmServo.setPosition(.44);
        wobbleClawServo.setPosition(.51); //need to change position and time
        sleep(1200);
    }

    public void setVelocity(DcMotorEx motor, double power) {
        if(RUN_USING_ENCODER) {
            motor.setVelocity(rpmToTicksPerSecond(power));
            Log.i("mode", "setting velocity");
        }
        else {
            Log.i("mode", "setting power");
            motor.setPower(power / MOTOR_MAX_RPM);
        }
    }

    public void runShooterMotors(double targetVelocity) {
        setVelocity(frontShoot, targetVelocity);
        setVelocity(backShoot, targetVelocity);
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        if(!RUN_USING_ENCODER) {
            Log.i("config", "skipping RUE");
            return;
        }

        if (!DEFAULT_GAINS) {
            Log.i("config", "setting custom gains");
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
            ));
        } else {
            Log.i("config", "setting default gains");
        }
    }
    private void setPIDFCoefficients2(DcMotorEx motor, PIDFCoefficients coefficients) {
        if(!RUN_USING_ENCODER) {
            Log.i("config", "skipping RUE");
            return;
        }

        if (!DEFAULT_GAINS) {
            Log.i("config", "setting custom gains");
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
            ));
        } else {
            Log.i("config", "setting default gains");
        }
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }
}

