package org.firstinspires.ftc.teamcode.drive.auton.Red1;

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

@Autonomous(group = "R1")
public class OneRingRed1 extends LinearOpMode {
    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_MAX_RPM = 5400;
    public static double MOTOR_GEAR_RATIO = 1;

    public static boolean RUN_USING_ENCODER = true;
    public static boolean DEFAULT_GAINS = false;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private VoltageSensor batteryVoltageSensor;

    private Servo shooterStopper, wobbleArm1, wobbleArm2, shootFlicker, flap, turret, droptakeStopper, wobbleClaw;

    private DcMotor intake, bottomRoller;


    /********************************************************************************************
     *
     */

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(45, 0, 0, 25);
    public static PIDFCoefficients MOTOR_VELO_PID_2 = new PIDFCoefficients(45, 0, 0, 25); // fix this

    public static double lastKf = 16.7;
    public static double lastKf_2 = 16.7; // fix this

    /********************************************************************************************
     *
     */

    double lastVoltage = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MotorConfigurationType motorConfigurationType = shooter1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        shooter1.setMotorType(motorConfigurationType);

        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MotorConfigurationType motorConfigurationType2 = shooter2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        shooter2.setMotorType(motorConfigurationType2);

        if (RUN_USING_ENCODER)
            shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else
            shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (RUN_USING_ENCODER)
            shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else
            shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        setPIDFCoefficients(shooter1, MOTOR_VELO_PID);

        setPIDFCoefficients2(shooter1, MOTOR_VELO_PID_2);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        turret = hardwareMap.get(Servo.class, "turret");
        flap = hardwareMap.get(Servo.class, "flap");
        wobbleArm1 = hardwareMap.get(Servo.class, "wobbleArm1");
        wobbleArm2 = hardwareMap.get(Servo.class, "wobbleArm2");
        shootFlicker = hardwareMap.get(Servo.class, "shootFlicker");
        droptakeStopper = hardwareMap.get(Servo.class, "droptakeStopper");
        wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");
        shooterStopper = hardwareMap.get(Servo.class, "shooterStopper");

        intake = hardwareMap.dcMotor.get("intake");
        bottomRoller = hardwareMap.dcMotor.get("bottomRoller");

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.update();
        telemetry.clearAll();


        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        Pose2d startPose = new Pose2d(-50, -2, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
/**
 *
 */

        //wobbleClaw.setPosition();
        wobbleArm1.setPosition(0);
        wobbleArm2.setPosition(0);

        turret.setPosition(.15);
        flap.setPosition(.55);

        droptakeStopper.setPosition(.25);
        //shooterStopper.setPosition();

        shootFlicker.setPosition(.35);
        sleep(250);
        shootFlicker.setPosition(.57);

        sleep(10000);

        //wobbleClaw.setPosition();

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

            setPIDFCoefficients2(shooter1, MOTOR_VELO_PID_2);
            setPIDFCoefficients(shooter1, MOTOR_VELO_PID);

            lastVoltage = batteryVoltageSensor.getVoltage();

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));

//            Trajectory traj0 = drive.trajectoryBuilder(startPose)
//                    .addTemporalMarker(0, () -> {
//                        runShooterMotors(2000);
//                        telemetry.addData("power1", shooter1.getPower());
//                        telemetry.addData("velo1", shooter1.getVelocity());
//                        telemetry.update();
//                    })
//                    .lineToLinearHeading(new Pose2d(-25, -7, Math.toRadians(5)))
//                    .addDisplacementMarker(() -> {
//                        sleep(150);
//                        shootFlicker.setPosition(0.1);
//                        sleep(100);
//                        shootFlicker.setPosition(0.45);
//                        sleep(400);
//                        shootFlicker.setPosition(0.1);
//                        sleep(100);
//                        shootFlicker.setPosition(0.45);
//                        sleep(400);
//                        shootFlicker.setPosition(0.1);
//                        sleep(100);
//                        shootFlicker.setPosition(0.45);
//                        sleep(170);
//                        shooter1.setPower(0);
//                        shooter1.setPower(0);
//                        liftServo.setPosition(.72);
//                        sleep(450);
//
//                        intake1.setPower(.8);
//                        intake2.setPower(.8);
//                    })
//                    .build();
//
//            Trajectory traj01 = drive.trajectoryBuilder(traj0.end())
//                    .lineToLinearHeading(new Pose2d(0, -7, 0))
//                    .addDisplacementMarker(() -> {
//                        sleep(600);
//                        intake1.setPower(0);
//                        intake2.setPower(0);
//                        liftServo.setPosition(.17);
//                        sleep(450);
//                    })
//                    .build();
//            Trajectory traj02 = drive.trajectoryBuilder(traj01.end())
//                    .addTemporalMarker(0, () -> {
//                        runShooterMotors(0.1);
//                    })
//                    .lineToLinearHeading(new Pose2d(-25, -7, Math.toRadians(5)))
//                    .addDisplacementMarker(() -> {
//                        sleep(150);
//                        shootFlicker.setPosition(0.1);
//                        sleep(100);
//                        shootFlicker.setPosition(0.45);
//                        sleep(270);
//                        shooter1.setPower(0);
//                        shooter1.setPower(0);
//                        liftServo.setPosition(.72);
//                        sleep(450);
//                    })
//                    .build();
//
//            Trajectory traj1 = drive.trajectoryBuilder(traj01.end())
//                    .addTemporalMarker(0, () -> {
//                        runShooterMotors(2860);
//                    })
//                    .splineToConstantHeading(new Vector2d(3, 15), 0)
//                    .addDisplacementMarker(() -> {
//                        shootFlicker.setPosition(0.1);
//                        sleep(100);
//                        shootFlicker.setPosition(0.45);
//                    })
//                    .build();
//
//            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                    .lineToLinearHeading(new Pose2d(3, 22.5, Math.toRadians(0)))
//                    .addDisplacementMarker(() -> {
//                        sleep(100);
//                        shootFlicker.setPosition(0.1);
//                        sleep(100);
//                        shootFlicker.setPosition(0.45);
//                    })
//                    .build();
//
//            Trajectory traj3 = drive.trajectoryBuilder(traj02.end())
//                    .lineToLinearHeading(new Pose2d(3, 28, Math.toRadians(0)))
//                    .addDisplacementMarker(() -> {
//                        sleep(100);
//                        shootFlicker.setPosition(0.1);
//                        sleep(100);
//                        shootFlicker.setPosition(0.45);
//                        sleep(150);
//                    })
//                    .build();
//
//            Trajectory traj4 = drive.trajectoryBuilder(traj02.end())
//                    .addTemporalMarker(0, () -> {
//                        shooter1.setPower(0);
//                        shooter1.setPower(0);
//                        liftServo.setPosition(.72);
//                    })
//                    .lineToLinearHeading(new Pose2d(50, 12, Math.toRadians(90)))
//                    .addDisplacementMarker(() -> {
//                        sleep(100);
//                        wobbleDown();
//                        wobbleArmServo.setPosition(.3);
//                        sleep(700);
//                        wobbleClawServo.setPosition(.5);
//                        sleep(350);
//                    //})
//                    .build();
//
//            Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
//                    .lineToLinearHeading(new Pose2d(-26.5, -12, Math.toRadians(0)))
//                    .addTemporalMarker(1.5, () -> {
//                        wobbleArmServo.setPosition(0);
//                    })
//                    .addDisplacementMarker(() -> {
//                        wobbleUp();
//                    })
//                    .build();
//
//            Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
//                    .lineToLinearHeading(new Pose2d(42, 17, Math.toRadians(90)))
//                    .addDisplacementMarker(() -> {
//                        wobbleDown();
//                    })
//                    .build();
//
//            Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
//                    .lineToLinearHeading(new Pose2d(42, 20, Math.toRadians(90)))
//                    .build();
//            Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
//                    .lineToLinearHeading(new Pose2d(24, 20, Math.toRadians(0)))
//                    .build();
//
//            drive.followTrajectory(traj0);
//            //high goal
//            sleep(700);
//            drive.followTrajectory(traj01);
//            sleep(200);
//            //picking up ring
//            drive.followTrajectory(traj02);
//            drive.followTrajectory(traj1);
//            //sleep(400);
//            //shoot();
//            drive.followTrajectory(traj2);
//            //shoot();
//            drive.followTrajectory(traj3);
//            //shoot();
//
//            drive.followTrajectory(traj4);
//            wobbleArmServo.setPosition(.5);
//            sleep(100);
//            //wobble goal
//            sleep(300);
//            drive.followTrajectory(traj5);
//            //picking up 2nd wobble goal
//            drive.followTrajectory(traj6);
//            wobbleArmServo.setPosition(.5);
//            sleep(100);
//            //dropping off the 2nd wobble goal
//            drive.followTrajectory(traj7);
//            drive.followTrajectory(traj8);
//            //white line
//
//            // Transfer the current pose to PoseStorage so we can use it in TeleOp
//            PoseStorage.currentPose = drive.getPoseEstimate();
//            sleep(10000);
//
//            //break;
//            Trajectory traj1 = drive.trajectoryBuilder(startPose)
//                    //.splineToConstantHeading(new Vector2d(-45, -2), 0)
//                    .addTemporalMarker(0, () -> {
//                        setVelocity(shooter1, 2700);
//                        setVelocity(shooter1, 2700);
//                    })
//                    .splineToConstantHeading(new Vector2d(3, 13), 0)
//                    .addDisplacementMarker(() -> {
//                        sleep(100);
//                        shootFlicker.setPosition(0.45);
//                        sleep(170);
//                        shootFlicker.setPosition(0.1);
//                    })
//                    .build();
//
//            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                    .lineToLinearHeading(new Pose2d(3, 21.5, Math.toRadians(0)))
//                    .addDisplacementMarker(() -> {
//                        sleep(100);
//                        shootFlicker.setPosition(0.45);
//                        sleep(170);
//                        shootFlicker.setPosition(0.1);
//                    })
//                    .build();
//
//            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                    .lineToLinearHeading(new Pose2d(3, 29, Math.toRadians(0)))
//                    .addDisplacementMarker(() -> {
//                        sleep(100);
//                        shootFlicker.setPosition(0.45);
//                        sleep(170);
//                        shootFlicker.setPosition(0.1);
//                        sleep(150);
//                    })
//                    .build();
//            Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                    .addTemporalMarker(0, () -> {
//                        shooter1.setPower(0);
//                        shooter1.setPower(0);
//                        //liftServo.setPosition(.63);
//                    })
//                    .lineToLinearHeading(new Pose2d(44, 0, Math.toRadians(270))) //change pose
//                    .addDisplacementMarker(() -> {
//                        sleep(250);
//                        wobbleArm1.setPosition(.44);
//                        sleep(900);
//                        wobbleClaw1.setPosition(.51); //need to change position and time
//                        sleep(500);
//                        //wobbleUp();
//                        wobbleArmServo.setPosition(.3);
//                        sleep(700);
//                        wobbleClawServo.setPosition(.5);
//                        sleep(350);
//                    })
//                    .build();
//            Trajectory traj5_0 = drive.trajectoryBuilder(traj4.end())
//                    .addTemporalMarker(1.5, () -> {
//                        wobbleDown();
//            })
//                    .lineToLinearHeading(new Pose2d(44, 4, Math.toRadians(270)))
//                    //.lineToLinearHeading(new Pose2d(-26.6, -12, Math.toRadians(0)))
//                    .addTemporalMarker(1.5, () -> {
//                        wobbleArmServo.setPosition(0);
//                    })
//                    .build();
//
//            Trajectory traj5 = drive.trajectoryBuilder(traj5_0.end())
//                    .lineToLinearHeading(new Pose2d(-26.6, -17, Math.toRadians(180)))
//                    //.lineToLinearHeading(new Pose2d(-26.6, -12, Math.toRadians(0)))
//                    .addTemporalMarker(1.5, () -> {
//                        wobbleArmServo.setPosition(0);
//                    })
//                    .addDisplacementMarker(() -> {
//                        sleep(250);
//                        wobbleUp();
//                    })
//                    .build();
//
//            Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
//                    .lineToLinearHeading(new Pose2d(32, 12, Math.toRadians(270))) //change pose
//                    .addDisplacementMarker(() -> {
//                        sleep(250);
//                        wobbleArmServo.setPosition(.44);
//                        sleep(350);
//                        wobbleClawServo.setPosition(.51); //need to change position and time
//                        sleep(200);
//                    })
//                    .build();
//
//            Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
//                    .lineToLinearHeading(new Pose2d(24, 8, 0))
//                    .addDisplacementMarker(() -> {
//                        sleep(250);
//                        wobbleUp();
//                    })
//                    .build();
//
//        drive.followTrajectory(traj1);
//        //sleep(400);
//        //shoot();
//        drive.followTrajectory(traj2);
//        //shoot();
//        drive.followTrajectory(traj3);
//        //shoot();
//        //
//        drive.followTrajectory(traj4);
//            /*wobbleArmServo.setPosition(.5);
//            sleep(100);*/
//        //wobble goal
//        //sleep(300);
//            drive.followTrajectory(traj5_0);
//        drive.followTrajectory(traj5);
//        //picking up 2nd wobble goal
//        drive.followTrajectory(traj6);
//        //dropping off the 2nd wobble goal
//        drive.followTrajectory(traj7);
//        //white line
//        //58, 63 2 x wobble 0 15,8 y wobble 0
//        //82, 88 x wobble 1 10, 18 7wobble 1
//
//        // Transfer the current pose to PoseStorage so we can use it in TeleOp
//        PoseStorage.currentPose = drive.getPoseEstimate();
//        break;
        }
    }
    public void shoot() {
        shootFlicker.setPosition(0.35);
        sleep(280);
        shootFlicker.setPosition(0.57);
    }
    public void wobbleUp () {
        //wobbleClaw.setPosition(.07); // need to change position and time
        sleep(700);
        wobbleArm1.setPosition(.2);
        wobbleArm2.setPosition(.2);
        sleep(500);
    }
    public void wobbleDown () {
        wobbleArm1.setPosition(.54);
        wobbleArm2.setPosition(.54);
        sleep(500);
        //wobbleClaw.setPosition(.07); // need to change position and time
        sleep(350);
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

//    public void runShooterMotors(double targetVelocity) {
//        setVelocity(shooter1, targetVelocity);
//        setVelocity(shooter1, targetVelocity);
//    }

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

