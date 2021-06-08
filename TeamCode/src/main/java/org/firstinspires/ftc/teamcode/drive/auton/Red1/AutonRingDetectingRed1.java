package org.firstinspires.ftc.teamcode.drive.auton.Red1;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.teamcode.drive.auton.Red1.AutonRingDetectingRed1.RingDetecting.pipeline;

@Autonomous(name = "AutonRingDetecting", group = "R1")
public class AutonRingDetectingRed1 extends LinearOpMode {

    FourRingRed1 FourR1 = new FourRingRed1();

    protected WebcamName webcamName;
    protected OpenCvWebcam webcam;

    public enum ThisManyRings {
        FourRings,
        OneRing,
        ZeroRings,
        Default
    }

    public volatile ThisManyRings HowManyRings;

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
    public void runOpMode() throws InterruptedException{

        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
            }
        });

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

        while (!opModeIsActive()) {
            if (pipeline.position == null) {
                telemetry.addData("still working on it", "gimme a sec");
                HowManyRings = ThisManyRings.Default;
            } else if (pipeline.position == RingDetecting.RingPosition.FOUR){
                telemetry.addData("Four Rings", "Waiting for start");
                HowManyRings = ThisManyRings.FourRings;
                telemetry.update();
            }
            else if (pipeline.position == RingDetecting.RingPosition.ONE){
                telemetry.addData("One Ring", "Waiting for start");
                HowManyRings = ThisManyRings.OneRing;
            }
            else if (pipeline.position == RingDetecting.RingPosition.NONE){
                telemetry.addData("Zero Rings", "Waiting for start");
                HowManyRings = ThisManyRings.ZeroRings;
            }
            telemetry.update();
        }



        waitForStart();

        if (opModeIsActive()){
            while (opModeIsActive()) {
//                if (lastKf_2 != MOTOR_VELO_PID_2.f) {
//                    MOTOR_VELO_PID_2.f = lastKf_2 * 12 / batteryVoltageSensor.getVoltage();
//                    lastKf_2 = MOTOR_VELO_PID_2.f;
//                }
//
//                if (lastKf != MOTOR_VELO_PID.f) {
//                    MOTOR_VELO_PID.f = lastKf * 12 / batteryVoltageSensor.getVoltage();
//                    lastKf = MOTOR_VELO_PID.f;
//                }
//
//                setPIDFCoefficients2(backShoot, MOTOR_VELO_PID_2);
//                setPIDFCoefficients(frontShoot, MOTOR_VELO_PID);
//
//                lastVoltage = batteryVoltageSensor.getVoltage();
//
//                drive.update();
//
//                Pose2d poseEstimate = drive.getPoseEstimate();

//                switch (HowManyRings) {
//                    case FourRings:
//                        telemetry.addData("4 rings detected", "wobble position C");
//                        //autonFourRings();
                FourR1.runOpMode();
//                        //runFinished = true;
//                        Trajectory traj1_4 = drive.trajectoryBuilder(startPose)
//                                //.splineToConstantHeading(new Vector2d(-45, -2), 0)
//                                .addTemporalMarker(0, () -> {
//                                    setVelocity(frontShoot, 2700);
//                                    setVelocity(backShoot, 2700);
//                                })
//                                .splineToConstantHeading(new Vector2d(3, 13), 0)
//                                .addDisplacementMarker(() -> {
//                                    sleep(100);
//                                    shootFlicker.setPosition(0.45);
//                                    sleep(170);
//                                    shootFlicker.setPosition(0.1);
//                                })
//                                .build();
//
//                        Trajectory traj2_4 = drive.trajectoryBuilder(traj1_4.end())
//                                .lineToLinearHeading(new Pose2d(3, 21.5, Math.toRadians(0)))
//                                .addDisplacementMarker(() -> {
//                                    sleep(100);
//                                    shootFlicker.setPosition(0.45);
//                                    sleep(170);
//                                    shootFlicker.setPosition(0.1);
//                                })
//                                .build();
//
//                        Trajectory traj3_4 = drive.trajectoryBuilder(traj2_4.end())
//                                .lineToLinearHeading(new Pose2d(3, 29, Math.toRadians(0)))
//                                .addDisplacementMarker(() -> {
//                                    sleep(100);
//                                    shootFlicker.setPosition(0.45);
//                                    sleep(170);
//                                    shootFlicker.setPosition(0.1);
//                                    sleep(150);
//                                })
//                                .build();
//                        Trajectory traj4_4 = drive.trajectoryBuilder(traj3_4.end())
//                                .addTemporalMarker(0, () -> {
//                                    frontShoot.setPower(0);
//                                    backShoot.setPower(0);
//                                    //liftServo.setPosition(.63);
//                                })
//                                .lineToLinearHeading(new Pose2d(65, -25, Math.toRadians(270))) //change pose
//                                .addDisplacementMarker(() -> {
//                                    sleep(250);
//                                    wobbleArmServo.setPosition(.44);
//                                    sleep(800);
//                                    wobbleClawServo.setPosition(.51); //need to change position and time
//                                    sleep(500);
//                                    //wobbleUp();
//                        /*wobbleArmServo.setPosition(.3);
//                        sleep(700);
//                        wobbleClawServo.setPosition(.5);
//                        sleep(350);*/
//                                })
//                                .build();
//                        Trajectory traj5_0_4 = drive.trajectoryBuilder(traj4_4.end())
//                                /*.addTemporalMarker(1.5, () -> {
//                                    wobbleDown();
//                        })*/
//                                .lineToLinearHeading(new Pose2d(65, -20, Math.toRadians(270)))
//                                //.lineToLinearHeading(new Pose2d(-26.6, -12, Math.toRadians(0)))
//                                /*.addTemporalMarker(1.5, () -> {
//                                    wobbleArmServo.setPosition(0);
//                                })*/
//                                .build();
//
//                        Trajectory traj5_4 = drive.trajectoryBuilder(traj5_0_4.end())
//                                .lineToLinearHeading(new Pose2d(-27, -17, Math.toRadians(180)))
//                                //.lineToLinearHeading(new Pose2d(-26.6, -12, Math.toRadians(0)))
//                                /*.addTemporalMarker(1.5, () -> {
//                                    wobbleArmServo.setPosition(0);
//                                })*/
//                                .addDisplacementMarker(() -> {
//                                    sleep(250);
//                                    wobbleUp();
//                                })
//                                .build();
//
//                        Trajectory traj6_4 = drive.trajectoryBuilder(traj5_4.end())
//                                .lineToLinearHeading(new Pose2d(64, -10, Math.toRadians(270))) //change pose
//                                .addDisplacementMarker(() -> {
//                                    sleep(250);
//                                    wobbleArmServo.setPosition(.44);
//                                    sleep(350);
//                                    wobbleClawServo.setPosition(.51); //need to change position and time
//                                    sleep(200);
//                                })
//                                .build();
//
//                        Trajectory traj7_4 = drive.trajectoryBuilder(traj6_4.end())
//                                .lineToLinearHeading(new Pose2d(24, 8, 0))
//                                .addDisplacementMarker(() -> {
//                                    sleep(250);
//                                    wobbleUp();
//                                })
//                                .build();
//
//                        drive.followTrajectory(traj1_4);
//                        //sleep(400);
//                        //shoot();
//                        drive.followTrajectory(traj2_4);
//                        //shoot();
//                        drive.followTrajectory(traj3_4);
//                        //shoot();
//                        //
//                        drive.followTrajectory(traj4_4);
//            /*wobbleArmServo.setPosition(.5);
//            sleep(100);*/
//                        //wobble goal
//                        //sleep(300);
//                        drive.followTrajectory(traj5_0_4);
//                        drive.followTrajectory(traj5_4);
//                        //picking up 2nd wobble goal
//                        drive.followTrajectory(traj6_4);
//                        //dropping off the 2nd wobble goal
//                        drive.followTrajectory(traj7_4);
//                        //white line
//                        //58, 63 2 x wobble 0 15,8 y wobble 0
//                        //82, 88 x wobble 1 10, 18 7wobble 1
//
//                        // Transfer the current pose to PoseStorage so we can use it in TeleOp
//                        PoseStorage.currentPose = drive.getPoseEstimate();
//                        break;
//                    case OneRing:
//                        Trajectory traj1_1 = drive.trajectoryBuilder(startPose)
//                                //.splineToConstantHeading(new Vector2d(-45, -2), 0)
//                                .addTemporalMarker(0, () -> {
//                                    setVelocity(frontShoot, 2700);
//                                    setVelocity(backShoot, 2700);
//                                })
//                                .splineToConstantHeading(new Vector2d(3, 13), 0)
//                                .addDisplacementMarker(() -> {
//                                    sleep(100);
//                                    shootFlicker.setPosition(0.45);
//                                    sleep(170);
//                                    shootFlicker.setPosition(0.1);
//                                })
//                                .build();
//
//                        Trajectory traj2_1 = drive.trajectoryBuilder(traj1_1.end())
//                                .lineToLinearHeading(new Pose2d(3, 21.5, Math.toRadians(0)))
//                                .addDisplacementMarker(() -> {
//                                    sleep(100);
//                                    shootFlicker.setPosition(0.45);
//                                    sleep(170);
//                                    shootFlicker.setPosition(0.1);
//                                })
//                                .build();
//
//                        Trajectory traj3_1 = drive.trajectoryBuilder(traj2_1.end())
//                                .lineToLinearHeading(new Pose2d(3, 29, Math.toRadians(0)))
//                                .addDisplacementMarker(() -> {
//                                    sleep(100);
//                                    shootFlicker.setPosition(0.45);
//                                    sleep(170);
//                                    shootFlicker.setPosition(0.1);
//                                    sleep(150);
//                                })
//                                .build();
//                        Trajectory traj4_1 = drive.trajectoryBuilder(traj3_1.end())
//                                .addTemporalMarker(0, () -> {
//                                    frontShoot.setPower(0);
//                                    backShoot.setPower(0);
//                                    //liftServo.setPosition(.63);
//                                })
//                                .lineToLinearHeading(new Pose2d(44, 0, Math.toRadians(270))) //change pose
//                                .addDisplacementMarker(() -> {
//                                    sleep(250);
//                                    wobbleArmServo.setPosition(.44);
//                                    sleep(900);
//                                    wobbleClawServo.setPosition(.51); //need to change position and time
//                                    sleep(500);
//                                    //wobbleUp();
//                        /*wobbleArmServo.setPosition(.3);
//                        sleep(700);
//                        wobbleClawServo.setPosition(.5);
//                        sleep(350);*/
//                                })
//                                .build();
//                        Trajectory traj5_0_1 = drive.trajectoryBuilder(traj4_1.end())
//                                /*.addTemporalMarker(1.5, () -> {
//                                    wobbleDown();
//                        })*/
//                                .lineToLinearHeading(new Pose2d(44, 4, Math.toRadians(270)))
//                                //.lineToLinearHeading(new Pose2d(-26.6, -12, Math.toRadians(0)))
//                                /*.addTemporalMarker(1.5, () -> {
//                                    wobbleArmServo.setPosition(0);
//                                })*/
//                                .build();
//
//                        Trajectory traj5_1 = drive.trajectoryBuilder(traj5_0_1.end())
//                                .lineToLinearHeading(new Pose2d(-26.6, -17, Math.toRadians(180)))
//                                //.lineToLinearHeading(new Pose2d(-26.6, -12, Math.toRadians(0)))
//                                /*.addTemporalMarker(1.5, () -> {
//                                    wobbleArmServo.setPosition(0);
//                                })*/
//                                .addDisplacementMarker(() -> {
//                                    sleep(250);
//                                    wobbleUp();
//                                })
//                                .build();
//
//                        Trajectory traj6_1 = drive.trajectoryBuilder(traj5_1.end())
//                                .lineToLinearHeading(new Pose2d(32, 12, Math.toRadians(270))) //change pose
//                                .addDisplacementMarker(() -> {
//                                    sleep(250);
//                                    wobbleArmServo.setPosition(.44);
//                                    sleep(350);
//                                    wobbleClawServo.setPosition(.51); //need to change position and time
//                                    sleep(200);
//                                })
//                                .build();
//
//                        Trajectory traj7_1 = drive.trajectoryBuilder(traj6_1.end())
//                                .lineToLinearHeading(new Pose2d(24, 8, 0))
//                                .addDisplacementMarker(() -> {
//                                    sleep(250);
//                                    wobbleUp();
//                                })
//                                .build();
//
//                        drive.followTrajectory(traj1_1);
//                        //sleep(400);
//                        //shoot();
//                        drive.followTrajectory(traj2_1);
//                        //shoot();
//                        drive.followTrajectory(traj3_1);
//                        //shoot();
//                        //
//                        drive.followTrajectory(traj4_1);
//            /*wobbleArmServo.setPosition(.5);
//            sleep(100);*/
//                        //wobble goal
//                        //sleep(300);
//                        drive.followTrajectory(traj5_0_1);
//                        drive.followTrajectory(traj5_1);
//                        //picking up 2nd wobble goal
//                        drive.followTrajectory(traj6_1);
//                        //dropping off the 2nd wobble goal
//                        drive.followTrajectory(traj7_1);
//                        //white line
//                        //58, 63 2 x wobble 0 15,8 y wobble 0
//                        //82, 88 x wobble 1 10, 18 7wobble 1
//
//                        // Transfer the current pose to PoseStorage so we can use it in TeleOp
//                        PoseStorage.currentPose = drive.getPoseEstimate();
//                        break;
//                    case ZeroRings:
//                        Trajectory traj1 = drive.trajectoryBuilder(startPose)
//                                //.splineToConstantHeading(new Vector2d(-45, -2), 0)
//                                .addTemporalMarker(0, () -> {
//                                    setVelocity(frontShoot, 2700);
//                                    setVelocity(backShoot, 2700);
//                                })
//                                .splineToConstantHeading(new Vector2d(3, 13), 0)
//                                .addDisplacementMarker(() -> {
//                                    sleep(100);
//                                    shootFlicker.setPosition(0.45);
//                                    sleep(170);
//                                    shootFlicker.setPosition(0.1);
//                                })
//                                .build();
//
//                        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                                .lineToLinearHeading(new Pose2d(3, 21.5, Math.toRadians(0)))
//                                .addDisplacementMarker(() -> {
//                                    sleep(100);
//                                    shootFlicker.setPosition(0.45);
//                                    sleep(170);
//                                    shootFlicker.setPosition(0.1);
//                                })
//                                .build();
//
//                        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                                .lineToLinearHeading(new Pose2d(3, 29, Math.toRadians(0)))
//                                .addDisplacementMarker(() -> {
//                                    sleep(100);
//                                    shootFlicker.setPosition(0.45);
//                                    sleep(170);
//                                    shootFlicker.setPosition(0.1);
//                                    sleep(150);
//                                })
//                                .build();
//                        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                                .addTemporalMarker(0, () -> {
//                                    frontShoot.setPower(0);
//                                    backShoot.setPower(0);
//                                    //liftServo.setPosition(.63);
//                                })
//                                .lineToLinearHeading(new Pose2d(20, -25, Math.toRadians(270))) //change pose
//                                .addDisplacementMarker(() -> {
//                                    sleep(250);
//                                    wobbleArmServo.setPosition(.44);
//                                    sleep(800);
//                                    wobbleClawServo.setPosition(.51); //need to change position and time
//                                    sleep(500);
//                                    //wobbleUp();
//                        /*wobbleArmServo.setPosition(.3);
//                        sleep(700);
//                        wobbleClawServo.setPosition(.5);
//                        sleep(350);*/
//                                })
//                                .build();
//                        Trajectory traj5_0 = drive.trajectoryBuilder(traj4.end())
//                                /*.addTemporalMarker(1.5, () -> {
//                                    wobbleDown();
//                        })*/
//                                .lineToLinearHeading(new Pose2d(21, -15, Math.toRadians(270)))
//                                //.lineToLinearHeading(new Pose2d(-26.6, -12, Math.toRadians(0)))
//                                /*.addTemporalMarker(1.5, () -> {
//                                    wobbleArmServo.setPosition(0);
//                                })*/
//                                .build();
//
//                        Trajectory traj5 = drive.trajectoryBuilder(traj5_0.end())
//                                .lineToLinearHeading(new Pose2d(-26.6, -17, Math.toRadians(180)))
//                                //.lineToLinearHeading(new Pose2d(-26.6, -12, Math.toRadians(0)))
//                                /*.addTemporalMarker(1.5, () -> {
//                                    wobbleArmServo.setPosition(0);
//                                })*/
//                                .addDisplacementMarker(() -> {
//                                    sleep(250);
//                                    wobbleUp();
//                                })
//                                .build();
//
//                        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
//                                .lineToLinearHeading(new Pose2d(18, -12, Math.toRadians(270))) //change pose
//                                .addDisplacementMarker(() -> {
//                                    sleep(250);
//                                    wobbleArmServo.setPosition(.44);
//                                    sleep(350);
//                                    wobbleClawServo.setPosition(.51); //need to change position and time
//                                    sleep(200);
//                                })
//                                .build();
//
//                        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
//                                .lineToLinearHeading(new Pose2d(24, 8, 0))
//                                .addDisplacementMarker(() -> {
//                                    sleep(250);
//                                    wobbleUp();
//                                })
//                                .build();
//
//                        drive.followTrajectory(traj1);
//                        //sleep(400);
//                        //shoot();
//                        drive.followTrajectory(traj2);
//                        //shoot();
//                        drive.followTrajectory(traj3);
//                        //shoot();
//                        //
//                        drive.followTrajectory(traj4);
//            /*wobbleArmServo.setPosition(.5);
//            sleep(100);*/
//                        //wobble goal
//                        //sleep(300);
//                        drive.followTrajectory(traj5_0);
//                        drive.followTrajectory(traj5);
//                        //picking up 2nd wobble goal
//                        drive.followTrajectory(traj6);
//                        //dropping off the 2nd wobble goal
//                        drive.followTrajectory(traj7);
//                        //white line
//                        //58, 63 2 x wobble 0 15,8 y wobble 0
//                        //82, 88 x wobble 1 10, 18 7wobble 1
//
//                        // Transfer the current pose to PoseStorage so we can use it in TeleOp
//                        PoseStorage.currentPose = drive.getPoseEstimate();
//                        break;
//                    case Default:
//                        telemetry.addData("ERROR", "NUMBER OF RINGS NOT RECOGNIZED");
//                        break;
                }
                telemetry.update();
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

    /*************************************************************************************************
     *************************************************************************************************
     *************************************************************************************************
     */

    public abstract static class RingDetecting extends LinearOpMode {
        protected WebcamName webcamName;
        protected OpenCvWebcam webcam;


        public void webcamInitialize() {
            webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
            webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
            webcam.setPipeline(pipeline);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
                }
            });
        }

        public int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        protected static RingDetection pipeline = new RingDetection();

        public enum RingPosition {
            FOUR,
            ONE,
            NONE
        }

        public static class RingDetection extends OpenCvPipeline {

            private final Scalar BLUE = new Scalar(0, 0, 255);
            private final Scalar GREEN = new Scalar(0, 255, 0);

            private final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(460, 370);

            private final int REGION_WIDTH = 170;
            private final int REGION_HEIGHT = 180;

            private int FOUR_RING_THRESHOLD = 145;
            private int ONE_RING_THRESHOLD = 132;

            Point region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            Point region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

            Mat region1_Cb;
            Mat YCrCb = new Mat();
            Mat Cb = new Mat();
            static int avg1;

            public volatile RingPosition position;

            void inputToCb(Mat input) {
                Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
                Core.extractChannel(YCrCb, Cb, 1);
            }

            @Override
            public void init(Mat firstFrame) {
                inputToCb(firstFrame);

                region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            }

            @Override
            public Mat processFrame(Mat input) {
                    inputToCb(input);

                    avg1 = (int) Core.mean(region1_Cb).val[0];

                    Imgproc.rectangle(
                            input, // Buffer to draw on
                            region1_pointA, // First point which defines the rectangle
                            region1_pointB, // Second point which defines the rectangle
                            BLUE, // The color the rectangle is drawn in
                            2); // Thickness of the rectangle lines


                    if (avg1 > FOUR_RING_THRESHOLD) {
                        position = RingPosition.FOUR;
                    } else if (avg1 > ONE_RING_THRESHOLD) {
                        position = RingPosition.ONE;
                    } else {
                        position = RingPosition.NONE;
                    }

                    Imgproc.rectangle(
                            input, // Buffer to draw on
                            region1_pointA, // First point which defines the rectangle
                            region1_pointB, // Second point which defines the rectangle
                            GREEN, // The color the rectangle is drawn in
                            -1); // Negative thickness means solid fill

                    return input;
            }
        }
    }
}