package org.firstinspires.ftc.teamcode.drive.auton.Blue1;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
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

import static org.firstinspires.ftc.teamcode.drive.auton.Blue1.AutonRingDetectingBlue1.RingDetecting.pipeline;
import static org.firstinspires.ftc.teamcode.drive.auton.Blue1.AutonRingDetectingBlue1.RingDetecting.RingDetection.avg1;

@Autonomous(group = "R2")
public class AutonRingDetectingBlue1 extends LinearOpMode {

    protected WebcamName webcamName;
    protected OpenCvWebcam webcam;

    public enum ThisManyRings {
        FourRings,
        OneRing,
        ZeroRings,
        Default
    }

    public volatile ThisManyRings HowManyRings;

    public BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_GEAR_RATIO = 1;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private VoltageSensor batteryVoltageSensor;


    private Servo wobbleArm1, wobbleArm2, flap, turret, shooterStopper, shootFlicker, droptakeStopper, wobbleClaw;

    private DcMotor intake, bottomRoller;

    private DcMotor frontLeft, backRight, backLeft, frontRight;

    /********************************************************************************************************************
     *
     */

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(45,0,0,21.5);
    public static PIDFCoefficients MOTOR_VELO_PID_2 = new PIDFCoefficients(45,0,0,21.5);

    public static double lastKf = 17.7;
    public static double lastKf_2 = 17.7;

    /********************************************************************************************************************
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
                webcam.openCameraDevice();
                webcam.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
            }
        });

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        DcMotorEx frontShoot = hardwareMap.get(DcMotorEx.class, "shooter1");
        frontShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MotorConfigurationType motorConfigurationType = frontShoot.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        frontShoot.setMotorType(motorConfigurationType);

        DcMotorEx backShoot = hardwareMap.get(DcMotorEx.class, "shooter2");
        backShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MotorConfigurationType motorConfigurationType2 = backShoot.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        frontShoot.setMotorType(motorConfigurationType2);

        frontShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        setPIDFCoefficients(frontShoot, MOTOR_VELO_PID);
        setPIDFCoefficients2(backShoot, MOTOR_VELO_PID_2);

        for(LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        turret = hardwareMap.get(Servo.class, "turret");
        flap = hardwareMap.get(Servo.class, "flap");
        wobbleArm1 = hardwareMap.get(Servo.class, "wobbleArm1");
        wobbleArm2 = hardwareMap.get(Servo.class, "wobbleArm2");
        wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");
        droptakeStopper = hardwareMap.get(Servo.class, "droptakeStopper");
        shooterStopper = hardwareMap.get(Servo.class, "shooterStopper");
        shootFlicker = hardwareMap.get(Servo.class, "shootFlicker");

        intake = hardwareMap.get(DcMotor.class, "intake");
        bottomRoller = hardwareMap.get(DcMotor.class, "bottomRoller");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.update();
        telemetry.clearAll();


        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        Pose2d startPose = new Pose2d(-50, -2, Math.toRadians(0));

        drive.setPoseEstimate(startPose);


        wobbleClaw.setPosition(.6);
        wobbleArm1.setPosition(0);
        wobbleArm2.setPosition(0);

        turret.setPosition(.15);
        flap.setPosition(.41);

        shoot();

        droptakeStopper.setPosition(.25);
        shooterStopper.setPosition(.9);

        sleep(5000);

        wobbleClaw.setPosition(.38);


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

        if (isStopRequested()) return;

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

        boolean gyro = true;


        switch (HowManyRings) {
            case FourRings:
                Trajectory traj1_4 = drive.trajectoryBuilder(startPose)
                        .lineToLinearHeading(new Pose2d(-25, -17, 0))
                        .addTemporalMarker(0, () -> {
                            flap.setPosition(.4);
                            turret.setPosition(.14);
                            setVelocity(frontShoot, 2650);
                            setVelocity2(backShoot, 2650);
                        })
                        .addDisplacementMarker(() -> {
                            shooterStopper.setPosition(.4);
                        })
                        .build();

                Trajectory traj2_4 = drive.trajectoryBuilder(traj1_4.end())
                        .addTemporalMarker(0, () -> {
                            frontShoot.setVelocity(0);
                            backShoot.setVelocity(0);
                        })
                        .lineToLinearHeading(new Pose2d(-5,-17, Math.toRadians(0)))
                        .build();

                Trajectory traj3_4 = drive.trajectoryBuilder(traj2_4.end())
                        .lineToLinearHeading(new Pose2d(-14, -17, 0))
                        .addDisplacementMarker(() -> {
                            setVelocity(frontShoot, 2590);
                            setVelocity2(backShoot, 2590);
                            intake.setPower(.8);
                            bottomRoller.setPower(-.7);
                        })
                        .build();

                Trajectory traj4_4 = drive.trajectoryBuilder(traj3_4.end())
                        .lineToLinearHeading(new Pose2d(-5, -17, 0),
                                SampleMecanumDriveCancelable.getVelocityConstraint(4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDriveCancelable.getAccelerationConstraint(4))
                        .build();

                Trajectory traj4PointOh_4 = drive.trajectoryBuilder(traj4_4.end())
                        .lineToLinearHeading(new Pose2d(0, -17, 0),
                                SampleMecanumDriveCancelable.getVelocityConstraint(4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDriveCancelable.getAccelerationConstraint(4))
                        .build();

                Trajectory traj5_4 = drive.trajectoryBuilder(traj4PointOh_4.end())
                        .addTemporalMarker(0, () -> {
                            intake.setPower(0);
                            bottomRoller.setPower(0);
                            frontShoot.setVelocity(0);
                            backShoot.setVelocity(0);
                        })
                        .lineToLinearHeading(new Pose2d(70,-25, Math.toRadians(270)))
                        .build();


                Trajectory traj6_4 = drive.trajectoryBuilder(traj5_4.end())
                        .lineToLinearHeading(new Pose2d(15, -5, Math.toRadians(0)))
                        .build();

                droptakeStopper.setPosition(0);

                drive.followTrajectory(traj1_4);

                sleep(1000);

                shoot();
                sleep(500);
                shoot();
                sleep(500);
                shoot();
                sleep(500);

                drive.followTrajectory(traj2_4);

                drive.followTrajectory(traj3_4);

                drive.followTrajectory(traj4_4);

                shoot();
                sleep(200);
                shoot();
                sleep(200);
                shoot();
                sleep(200);

                drive.followTrajectory(traj4PointOh_4);

                shoot();
                sleep(200);
                shoot();

                drive.followTrajectory(traj5_4);

                while (wobbleArm2.getPosition() < .54) {
                    wobbleArm1.setPosition(wobbleArm1.getPosition() + .01);
                    wobbleArm2.setPosition(wobbleArm2.getPosition() + .01);
                    sleep(20);
                }
                wobbleClaw.setPosition(.6);
                sleep(500);

                while (wobbleArm2.getPosition() > 0) {
                    wobbleArm1.setPosition(wobbleArm1.getPosition() - .01);
                    wobbleArm2.setPosition(wobbleArm2.getPosition() - .01);
                    sleep(25);
                }


                drive.followTrajectory(traj6_4);

                flap.setPosition(.39);
                turret.setPosition(.15);
                shooterStopper.setPosition(.9);

                while (!isStopRequested()) {
                    while (Math.abs(getAngle()) > .5) {
                        frontRight.setPower(-.04 * getAngle());
                        backRight.setPower(-.04 * getAngle());
                        frontLeft.setPower(.04 * getAngle());
                        backLeft.setPower(.04 * getAngle());
                    }
                }
                PoseStorage.currentPose = drive.getPoseEstimate();

                break;

            case OneRing:

                Trajectory traj1_1 = drive.trajectoryBuilder(startPose)
                        .addTemporalMarker(0, () -> {
                            flap.setPosition(.4);
                            turret.setPosition(.18);
                            setVelocity(frontShoot, 2620);
                            setVelocity2(backShoot, 2620);
                        })
                        .lineToLinearHeading(new Pose2d(-1, -5, Math.toRadians(345)))
                        .addDisplacementMarker(() -> {
                            shooterStopper.setPosition(.4);
                        })
                        .build();

                Trajectory traj2_1 = drive.trajectoryBuilder(traj1_1.end())
                        .addTemporalMarker(0, () -> {
                            frontShoot.setVelocity(0);
                            backShoot.setVelocity(0);
                        })
                        .lineToLinearHeading(new Pose2d(30,-4, Math.toRadians(120)))
                        .build();


                Trajectory traj4_1 = drive.trajectoryBuilder(traj2_1.end())
                        .lineToLinearHeading(new Pose2d(15, 0, Math.toRadians(0)))
                        .build();

                droptakeStopper.setPosition(0);

                drive.followTrajectory(traj1_1);

                sleep(1000);

                shoot();
                sleep(500);
                shoot();
                sleep(500);
                shoot();
                sleep(500);

                drive.followTrajectory(traj2_1);

                while (wobbleArm2.getPosition() < .54) {
                    wobbleArm1.setPosition(wobbleArm1.getPosition() + .01);
                    wobbleArm2.setPosition(wobbleArm2.getPosition() + .01);
                    sleep(20);
                }
                wobbleClaw.setPosition(.6);
                sleep(500);

                while (wobbleArm2.getPosition() > 0) {
                    wobbleArm1.setPosition(wobbleArm1.getPosition() - .01);
                    wobbleArm2.setPosition(wobbleArm2.getPosition() - .01);
                    sleep(25);
                }


                drive.followTrajectory(traj4_1);

                flap.setPosition(.39);
                turret.setPosition(.15);
                shooterStopper.setPosition(.9);

                while (!isStopRequested()) {
                    while (Math.abs(getAngle()) > .5) {
                        frontRight.setPower(-.04 * getAngle());
                        backRight.setPower(-.04 * getAngle());
                        frontLeft.setPower(.04 * getAngle());
                        backLeft.setPower(.04 * getAngle());
                    }
                }
                PoseStorage.currentPose = drive.getPoseEstimate();

                break;
            case ZeroRings:

                Trajectory traj1_0 = drive.trajectoryBuilder(startPose)
                        .addTemporalMarker(0, () -> {
                            flap.setPosition(.4);
                            turret.setPosition(.18);
                            setVelocity(frontShoot, 2620);
                            setVelocity2(backShoot, 2620);
                        })
                        .lineToLinearHeading(new Pose2d(-1, -5, Math.toRadians(345)))
                        .addDisplacementMarker(() -> {
                            shooterStopper.setPosition(.4);
                        })
                        .build();

                Trajectory traj2PointOh_0 = drive.trajectoryBuilder(traj1_0.end())
                        .addTemporalMarker(0, () -> {
                            frontShoot.setVelocity(0);
                            backShoot.setVelocity(0);
                        })
                        .lineToLinearHeading(new Pose2d(-30,-5, Math.toRadians(180)))
                        .build();

                Trajectory traj2_0 = drive.trajectoryBuilder(traj2PointOh_0.end())
                        .lineToLinearHeading(new Pose2d(0,-5, Math.toRadians(180)))
                        .build();


                Trajectory traj4_0 = drive.trajectoryBuilder(traj2_0.end())
                        .lineToLinearHeading(new Pose2d(15, -18, Math.toRadians(0)))
                        .build();

                droptakeStopper.setPosition(0);

                drive.followTrajectory(traj1_0);


                sleep(1000);

                shoot();
                sleep(500);
                shoot();
                sleep(500);
                shoot();
                sleep(500);

                drive.followTrajectory(traj2PointOh_0);

                drive.followTrajectory(traj2_0);

                while (wobbleArm2.getPosition() < .54) {
                    wobbleArm1.setPosition(wobbleArm1.getPosition() + .01);
                    wobbleArm2.setPosition(wobbleArm2.getPosition() + .01);
                    sleep(20);
                }
                wobbleClaw.setPosition(.6);
                sleep(500);

                while (wobbleArm2.getPosition() > 0) {
                    wobbleArm1.setPosition(wobbleArm1.getPosition() - .01);
                    wobbleArm2.setPosition(wobbleArm2.getPosition() - .01);
                    sleep(25);
                }

                sleep(10000);

                drive.followTrajectory(traj4_0);

                flap.setPosition(.39);
                turret.setPosition(.15);
                shooterStopper.setPosition(.9);

                while (!isStopRequested()) {
                    while (Math.abs(getAngle()) > .5) {
                        frontRight.setPower(-.04 * getAngle());
                        backRight.setPower(-.04 * getAngle());
                        frontLeft.setPower(.04 * getAngle());
                        backLeft.setPower(.04 * getAngle());
                    }
                }

                PoseStorage.currentPose = drive.getPoseEstimate();

                break;

            case Default:
        }
        telemetry.update();
    }



    public void shoot() {
        shootFlicker.setPosition(.35);
        sleep(280);
        shootFlicker.setPosition(.57);
    }

    public void wobbleUp() {
        wobbleClaw.setPosition(.38);
        sleep(500);
        wobbleArm1.setPosition(.2);
        wobbleArm2.setPosition(.2);
        sleep(1000);
    }

    public void wobbleDown() {
        wobbleArm1.setPosition(.54);
        wobbleArm2.setPosition(.54);
        sleep(500);
        wobbleClaw.setPosition(.6);
        sleep(600);
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }

    public void setVelocity(DcMotorEx motor, double power) {
        motor.setVelocity(rpmToTicksPerSecond(power));
        Log.i("mode", "setting velocity");
    }

    public void setVelocity2(DcMotorEx motor, double power) {
        motor.setVelocity(rpmToTicksPerSecond(power));
        Log.i("mode", "setting velocity");
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        Log.i("config", "setting custom gains");
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }

    private void setPIDFCoefficients2(DcMotorEx motor, PIDFCoefficients coefficients) {
        Log.i("config", "setting custom gains");
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }
    void composeTelemetry() {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });
    }
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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

            private final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(1000, 780);

            private final int REGION_WIDTH = 280;
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

