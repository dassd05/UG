package org.firstinspires.ftc.teamcode.drive.auton.Blue1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.subsystems.Robot;
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
import static org.firstinspires.ftc.teamcode.drive.OtherConstants.*;
import static org.firstinspires.ftc.teamcode.drive.ServoConstants.*;
import static org.firstinspires.ftc.teamcode.drive.subsystems.Robot.*;

@Autonomous(group = "R2")
public class AutonRingDetectingBlue1 extends LinearOpMode {

    Robot r = new Robot();

    protected WebcamName webcamName;
    protected OpenCvWebcam webcam;

    public enum ThisManyRings {
        FourRings,
        OneRing,
        ZeroRings,
    }

    public volatile ThisManyRings HowManyRings;

    boolean firstTime = true;
    boolean runFSM = false; //when you realize that running state machine commands in a loop causes the timer to reset infinitely and causes everything to go crazy...

    @Override
    public void runOpMode() throws InterruptedException {

        r.telemetry = telemetry;
        r.dashboard = FtcDashboard.getInstance();
        r.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, r.dashboard.getTelemetry());

        webcamInit();

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        Pose2d startPose = new Pose2d(-50, -2, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        r.turret.setPosition(.15);
        r.flap.setPosition(.41);


        /**
         *  Four Ring Trajectories
         *  ------------------------------------------------------------------------------
         */

        Trajectory traj1_4 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-25, -17, 0))
                .addTemporalMarker(0, () -> {
                    r.flap.setPosition(.4);
                    r.turret.setPosition(.18);
                    r.setVelocity(r.shooter1, shooterAtStack);
                    r.setVelocity2(r.shooter2, shooterAtStack);
                })
                .addDisplacementMarker(() -> r.shooterStopper.setPosition(.4))
                .build();

        Trajectory traj2_4 = drive.trajectoryBuilder(traj1_4.end())
                .addTemporalMarker(0, () -> {
                    r.shooter1.setVelocity(shooterOff);
                    r.shooter2.setVelocity(shooterOff);
                })
                .lineToLinearHeading(new Pose2d(-5, -17, Math.toRadians(0)))
                .build();

        Trajectory traj3_4 = drive.trajectoryBuilder(traj2_4.end())
                .lineToLinearHeading(new Pose2d(-14, -17, 0))
                .addDisplacementMarker(() -> {
                    r.setVelocity(r.shooter1, shooterAfterIntake);
                    r.setVelocity2(r.shooter2, shooterAfterIntake);
                    r.intakeOn();
                    r.turret.setPosition(.15);
                })
                .build();

        Trajectory traj4_4 = drive.trajectoryBuilder(traj3_4.end())
                .lineToLinearHeading(new Pose2d(-5, -17, 0),
                        SampleMecanumDriveCancelable.getVelocityConstraint(4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDriveCancelable.getAccelerationConstraint(4))
                .build();

        Trajectory traj5_4 = drive.trajectoryBuilder(traj4_4.end())
                .lineToLinearHeading(new Pose2d(0, -17, 0),
                        SampleMecanumDriveCancelable.getVelocityConstraint(4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDriveCancelable.getAccelerationConstraint(4))
                .build();

        Trajectory traj6_4 = drive.trajectoryBuilder(traj5_4.end())
                .addTemporalMarker(0, () -> {
                    r.intakeOff();
                    r.shooter1.setVelocity(shooterOff);
                    r.shooter2.setVelocity(shooterOff);
                })
                .lineToLinearHeading(new Pose2d(65, -10, Math.toRadians(250)))
                .build();


        Trajectory traj7_4 = drive.trajectoryBuilder(traj6_4.end())
                .addTemporalMarker(500, () -> r.wgSlowStow())
                .lineToLinearHeading(new Pose2d(15, -5, Math.toRadians(0)))
                .build();

        /**------------------------------------------------------------------------------
         *  Four Ring Trajectories
         */


        /**
         * One Ring Trajectories
         * ------------------------------------------------------------------------------
         */

        Trajectory traj1_1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-25, -17, 0))
                .addTemporalMarker(0, () -> {
                    r.flap.setPosition(.4);
                    r.turret.setPosition(.18);
                    r.setVelocity(r.shooter1, shooterAtStack);
                    r.setVelocity2(r.shooter2, shooterAtStack);
                })
                .addDisplacementMarker(() -> r.shooterStopper.setPosition(.4))
                .build();

        Trajectory traj2_1 = drive.trajectoryBuilder(traj1_1.end())
                .addTemporalMarker(0, () -> {
                    r.setVelocity(r.shooter1, shooterAfterIntake);
                    r.setVelocity2(r.shooter2, shooterAfterIntake);
                    r.intakeOn();
                    r.turret.setPosition(.15);
                })
                .lineToLinearHeading(new Pose2d(-5, -17, 0))
                .build();

        Trajectory traj3_1 = drive.trajectoryBuilder(traj2_1.end())
                .addTemporalMarker(0, () -> {
                    r.shooter1.setVelocity(shooterOff);
                    r.shooter2.setVelocity(shooterOff);
                    r.intakeOff();
                })
                .lineToLinearHeading(new Pose2d(30, -4, Math.toRadians(120)))
                .build();


        Trajectory traj4_1 = drive.trajectoryBuilder(traj3_1.end())
                .addTemporalMarker(500, () -> r.wgSlowStow())
                .lineToLinearHeading(new Pose2d(15, 0, Math.toRadians(0)))
                .build();

        /**------------------------------------------------------------------------------
         * One Ring Trajectories
         */


        /**
         * Zero Ring Trajectories
         * ------------------------------------------------------------------------------
         */

        Trajectory traj1_0 = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    r.flap.setPosition(.4);
                    r.turret.setPosition(.18);
                    r.setVelocity(r.shooter1, shooterZeroRing);
                    r.setVelocity2(r.shooter2, shooterZeroRing);
                })
                .lineToLinearHeading(new Pose2d(-1, -5, Math.toRadians(350)))
                .addDisplacementMarker(() -> r.shooterStopper.setPosition(.4))
                .build();

        Trajectory traj2_0 = drive.trajectoryBuilder(traj1_0.end())
                .addTemporalMarker(0, () -> {
                    r.shooter1.setVelocity(0);
                    r.shooter2.setVelocity(0);
                })
                .lineToLinearHeading(new Pose2d(-30, -5, Math.toRadians(180)))
                .build();

        Trajectory traj3_0 = drive.trajectoryBuilder(traj2_0.end())
                .lineToLinearHeading(new Pose2d(0, -5, Math.toRadians(180)))
                .build();


        Trajectory traj4_0 = drive.trajectoryBuilder(traj3_0.end())
                .lineToLinearHeading(new Pose2d(15, -18, Math.toRadians(0)))
                .build();

        /**------------------------------------------------------------------------------
         * Zero Ring Trajectories
         */


        while (!opModeIsActive())
            updateStarterStack();

        waitForStart();

        if (isStopRequested()) return;

        FourRingState = FourRing.DROP_STOPPER;
        OneRingState = OneRing.DROP_STOPPER;
        ZeroRingState = ZeroRings.DROP_STOPPER;
        r.waitTimer.reset();

        while (opModeIsActive()) {

            r.setCorrectedPIDF();

            switch (HowManyRings) {
                case FourRings:
                    /**
                     * Four Ring FSM Following
                     * ------------------------------------------------------------------------
                     */
                    switch (FourRingState) {
                        case DROP_STOPPER:

                            r.droptakeStopper.setPosition(0);

                            if (r.waitTimer.time() >= 50) {
                                FourRingState = FourRing.TRAJECTORY1;
                                drive.followTrajectoryAsync(traj1_4);
                            }
                            break;

                        case TRAJECTORY1:

                            if (!drive.isBusy()) {
                                FourRingState = FourRing.WAIT1;
                                r.waitTimer.reset();
                            }
                            break;

                        case WAIT1:

                            if (r.waitTimer.time() >= 500) {
                                FourRingState = FourRing.SHOOT_1;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case SHOOT_1:

                            if (runFSM) {
                                r.flick();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= flickerRecoveryTime + 150) {
                                FourRingState = FourRing.SHOOT_2;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case SHOOT_2:

                            if (runFSM) {
                                r.flick();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= flickerRecoveryTime + 150) {
                                FourRingState = FourRing.SHOOT_3;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case SHOOT_3:

                            if (runFSM) {
                                r.flick();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= flickerRecoveryTime + 150) {
                                FourRingState = FourRing.KNOCK;
                                drive.followTrajectoryAsync(traj2_4);
                            }
                            break;

                        case KNOCK:

                            if (!drive.isBusy()) {
                                FourRingState = FourRing.GO_BACK;
                                drive.followTrajectoryAsync(traj3_4);
                            }
                            break;

                        case GO_BACK:

                            if (!drive.isBusy()) {
                                FourRingState = FourRing.INTAKE;
                                drive.followTrajectoryAsync(traj4_4);
                            }
                            break;

                        case INTAKE:

                            if (!drive.isBusy()) {
                                FourRingState = FourRing.SHOOT2_1;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case SHOOT2_1:

                            if (runFSM) {
                                r.flick();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= flickerRecoveryTime + 150) {
                                FourRingState = FourRing.SHOOT2_2;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case SHOOT2_2:

                            if (runFSM) {
                                r.flick();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= flickerRecoveryTime + 150) {
                                FourRingState = FourRing.SHOOT2_3;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case SHOOT2_3:

                            if (runFSM) {
                                r.flick();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= flickerRecoveryTime + 150) {
                                FourRingState = FourRing.INTAKE2;
                                drive.followTrajectoryAsync(traj5_4);
                            }
                            break;

                        case INTAKE2:

                            if (!drive.isBusy()) {
                                FourRingState = FourRing.SHOOT3_1;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case SHOOT3_1:

                            if (runFSM) {
                                r.flick();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= flickerRecoveryTime + 150) {
                                FourRingState = FourRing.SHOOT3_2;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case SHOOT3_2:

                            if (runFSM) {
                                r.flick();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= flickerRecoveryTime + 150) {
                                FourRingState = FourRing.TRAJECTORY6;
                                drive.followTrajectoryAsync(traj6_4);
                            }
                            break;

                        case TRAJECTORY6:

                            if (!drive.isBusy()) {
                                FourRingState = FourRing.WOBBLE;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case WOBBLE:

                            if (runFSM) {
                                r.wgSlowDown();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= 1500) {
                                FourRingState = FourRing.WHITE_LINE;
                                drive.followTrajectoryAsync(traj7_4);
                            }
                            break;

                        case WHITE_LINE:

                            if (!drive.isBusy()) {
                                FourRingState = FourRing.CORRECTION;
                                firstTime = true;
                            }
                            break;

                        case CORRECTION:
                            //originally had gyro angle correction, but dont really need it now
                            if (firstTime) {
                                r.flap.setPosition(.39);
                                r.turret.setPosition(.15);
                                r.shooterStopper.setPosition(.9);
                                firstTime = false;
                            }
                            r.setMecanumPowers(0, 0, 0, 0);
                            break;
                    }
                    break;
                /**
                 * ------------------------------------------------------------------------
                 * Four Ring FSM Following
                 */

                case OneRing:
                    /**
                     * One Ring FSM Following
                     * ------------------------------------------------------------------------
                     */
                    switch (OneRingState) {
                        case DROP_STOPPER:

                            r.droptakeStopper.setPosition(0);

                            if (r.waitTimer.time() >= 50) {
                                OneRingState = OneRing.TRAJECTORY1;
                                drive.followTrajectoryAsync(traj1_1);
                            }
                            break;

                        case TRAJECTORY1:

                            if (!drive.isBusy()) {
                                OneRingState = OneRing.WAIT;
                                r.waitTimer.reset();
                            }
                            break;

                        case WAIT:

                            if (r.waitTimer.time() >= 500) {
                                OneRingState = OneRing.SHOOT_1;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case SHOOT_1:

                            if (runFSM) {
                                r.flick();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= flickerRecoveryTime + 150) {
                                OneRingState = OneRing.SHOOT_2;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case SHOOT_2:

                            if (runFSM) {
                                r.flick();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= flickerRecoveryTime + 150) {
                                OneRingState = OneRing.SHOOT_3;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case SHOOT_3:

                            if (runFSM) {
                                r.flick();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= flickerRecoveryTime + 150) {
                                OneRingState = OneRing.INTAKE;
                                drive.followTrajectoryAsync(traj2_1);
                            }
                            break;

                        case INTAKE:

                            if (!drive.isBusy()) {
                                OneRingState = OneRing.SHOOT2_1;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case SHOOT2_1:

                            if (runFSM) {
                                r.flick();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= flickerRecoveryTime + 150) {
                                OneRingState = OneRing.SHOOT2_2;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case SHOOT2_2:

                            if (runFSM) {
                                r.flick();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= flickerRecoveryTime + 150) {
                                OneRingState = OneRing.SHOOT2_3;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case SHOOT2_3:

                            if (runFSM) {
                                r.flick();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= flickerRecoveryTime + 150) {
                                OneRingState = OneRing.TRAJECTORY3;
                                drive.followTrajectoryAsync(traj3_1);
                            }
                            break;

                        case TRAJECTORY3:

                            if (!drive.isBusy()) {
                                OneRingState = OneRing.WOBBLE;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case WOBBLE:

                            if (runFSM) {
                                r.wgSlowDown();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= 1500) {
                                OneRingState = OneRing.WHITE_LINE;
                                drive.followTrajectoryAsync(traj4_1);
                            }
                            break;

                        case WHITE_LINE:

                            if (!drive.isBusy()) {
                                OneRingState = OneRing.CORRECTION;
                                firstTime = true;
                            }
                            break;

                        case CORRECTION:
                            //originally had gyro angle correction, but dont really need it now
                            if (firstTime) {
                                r.flap.setPosition(.39);
                                r.turret.setPosition(.15);
                                r.shooterStopper.setPosition(.9);
                                firstTime = false;
                            }
                            r.setMecanumPowers(0, 0, 0, 0);
                            break;
                    }
                    break;
                /**
                 * ------------------------------------------------------------------------
                 * One Ring FSM Following
                 */

                case ZeroRings:
                    /**
                     * Zero Ring FSM Following
                     * ------------------------------------------------------------------------
                     */
                    switch (ZeroRingState) {
                        case DROP_STOPPER:

                            r.droptakeStopper.setPosition(0);

                            if (r.waitTimer.time() >= 50) {
                                ZeroRingState = ZeroRings.TRAJECTORY1;
                                drive.followTrajectoryAsync(traj1_0);
                            }
                            break;

                        case TRAJECTORY1:

                            if (!drive.isBusy()) {
                                ZeroRingState = ZeroRings.WAIT1;
                                r.waitTimer.reset();
                            }
                            break;

                        case WAIT1:

                            if (r.waitTimer.time() >= 500) {
                                ZeroRingState = ZeroRings.SHOOT_1;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case SHOOT_1:

                            if (runFSM) {
                                r.flick();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= flickerRecoveryTime + 150) {
                                ZeroRingState = ZeroRings.SHOOT_2;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case SHOOT_2:

                            if (runFSM) {
                                r.flick();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= flickerRecoveryTime + 150) {
                                ZeroRingState = ZeroRings.SHOOT_3;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case SHOOT_3:

                            if (runFSM) {
                                r.flick();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= flickerRecoveryTime + 150) {
                                ZeroRingState = ZeroRings.TRAJECTORY2;
                                drive.followTrajectoryAsync(traj2_0);
                            }
                            break;

                        case TRAJECTORY2:

                            if (!drive.isBusy()) {
                                ZeroRingState = ZeroRings.TRAJECTORY3;
                                drive.followTrajectoryAsync(traj3_0);
                            }
                            break;

                        case TRAJECTORY3:

                            if (!drive.isBusy()) {
                                ZeroRingState = ZeroRings.WOBBLE;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case WOBBLE:

                            if (runFSM) {
                                r.wgSlowDown();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= 1500) {
                                ZeroRingState = ZeroRings.WAIT2;
                                runFSM = true;
                                r.waitTimer.reset();
                            }
                            break;

                        case WAIT2:

                            if (runFSM) {
                                r.wgSlowStow();
                                runFSM = false;
                            }

                            if (r.waitTimer.time() >= 10000) {
                                ZeroRingState = ZeroRings.WHITE_LINE;
                                drive.followTrajectoryAsync(traj4_0);
                            }
                            break;

                        case WHITE_LINE:

                            if (!drive.isBusy()) {
                                ZeroRingState = ZeroRings.CORRECTION;
                                firstTime = true;
                            }
                            break;

                        case CORRECTION:
                            //originally had gyro angle correction, but dont really need it now
                            if (firstTime) {
                                r.flap.setPosition(.39);
                                r.turret.setPosition(.15);
                                r.shooterStopper.setPosition(.9);
                                firstTime = false;
                            }
                            r.setMecanumPowers(0, 0, 0, 0);
                            break;
                    }
                    break;
                /**
                 * ------------------------------------------------------------------------
                 * Zero Ring FSM Following
                 */
            }

            r.updateAllStatesNoShooter();

            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    public void webcamInit() {
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
    }

    public void updateStarterStack() {
        if (pipeline.position == null) {
            telemetry.addData("still working on it", "gimme a sec");
        } else if (pipeline.position == RingDetecting.RingPosition.FOUR){
            telemetry.addData("Four Rings", "Waiting for start");
            HowManyRings = ThisManyRings.FourRings;
            telemetry.update();
        } else if (pipeline.position == RingDetecting.RingPosition.ONE){
            telemetry.addData("One Ring", "Waiting for start");
            HowManyRings = ThisManyRings.OneRing;
        } else if (pipeline.position == RingDetecting.RingPosition.NONE){
            telemetry.addData("Zero Rings", "Waiting for start");
            HowManyRings = ThisManyRings.ZeroRings;
        }
        telemetry.update();
    }

    /**
     --------------------------------------------------------------------------------------
     --------------------------------------------------------------------------------------
     --------------------------------------------------------------------------------------
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

            Point region1_pointA = new Point(
                    LEFT_REGION1_TOPLEFT_ANCHOR_POINT.x,
                    LEFT_REGION1_TOPLEFT_ANCHOR_POINT.y);
            Point region1_pointB = new Point(
                    LEFT_REGION1_TOPLEFT_ANCHOR_POINT.x + LEFT_REGION_WIDTH,
                    LEFT_REGION1_TOPLEFT_ANCHOR_POINT.y + LEFT_REGION_HEIGHT);

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


                if (avg1 > LEFT_4_THRESHOLD) {
                    position = RingPosition.FOUR;
                } else if (avg1 > LEFT_1_THRESHOLD) {
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