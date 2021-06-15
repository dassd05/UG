package org.firstinspires.ftc.teamcode.drive.stuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config
@TeleOp(group = "testing")
public class Tele extends LinearOpMode {

//    public static final int RED_ALLIANCE_DRIVING = 90;
//    public static final int BLUE_ALLIANCE_DRIVING = 270;
//    public static double fieldCentricDriveAngle = RED_ALLIANCE_DRIVING;

    public static double turretPos = -1;
    public static boolean shoot = false;
    private double lastTurretPos = turretPos;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        //------------------------------------------------------------------------------------------
        // INIT
        //------------------------------------------------------------------------------------------

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

//        BlueGoalVisionPipeline pipeline = new BlueGoalVisionPipeline();

        UltimateGoalBase.Parameters params = new UltimateGoalBase.Parameters(this);
        UltimateGoalBase.Hardware hardware = new UltimateGoalBase.Hardware(hardwareMap);
        hardware.getVoltageSensor();
        hardware.getShooter();
//        hardware.getCamera1(pipeline);
        params.hardware = hardware;
        params.dashboard = dashboard;
        params.telemetry = telemetry;

        UltimateGoalBase robot = new UltimateGoalBase(params);

//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.setPipeline(pipeline);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//                FtcDashboard.getInstance().startCameraStream(webcam, 0);
//            }
//        });

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

//            Vector2d input = new Vector2d(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x
//            ).rotated(Math.toRadians(fieldCentricDriveAngle) - robot.getPosition().getHeading());
//
//            robot.drive.setWeightedDrivePower(
//                    new Pose2d(
//                            input.getX() * .7,
//                            input.getY() * .7,
//                            (-gamepad1.right_stick_x * .7)
//                    )
//            );

            if (shoot = true) {
                shoot = false;

                robot.queueShot();
            }
            if (turretPos != -1 && turretPos != lastTurretPos) {
                hardware.turret.setPosition(turretPos);
                lastTurretPos = turretPos;
            }

//            if(pipeline.isGoalVisible()) {
//                //returns positive if robot needs to turn counterclockwise
//                double motorPower = pipeline.getMotorPower();//), -1, 1, -0.5, 0.5);
////                robot.drive.setMotorPowers(-motorPower, -motorPower, motorPower, motorPower);
//            }
//
//            telemetry.addData("get distance to goal wall", pipeline.getDistanceToGoalWall());
//            telemetry.addData("get field pos", pipeline.getFieldPositionFromGoal());
//            telemetry.addData("get goal height", pipeline.getGoalHeight());
//            telemetry.addData("get pitch", pipeline.getPitch());
//            telemetry.addData("get yaw", pipeline.getYaw());
////            telemetry.addData("", pipeline.getPowerShotAngles(pipeline.getDistanceToGoalWall()));
//            telemetry.addData("is goal visible", pipeline.isGoalVisible());
//            telemetry.addData("is corners visible", pipeline.isCornersVisible());
//            telemetry.addData("is goal centered", pipeline.isGoalCentered());
            robot.update();
        }
    }
}
