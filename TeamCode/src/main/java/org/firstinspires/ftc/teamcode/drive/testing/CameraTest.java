package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.stuff.BlueGoalVisionPipeline;
import org.firstinspires.ftc.teamcode.drive.stuff.Camera;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp(group = "testing")
public class CameraTest extends LinearOpMode {

    public static int camPixelHeight = 240;

    OpenCvCamera webcam1;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = FtcDashboard.getInstance();

        BlueGoalVisionPipeline pipeline = new BlueGoalVisionPipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam1.setPipeline(pipeline);

        webcam1.openCameraDeviceAsync(() -> {
            webcam1.startStreaming(camPixelHeight * 4/3, camPixelHeight, OpenCvCameraRotation.UPRIGHT);
            dashboard.startCameraStream(webcam1, 0);
        });

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();


        while(!isStopRequested()) {

            telemetry.addData("get distance to goal wall", pipeline.getDistanceToGoalWall());
            telemetry.addData("get field pos", pipeline.getFieldPositionFromGoal());
            telemetry.addData("get goal height", pipeline.getGoalHeight());
            telemetry.addData("get pitch", pipeline.getPitch());
            telemetry.addData("get yaw", pipeline.getYaw());
//            telemetry.addData("", pipeline.getPowerShotAngles(pipeline.getDistanceToGoalWall()));
            telemetry.addData("is goal visible", pipeline.isGoalVisible());
            telemetry.addData("is corners visible", pipeline.isCornersVisible());
            telemetry.addData("is goal centered", pipeline.isGoalCentered());
            telemetry.update();
        }

//        camera.vuforia.deactivate();
    }

}
