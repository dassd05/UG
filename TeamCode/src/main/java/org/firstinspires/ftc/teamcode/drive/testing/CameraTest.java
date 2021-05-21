package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.stuff.Camera;

@TeleOp(group = "testing")
public class CameraTest extends LinearOpMode {

    Camera camera;

    double xPos;
    double yPos;
    double zPos;
    double roll;
    double pitch;
    double heading;

    @Override
    public void runOpMode() throws InterruptedException {
        camera = new Camera(hardwareMap, new Camera.CamType.Webcam(null, null),  Camera.TrackingType.RING_DETECTION);

//        camera.vuforia = camera.new Vuforia();
//        Camera.Vuforia vuf = camera.new Vuforia();
//        vuf.activate();

//        waitForStart();

//        camera.vuforia.activate();

        while(!isStopRequested()) {
//            camera.vuforia.update();
//            xPos = camera.vuforia.xPos;
//            yPos = camera.vuforia.yPos;
//            zPos = camera.vuforia.zPos;
//            roll = camera.vuforia.roll;
//            pitch = camera.vuforia.pitch;
//            heading = camera.vuforia.heading;
//
//            telemetry.addData("position", "{x, y, z} = %.0f, %.0f, %.0f", xPos, yPos, zPos);
//            telemetry.addData("angles", "{roll, pitch, heading} = %.0f, %.0f, %.0f",
//                    roll, pitch, heading);
            telemetry.addData("num rings", camera.pipeline.position);
            telemetry.addData("avg1", camera.pipeline.avg1);
            telemetry.addData("avg2", camera.pipeline.avgs2);
            telemetry.update();
        }

//        camera.vuforia.deactivate();
    }

}
