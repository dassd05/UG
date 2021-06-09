package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    private Encoder leftEncoder, rightEncoder, horizontalEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "bottomRoller"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        horizontalEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backRight"));

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        horizontalEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);


        horizontalEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        double offsetRight = encoderTicksToInches(rightEncoder.getCurrentPosition());
        double offsetLeft = encoderTicksToInches(leftEncoder.getCurrentPosition());
        double offsetHorizontal = encoderTicksToInches(horizontalEncoder.getCurrentPosition());


        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("right", encoderTicksToInches(rightEncoder.getCurrentPosition()) - offsetRight);
            telemetry.addData("left", encoderTicksToInches(leftEncoder.getCurrentPosition()) - offsetLeft);
            telemetry.addData("horizontal", encoderTicksToInches(horizontalEncoder.getCurrentPosition()) - offsetHorizontal);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
    public static double encoderTicksToInches(double ticks) {
        return (17.5/25.4) * 2 * Math.PI * 1 * ticks / 8192;
    }
}
