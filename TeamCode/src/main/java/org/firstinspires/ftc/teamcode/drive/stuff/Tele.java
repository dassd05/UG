package org.firstinspires.ftc.teamcode.drive.stuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.drive.ServoConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config
@TeleOp(group = "testing")
public class Tele extends LinearOpMode {

//    public static final int RED_ALLIANCE_DRIVING = 90;
//    public static final int BLUE_ALLIANCE_DRIVING = 270;
//    public static double fieldCentricDriveAngle = RED_ALLIANCE_DRIVING;

    public static double turretPos = 0.18;
    public static boolean shoot = false;
    public static double flap = 0.39;
    public static double feedforward = 15.8;
    public static int velocity = 2500;

    private double lastTurretPos = turretPos;

    FtcDashboard dashboard;

    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        //------------------------------------------------------------------------------------------
        // INIT
        //------------------------------------------------------------------------------------------

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        PIDFCoefficients shooterPIDF = new PIDFCoefficients(45, 0, 0, feedforward);

//        BlueGoalVisionPipeline pipeline = new BlueGoalVisionPipeline();

        UltimateGoalBase.Parameters params = new UltimateGoalBase.Parameters(this);
        UltimateGoalBase.Hardware hardware = new UltimateGoalBase.Hardware(hardwareMap);
        hardware.getVoltageSensor();
        hardware.getShooter();
        hardware.shooter.setPIDFCoefficients(shooterPIDF, hardware.voltageSensor.getVoltage());
//        hardware.getCamera1(pipeline);
        params.hardware = hardware;
        params.dashboard = dashboard;
        params.telemetry = telemetry;

        UltimateGoalBase robot = new UltimateGoalBase(params);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        double angle = 0;

        while (opModeIsActive() && !isStopRequested()) {
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

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

            hardware.shooter.setVelocity(velocity);
            hardware.flap.setPosition(flap);

//            if (shoot = true) {

//                robot.queueShot();
            hardware.shootFlicker.setPosition(ServoConstants.shootFlickerShot);
            sleep(400);
            hardware.shootFlicker.setPosition(ServoConstants.shootFlickerOut);
            sleep(400);
//            }
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
            telemetry.addData("angle", angle);
            robot.update();
        }
    }
}
