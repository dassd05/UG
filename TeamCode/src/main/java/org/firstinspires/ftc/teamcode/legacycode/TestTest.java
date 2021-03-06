package org.firstinspires.ftc.teamcode.legacycode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Disabled
@TeleOp(name = "TestingTest", group = "test")
public class TestTest extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;

    public BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() {
        double driveTurn;
        double driveVertical;
        double driveHorizontal;

        double gamepadXCoordinate;
        double gamepadYCoordinate;
        double gamepadHypot;
        double gamepadXControl;
        double gamepadYControl;
        double gamepadDegree;
        double robotDegree;
        double movementRadian;

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

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

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        while (opModeIsActive()) {
            driveTurn = -0.7 * gamepad1.left_stick_x;
            driveVertical = -0.7 * gamepad1.right_stick_y;
            driveHorizontal = 0.7 * gamepad1.right_stick_x;

            gamepadXCoordinate = gamepad1.right_stick_x;
            gamepadYCoordinate = -gamepad1.right_stick_y;
            gamepadHypot = Math.hypot(gamepadXCoordinate, gamepadYCoordinate);
            gamepadDegree = Math.toDegrees(Math.atan2(gamepadXCoordinate, gamepadYCoordinate));
            robotDegree = getAngle();
            movementRadian = Math.toRadians(gamepadDegree - robotDegree);
            gamepadXControl = gamepadHypot * Math.cos(movementRadian);
            gamepadYControl = gamepadHypot * Math.sin(movementRadian);

            frontRight.setPower(Range.clip(((gamepadYControl * Math.abs(gamepadYControl)) - (gamepadXControl * Math.abs(gamepadXControl)) + driveTurn), -1, 1));
            frontLeft.setPower(Range.clip(((gamepadYControl * Math.abs(gamepadYControl)) + (gamepadXControl * Math.abs(gamepadXControl)) - driveTurn), -1, 1));
            backLeft.setPower(Range.clip(((gamepadYControl * Math.abs(gamepadYControl)) - (gamepadXControl * Math.abs(gamepadXControl)) - driveTurn), -1, 1));
            backRight.setPower(Range.clip(((gamepadYControl * Math.abs(gamepadYControl)) + (gamepadXControl * Math.abs(gamepadXControl)) + driveTurn), -1, 1));

            /*frontRight.setPower(driveTurn + driveVertical - driveHorizontal);
            backRight.setPower(driveTurn + driveVertical + driveHorizontal);
            frontLeft.setPower(-driveTurn + driveVertical + driveHorizontal);
            backLeft.setPower(-driveTurn + driveVertical - driveHorizontal);*/
        }
        telemetry.update();
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
}
