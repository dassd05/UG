package org.firstinspires.ftc.teamcode.drive.advanced;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.drive.pogcode.GamepadListenerEx;

import java.util.*;

@TeleOp(group = "advanced")
public class TeleOpAugmentedDrivingRed extends LinearOpMode {

    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_GEAR_RATIO = 1;


    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private VoltageSensor batteryVoltageSensor;


    private Servo shooterStopper, wobbleArm1, wobbleArm2, shootFlicker, flap, turret, droptakeStopper, wobbleClaw;

    private DcMotor intake, bottomRoller;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;

    public BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;


    /********************************************************************************************
     *
     */

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(45, 0, 0, 25);
    public static PIDFCoefficients MOTOR_VELO_PID_2 = new PIDFCoefficients(45, 0, 0, 25); // fix this

    public static double lastKf = 17.7;
    public static double lastKf_2 = 17.7; // fix this

    /********************************************************************************************
     *
     */

    double lastVoltage = 0;



    boolean intakeOn = false;
    boolean reversed = false;

    double reverse = 1;

    boolean shooterOn = false;

    @Override
    public void runOpMode() throws InterruptedException {

        double driveTurn;

        double gamepadXCoordinate;
        double gamepadYCoordinate;
        double gamepadHypot;
        double gamepadXControl;
        double gamepadYControl;
        double gamepadDegree;
        double robotDegree;
        double movementRadian;

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
        backShoot.setMotorType(motorConfigurationType2);


        frontShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        shootFlicker = hardwareMap.get(Servo.class, "shootFlicker");

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();


        setPIDFCoefficients(frontShoot, MOTOR_VELO_PID);


        setPIDFCoefficients2(backShoot, MOTOR_VELO_PID_2);

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

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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


        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.update();
        telemetry.clearAll();


        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(PoseStorage.currentPose);


        GamepadListenerEx gamepadListener1 = new GamepadListenerEx(gamepad1) {
            @Override
            public void onButtonPress(Button button) {
                super.onButtonPress(button);
                if (button == Button.b) shoot();

                if (button == Button.a) shooterOn = !shooterOn;

            }
        };
        GamepadListenerEx gamepadListener2 = new GamepadListenerEx(gamepad2) {
            @Override
            public void onButtonPress(Button button) {
                super.onButtonPress(button);

                if (button == Button.right_bumper) intakeOn = !intakeOn;

                //if (button == Button.left_bumper) reversed = !reversed;

            }
        };



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

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

            lastVoltage = batteryVoltageSensor.getVoltage();


            if (gamepad1.dpad_up) {
                wobbleUp();
            }

            if (gamepad1.dpad_down) {
                wobbleDown();
            }

            if (gamepad1.dpad_left) {
                wobbleClaw.setPosition(.6);
            }

            turret.setPosition(.2);
            flap.setPosition(.48);

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();


            if (intakeOn) {
                if (gamepad2.left_bumper) {
                    intake.setPower(-.8);
                    bottomRoller.setPower(.7);
                } else {
                    intake.setPower(.8);
                    bottomRoller.setPower(-.7);
                }
            } else {
                intake.setPower(0);
                bottomRoller.setPower(0);
            }

            if (shooterOn) {
                setVelocity(frontShoot, 2700);
                setVelocity2(backShoot, 2700);
                shooterStopper.setPosition(.4);
            } else if (!shooterOn) {
                frontShoot.setVelocity(0);
                backShoot.setVelocity(0);
                shooterStopper.setPosition(.9);
            }


            driveTurn = -gamepad1.left_stick_x/2;

            gamepadXCoordinate = gamepad1.right_stick_x;
            gamepadYCoordinate = -gamepad1.right_stick_y;
            gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);
            gamepadDegree = Math.toDegrees(Math.atan2(gamepadYCoordinate, gamepadXCoordinate)) + 90;
            if (gamepadDegree > 180) {
                gamepadDegree = -360 + gamepadDegree;
            }
            robotDegree = getAngle();
            movementRadian = Math.toRadians(gamepadDegree - robotDegree);
            gamepadXControl = gamepadHypot * Math.cos(movementRadian);
            gamepadYControl = gamepadHypot * Math.sin(movementRadian);

            double fr = Range.clip((gamepadYControl * Math.abs(gamepadYControl)) - (gamepadXControl * Math.abs(gamepadXControl)) + driveTurn, -1, 1);
            double fl = Range.clip((gamepadYControl * Math.abs(gamepadYControl)) + (gamepadXControl * Math.abs(gamepadXControl)) - driveTurn, -1, 1);
            double bl = Range.clip((gamepadYControl * Math.abs(gamepadYControl)) - (gamepadXControl * Math.abs(gamepadXControl)) - driveTurn, -1, 1);
            double br = Range.clip((gamepadYControl * Math.abs(gamepadYControl)) + (gamepadXControl * Math.abs(gamepadXControl)) + driveTurn, -1, 1);

            if (gamepad1.right_bumper) {
                frontRight.setPower(fr / 2);
                frontLeft.setPower(fl / 2);
                backLeft.setPower(bl / 2);
                backRight.setPower(br / 2);
            } else {
                frontRight.setPower(fr);
                frontLeft.setPower(fl);
                backLeft.setPower(bl);
                backRight.setPower(br);
            }

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
            gamepadListener1.update();
            gamepadListener2.update();
        }
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

    public void shoot() {
        shootFlicker.setPosition(.35);
        sleep(280);
        shootFlicker.setPosition(.57);
    }

    public void wobbleUp() {
        wobbleClaw.setPosition(.3);
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
}
