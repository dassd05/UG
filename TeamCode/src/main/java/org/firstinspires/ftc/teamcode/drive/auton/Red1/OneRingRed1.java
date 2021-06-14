package org.firstinspires.ftc.teamcode.drive.auton.Red1;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;


@Autonomous(group = "R1")
public class OneRingRed1 extends LinearOpMode {

    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_GEAR_RATIO = 1;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private VoltageSensor batteryVoltageSensor;


    private Servo wobbleArm1, wobbleArm2, flap, turret, shooterStopper, shootFlicker, droptakeStopper, wobbleClaw;

    private DcMotor intake, bottomRoller;

    /********************************************************************************************************************
     *
     */

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(45,0,0,25);
    public static PIDFCoefficients MOTOR_VELO_PID_2 = new PIDFCoefficients(45,0,0,25);

    public static double lastKf = 16.7;
    public static double last_Kf_2 = 16.7;

    /********************************************************************************************************************
     *
     */

    double lastVoltage = 0;

    @Override
    public void runOpMode() throws InterruptedException {
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


        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.update();
        telemetry.clearAll();


        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        Pose2d startPose = new Pose2d(-50, -2, Math.toRadians(0));

        drive.setPoseEstimate(startPose);


        wobbleClaw.setPosition(.7);
        wobbleArm1.setPosition(0);
        wobbleArm2.setPosition(0);

        turret.setPosition(.2);
        flap.setPosition(.425);
        /**
         * change flap
         */

        droptakeStopper.setPosition(.25);
        shooterStopper.setPosition(.9);

        sleep(5000);

        wobbleClaw.setPosition(.38);


        waitForStart();


        if (isStopRequested()) return;

        /*******************************************************************************************
         *
         */
        Trajectory traj1_1 = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(700, () -> {
                    shootFlicker.setPosition(.35);
                })
                .addTemporalMarker(980, () -> {
                    shootFlicker.setPosition(.57);
                })
                .splineTo(new Vector2d(-24, -17), 0)
                .addDisplacementMarker(() -> {
                    flap.setPosition(.4);
                })
                .splineTo(new Vector2d(-24, 10), 0)
                .splineTo(new Vector2d(-15, 10), 0)
                .build();


        Trajectory traj2_1 = drive.trajectoryBuilder(traj1_1.end())
                .addTemporalMarker(0, () -> {
                    frontShoot.setVelocity(0);
                    backShoot.setVelocity(0);
                })
                .lineToLinearHeading(new Pose2d(45, 0, Math.toRadians(90)))
                .build();

        Trajectory traj3_1 = drive.trajectoryBuilder(traj2_1.end())
                .lineToLinearHeading(new Pose2d(45, 10, Math.toRadians(90)))
                .build();

        Trajectory traj4_1 = drive.trajectoryBuilder(traj3_1.end())
                .lineToLinearHeading(new Pose2d(20, 10, Math.toRadians(0)))
                .build();

        /*******************************************************************************************
         *
         */

        droptakeStopper.setPosition(0);
        setVelocity(frontShoot, 2700);
        setVelocity(frontShoot, 2700);
        sleep(350);
        intake.setPower(.8);
        bottomRoller.setPower(-.7);

        drive.followTrajectory(traj1_1);

        sleep(150);

        turret.setPosition(.2);
        sleep(300);
        shoot();
        turret.setPosition(.26);
        sleep(300);
        shoot();
        turret.setPosition(.32);
        sleep(300);
        shoot();

        drive.followTrajectory(traj2_1);

        while (wobbleArm2.getPosition() < .54) {
            wobbleArm1.setPosition(wobbleArm1.getPosition() + .01);
            wobbleArm2.setPosition(wobbleArm2.getPosition() + .01);
            sleep(25);
        }

        drive.followTrajectory(traj3_1);

        while (wobbleArm2.getPosition() > 0) {
            wobbleArm1.setPosition(wobbleArm1.getPosition() - .01);
            wobbleArm2.setPosition(wobbleArm2.getPosition() - .01);
            sleep(25);
        }
        wobbleClaw.setPosition(.7);

        drive.followTrajectory(traj4_1);

        flap.setPosition(.4);
        turret.setPosition(.15);
        shooterStopper.setPosition(.9);

        PoseStorage.currentPose = drive.getPoseEstimate();
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
        wobbleClaw.setPosition(.7);
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

