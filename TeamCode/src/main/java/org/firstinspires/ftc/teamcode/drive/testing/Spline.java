package org.firstinspires.ftc.teamcode.drive.testing;

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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;


@Autonomous(group = "drive")
public class Spline extends LinearOpMode {

    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_MAX_RPM = 5400;
    public static double MOTOR_GEAR_RATIO = 1;

    public static boolean RUN_USING_ENCODER_FLYWHEEL = true;
    public static boolean DEFAULT_GAINS = false;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private VoltageSensor batteryVoltageSensor;

    private Servo shooterStopper, wobbleArm1, wobbleArm2, shootFlicker, flap, turret, droptakeStopper, wobbleClaw;

    private DcMotor intake, bottomRoller;


    /********************************************************************************************
     *
     */

    public static PIDFCoefficients MOTOR_VELO_FLYWHEEL_PID = new PIDFCoefficients(45, 0, 0, 25);
    public static PIDFCoefficients MOTOR_VELO_FLYWHEEL_PID_2 = new PIDFCoefficients(45, 0, 0, 25);

    public static double lastKf = 17;
    public static double lastKf_2 = 17; // fix this

    /********************************************************************************************
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
        backShoot.setMotorType(motorConfigurationType2);

        if (RUN_USING_ENCODER_FLYWHEEL)
            frontShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else
            frontShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (RUN_USING_ENCODER_FLYWHEEL)
            backShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else
            backShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        setPIDFCoefficients(frontShoot, MOTOR_VELO_FLYWHEEL_PID);

        setPIDFCoefficients2(backShoot, MOTOR_VELO_FLYWHEEL_PID_2);

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

        bottomRoller.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        Pose2d startPose = new Pose2d(-50, -2, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.update();
        telemetry.clearAll();

        /**
         *
         */

        //wobbleClaw.setPosition();
        wobbleArm1.setPosition(0);
        wobbleArm2.setPosition(0);

        turret.setPosition(.15);
        flap.setPosition(.41);

        droptakeStopper.setPosition(.25);
        shooterStopper.setPosition(.9);

        shootFlicker.setPosition(.35);
        sleep(250);
        shootFlicker.setPosition(.57);

        sleep(5000);

        //wobbleClaw.setPosition();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (lastKf_2 != MOTOR_VELO_FLYWHEEL_PID_2.f) {
                MOTOR_VELO_FLYWHEEL_PID_2.f = lastKf_2 * 12 / batteryVoltageSensor.getVoltage();
                lastKf_2 = MOTOR_VELO_FLYWHEEL_PID_2.f;
            }

            if (lastKf != MOTOR_VELO_FLYWHEEL_PID.f) {
                MOTOR_VELO_FLYWHEEL_PID.f = lastKf * 12 / batteryVoltageSensor.getVoltage();
                lastKf = MOTOR_VELO_FLYWHEEL_PID.f;
            }

            setPIDFCoefficients2(backShoot, MOTOR_VELO_FLYWHEEL_PID_2);
            setPIDFCoefficients(frontShoot, MOTOR_VELO_FLYWHEEL_PID);
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            Trajectory traj0_0 = drive.trajectoryBuilder(startPose)
                    .addTemporalMarker(0, () -> {
                        setVelocity(frontShoot, 2700);
                        setVelocity2(backShoot, 2700);
                    })
                    .splineToConstantHeading(new Vector2d(-15, 10), 0)
                    .addDisplacementMarker(() -> {
                        shooterStopper.setPosition(.4);
                    })
                    .build();

            Trajectory traj1_0 = drive.trajectoryBuilder(traj0_0.end())
                    .addTemporalMarker(0, () -> {
                        frontShoot.setVelocity(0);
                        backShoot.setVelocity(0);
                    })
                    .lineToLinearHeading(new Pose2d(45, -40, Math.toRadians(0)))
                    .build();

            Trajectory traj2_0 = drive.trajectoryBuilder(traj1_0.end())
                    .lineToLinearHeading(new Pose2d(55, -40, Math.toRadians(0)))
                    .build();

            Trajectory traj3_0 = drive.trajectoryBuilder(traj2_0.end())
                    .splineTo(new Vector2d(50, 20), Math.toRadians(0))
                    .build();

            Trajectory traj4_0 = drive.trajectoryBuilder(traj3_0.end())
                    .lineToLinearHeading(new Pose2d(15, 20, Math.toRadians(0)))
                    .build();


            droptakeStopper.setPosition(0);

            drive.followTrajectory(traj0_0);

            turret.setPosition(.24);
            sleep(500);
            shoot();
            turret.setPosition(.31);
            sleep(500);
            shoot();
            turret.setPosition(.39);
            sleep(500);
            shoot();

            drive.followTrajectory(traj1_0);

            sleep(250);
            wobbleDown();
            sleep(700);

            drive.followTrajectory(traj2_0);

            while (wobbleArm1.getPosition() > 0) {
                wobbleArm1.setPosition((wobbleArm1.getPosition()) - .01);
                wobbleArm2.setPosition((wobbleArm2.getPosition()) - .01);
                sleep(25);
            }

            drive.followTrajectory(traj3_0);

            sleep(1000);

            drive.followTrajectory(traj4_0);

            turret.setPosition(.15);
            flap.setPosition(.41);
            shooterStopper.setPosition(.9);


            PoseStorage.currentPose = drive.getPoseEstimate();
            break;
        }
    }

    public void shoot() {
        shootFlicker.setPosition(0.35);
        sleep(280);
        shootFlicker.setPosition(0.57);
    }
    public void wobbleUp () {
        //wobbleClaw.setPosition(.07); // need to change position and time
        sleep(700);
        wobbleArm1.setPosition(.2);
        wobbleArm2.setPosition(.2);
        sleep(500);
    }
    public void wobbleDown () {
        wobbleArm1.setPosition(.54);
        wobbleArm2.setPosition(.54);
        sleep(500);
        //wobbleClaw.setPosition(.07); // need to change position and time
        sleep(350);
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }

    private void setVelocity(DcMotorEx motor, double power) {
        if(RUN_USING_ENCODER_FLYWHEEL) {
            motor.setVelocity(rpmToTicksPerSecond(power));
            Log.i("mode", "setting velocity");
        }
        else {
            Log.i("mode", "setting power");
            motor.setPower(power / MOTOR_MAX_RPM);
        }
    }
    private void setVelocity2(DcMotorEx motor, double power) {
        if (RUN_USING_ENCODER_FLYWHEEL) {
            motor.setVelocity(rpmToTicksPerSecond(power));
            Log.i("mode", "setting velocity");
        } else {
            Log.i("mode", "setting power");
            motor.setPower(power / MOTOR_MAX_RPM);
        }
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        if(!RUN_USING_ENCODER_FLYWHEEL) {
            Log.i("config", "skipping RUE");
            return;
        }

        if (!DEFAULT_GAINS) {
            Log.i("config", "setting custom gains");
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
            ));
        } else {
            Log.i("config", "setting default gains");
        }
    }
    private void setPIDFCoefficients2(DcMotorEx motor, PIDFCoefficients coefficients) {
        if(!RUN_USING_ENCODER_FLYWHEEL) {
            Log.i("config", "skipping RUE");
            return;
        }

        if (!DEFAULT_GAINS) {
            Log.i("config", "setting custom gains");
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
            ));
        } else {
            Log.i("config", "setting default gains");
        }
    }
}
