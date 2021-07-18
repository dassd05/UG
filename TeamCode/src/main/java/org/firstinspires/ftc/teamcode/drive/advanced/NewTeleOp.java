package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.drive.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.pogcode.GamepadListenerEx;

import static org.firstinspires.ftc.teamcode.drive.OtherConstants.*;

@TeleOp(group = "advanced")
    public class NewTeleOp extends LinearOpMode {

        Robot r = new Robot();

        private FtcDashboard dashboard = FtcDashboard.getInstance();

        Orientation angles;
        Acceleration gravity;

        double lastVoltage = 0;

        boolean intakeOn = false;
        boolean reversed = false;

        double reverse = 1;

        boolean shooterOn = false;
        boolean highGoal = false;


        @Override
        public void runOpMode() throws InterruptedException {

            r.init(hardwareMap);

            double driveTurn;

            double gamepadXCoordinate;
            double gamepadYCoordinate;
            double gamepadHypot;
            double gamepadXControl;
            double gamepadYControl;
            double gamepadDegree;
            double robotDegree;
            double movementRadian;

            composeTelemetry();

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
                    if (button == Button.a) shooterOn = !shooterOn;

                    if (button == Button.x) highGoal = !highGoal;

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

            r.turret.setPosition(.2);
            r.flap.setPosition(.48);

            waitForStart();

            r.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

            if (isStopRequested()) return;

            while (opModeIsActive() && !isStopRequested()) {

                if (gamepad1.dpad_up)
                    r.wgLift();
                if (gamepad1.dpad_down)
                    r.wgDown();
                if (gamepad1.dpad_right)
                    r.wgDeploy();
                if (gamepad1.dpad_left)
                    r.wgStow();

                r.updateWGState();

                if (gamepad1.b)
                    r.flick();

                r.updateFlickState();

                if (lastKf_2 != MOTOR_VELO_PID_2.f) {
                    MOTOR_VELO_PID_2.f = lastKf_2 * 12 / r.batteryVoltageSensor.getVoltage();
                    lastKf_2 = MOTOR_VELO_PID_2.f;
                }

                if (lastKf != MOTOR_VELO_PID.f) {
                    MOTOR_VELO_PID.f = lastKf * 12 / r.batteryVoltageSensor.getVoltage();
                    lastKf = MOTOR_VELO_PID.f;
                }

                r.setPIDFCoefficients2(r.shooter2, MOTOR_VELO_PID_2);
                r.setPIDFCoefficients(r.shooter1, MOTOR_VELO_PID);

                lastVoltage = r.batteryVoltageSensor.getVoltage();

                drive.update();

                Pose2d poseEstimate = drive.getPoseEstimate();

                if (intakeOn) {
                    r.intake.setPower(iOn);
                    r.bottomRoller.setPower(brOn);
                    if (gamepad2.left_bumper) {
                        r.intake.setPower(-iOn);
                        r.bottomRoller.setPower(-brOn);
                    }
                } else if (!intakeOn) {
                    r.intake.setPower(iOff);
                    r.bottomRoller.setPower(brOff);
                }

                if (shooterOn) {
                    r.flap.setPosition(.48);
                    r.turret.setPosition(.2);
                    r.setVelocity(r.shooter1, 2900);
                    r.setVelocity2(r.shooter2, 2900);
                    r.shooterStopper.setPosition(.4);
                } else if (!shooterOn) {
                    r.shooter1.setVelocity(0);
                    r.shooter2.setVelocity(0);
                    r.shooterStopper.setPosition(.9);
                }

//            if (highGoal) {
//                turret.setPosition(.16);
//                flap.setPosition(.4);
//                setVelocity(frontShoot, 2590);
//                setVelocity2(backShoot, 2590);
//                shooterStopper.setPosition(.4);
//            } else if (!highGoal) {
//                flap.setPosition(.48);
//                frontShoot.setVelocity(0);
//                backShoot.setVelocity(0);
//                shooterStopper.setPosition(.9);
//                turret.setPosition(.2);
//            }


                driveTurn = -gamepad1.left_stick_x/2;

                gamepadXCoordinate = -gamepad1.right_stick_x;
                gamepadYCoordinate = gamepad1.right_stick_y;
                gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);
                gamepadDegree = Math.toDegrees(Math.atan2(gamepadYCoordinate, gamepadXCoordinate)) + 90;
                if (gamepadDegree > 180) {
                    gamepadDegree = -360 + gamepadDegree;
                }
                robotDegree = r.getAngle();
                movementRadian = Math.toRadians(gamepadDegree - robotDegree);
                gamepadXControl = gamepadHypot * Math.cos(movementRadian);
                gamepadYControl = gamepadHypot * Math.sin(movementRadian);

                double fr = Range.clip((gamepadYControl * Math.abs(gamepadYControl)) - (gamepadXControl * Math.abs(gamepadXControl)) + driveTurn, -1, 1);
                double fl = Range.clip((gamepadYControl * Math.abs(gamepadYControl)) + (gamepadXControl * Math.abs(gamepadXControl)) - driveTurn, -1, 1);
                double bl = Range.clip((gamepadYControl * Math.abs(gamepadYControl)) - (gamepadXControl * Math.abs(gamepadXControl)) - driveTurn, -1, 1);
                double br = Range.clip((gamepadYControl * Math.abs(gamepadYControl)) + (gamepadXControl * Math.abs(gamepadXControl)) + driveTurn, -1, 1);

                if (gamepad1.right_bumper) {
                   r.setMecanumPowers(fl/3, fr/3, bl/3, br/3);
                } else {
                    r.setMecanumPowers(fl, fr, bl, br);
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
                    angles = r.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = r.imu.getGravity();
                }
            });
        }

        public void shoot() {
            r.shootFlicker.setPosition(.35);
            sleep(280);
            r.shootFlicker.setPosition(.57);
        }
    }