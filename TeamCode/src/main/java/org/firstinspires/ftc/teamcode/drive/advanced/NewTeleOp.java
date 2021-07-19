package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.drive.subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.pogcode.GamepadListenerEx;

import static org.firstinspires.ftc.teamcode.drive.OtherConstants.*;

@TeleOp(group = "advanced")
    public class NewTeleOp extends LinearOpMode {

        Robot r = new Robot();

        boolean intakeOn = false;

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

            SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            drive.setPoseEstimate(PoseStorage.currentPose);


            GamepadListenerEx gamepadListener1 = new GamepadListenerEx(gamepad1) {
                @Override
                public void onButtonPress(Button button) {
                    super.onButtonPress(button);

                    if (button == Button.a && r.whatShootState != Robot.ShooterState.MIDDLE)
                        r.shootMiddle();
                    else if (button == Button.a)
                        r.offMiddle();

                    if (button == Button.x && r.whatShootState != Robot.ShooterState.HIGH)
                        r.shootHigh();
                    else if (button == Button.x)
                        r.offHigh();
                }
            };
            GamepadListenerEx gamepadListener2 = new GamepadListenerEx(gamepad2) {
                @Override
                public void onButtonPress(Button button) {
                    super.onButtonPress(button);
                    if (button == Button.right_bumper) intakeOn = !intakeOn;
                }
            };

            r.turret.setPosition(.2);
            r.flap.setPosition(.48);

            waitForStart();

            r.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

            if (isStopRequested()) return;

            while (opModeIsActive() && !isStopRequested()) {

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

                if (intakeOn) {
                    if (gamepad2.left_bumper) r.intakeReverse();
                    else r.intakeOn();
                } else if (!intakeOn) {
                    r.intakeOff();
                }

                r.updateIntakeState();

                r.updateShooterState();


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

                if (gamepad1.right_bumper)
                   r.setMecanumPowers(fl/3, fr/3, bl/3, br/3);
                 else
                    r.setMecanumPowers(fl, fr, bl, br);

                drive.update();

                Pose2d poseEstimate = drive.getPoseEstimate();

                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
                telemetry.update();
                gamepadListener1.update();
                gamepadListener2.update();
            }
        }
    }