package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.pogcode.GamepadListenerEx;

import static org.firstinspires.ftc.teamcode.drive.OtherConstants.*;
import static org.firstinspires.ftc.teamcode.drive.subsystems.Robot.*;

@TeleOp(name = "TeleOp Blue", group = "1")
public class NewTeleOpBlue extends LinearOpMode {

    Robot r = new Robot(); //instantiate Robot object

    boolean intakeOn = false; //for toggling

    @Override
    public void runOpMode() throws InterruptedException {

        r.telemetry = telemetry;
        r.dashboard = FtcDashboard.getInstance();
        r.init(hardwareMap); //init hardware
        telemetry = new MultipleTelemetry(telemetry, r.dashboard.getTelemetry());

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(PoseStorage.currentPose);

        //controls shooter state (a toggle middle & x toggle high) (state machine makes life easy)
        GamepadListenerEx gamepadListener1 = new GamepadListenerEx(gamepad1) {
            @Override
            public void onButtonPress(Button button) {
                super.onButtonPress(button);

                if (button == Button.a && r.whatShootState != ShooterState.MIDDLE)
                    r.shootMiddle();
                else if (button == Button.a)
                    r.offMiddle();

                if (button == Button.x && r.whatShootState != ShooterState.HIGH)
                    r.shootHigh();
                else if (button == Button.x)
                    r.offHigh();
            }
        };
        //toggles intake on/off with right bumper
        GamepadListenerEx gamepadListener2 = new GamepadListenerEx(gamepad2) {
            @Override
            public void onButtonPress(Button button) {
                super.onButtonPress(button);
                if (button == Button.right_bumper) intakeOn = !intakeOn;
            }
        };

        waitForStart();

        while (opModeIsActive()) {

            r.setCorrectedPIDF(); //battery compensated feedforward application

            if (gamepad1.dpad_up) //wobble controls
                r.wgLift();
            if (gamepad1.dpad_down)
                r.wgDown();
            if (gamepad1.dpad_right)
                r.wgDeploy();
            if (gamepad1.dpad_left)
                r.wgStow();

            if (gamepad1.b) //b -> shoot
                r.flick();

            //when intake is on, left bumper can be held to reverse direction (can do toggle, but Harish no like)
            if (intakeOn) {
                if (gamepad2.left_bumper) r.intakeReverse();
                else r.intakeOn();
            } else {
                r.intakeOff();
            }

            r.updateAllStates(); // update state machine stuff for wg, intake, shooter, and flicker

            //gamepad inputs
            double driveTurn = -gamepad1.left_stick_x/2;
            double gamepadXCoordinate = -gamepad1.right_stick_x;
            double gamepadYCoordinate = gamepad1.right_stick_y;

            if (gamepad1.right_bumper) //holding right bumper -> slow mode
               r.fieldCentricDrive(driveTurn/2.5, gamepadXCoordinate/2.5,
                       gamepadYCoordinate/2.5);
             else
                r.fieldCentricDrive(driveTurn, gamepadXCoordinate, gamepadYCoordinate);

            //telemetry + general update stuff
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