package org.firstinspires.ftc.teamcode.legacycode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TMain", group = "sensor")
public class Teleop extends Robot {
    @Override
    public void runOpMode() {
        initialize();

        double driveTurn;
        double driveVertical;
        double driveHorizontal;
        double driveDegree;
        double driveSpeed;
        double targetAngle = 0;

        boolean aPressed = false;
        boolean aReleased = false;
        int numShots = 0;

        boolean normalDrive = true;

        while (opModeIsActive()) {
            driveTurn = -0.7 * gamepad1.right_stick_x;
            driveVertical = -0.7 * gamepad1.left_stick_y;
            driveHorizontal = 0.7 * gamepad1.left_stick_x;
            driveDegree = Math.toDegrees(Math.atan(driveHorizontal / driveVertical));
            driveDegree += driveVertical > 0 ? 0 : 180;
            driveSpeed = Math.sqrt(Math.pow(driveVertical, 2) + Math.pow(driveHorizontal, 2));

            if (normalDrive) {
                turn( driveTurn);
                drive(driveVertical, driveHorizontal);
            } else {
                turn(driveTurn);
                drive((float)(driveDegree + 90 - getAngle()), driveSpeed);
            }

            if (gamepad1.dpad_up) {
                driveGyroStraight(targetAngle);
            } else if (gamepad1.dpad_right) {
                driveGyroStraight(targetAngle);
            } else if (gamepad1.dpad_down) {
                driveGyroStraight(targetAngle);
            } else if (gamepad1.dpad_left) {
                driveGyroStraight(targetAngle);
            } else {
                targetAngle = getAngle();
            }

            if (gamepad1.y) {
                driveStraightWhileTurn((float)driveDegree, driveSpeed);
            }

            if (gamepad1.right_bumper) {
                goMaxSpeed();
            } else if (gamepad1.left_bumper) {
                brake();
            }


            if (gamepad1.a) {
                aPressed = true;
            } else if (aPressed) {
                aPressed = false;
                aReleased = true;
                numShots ++;
            }


            if (gamepad1.x) {
                if (getWobbleArmPosition() > 90) {
                    releaseWobbleGoal();
                } else {
                    grabWobbleGoal();
                }
            }


            updateRobot();
            vuforiaCode.updateVuforia();
            updateFps();
            telemetry.addData("fps", getFps());
            telemetry.update();
        }
        // Disable Tracking when we are done;
        vuforiaCode.targetsUltimateGoal.deactivate();
    }

    public void initialize() {
        super.initialize();

        // Does the needed adjustments of the hardware
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        readyToGo();
    }
}



