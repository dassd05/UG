package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This opmode demonstrates how one can augment driver control by following Road Runner arbitrary
 * Road Runner trajectories at any time during teleop. This really isn't recommended at all. This is
 * not what Trajectories are meant for. A path follower is more suited for this scenario. This
 * sample primarily serves as a demo showcasing Road Runner's capabilities.
 * <p>
 * This bot starts in driver controlled mode by default. The player is able to drive the bot around
 * like any teleop opmode. However, if one of the select buttons are pressed, the bot will switch
 * to automatic control and run to specified location on its own.
 * <p>
 * If A is pressed, the bot will generate a splineTo() trajectory on the fly and follow it to
 * targetA (x: 45, y: 45, heading: 90deg).
 * <p>
 * If B is pressed, the bot will generate a lineTo() trajectory on the fly and follow it to
 * targetB (x: -15, y: 25, heading: whatever the heading is when you press B).
 * <p>
 * If Y is pressed, the bot will turn to face 45 degrees, no matter its position on the field.
 * <p>
 * Pressing X will cancel trajectory following and switch control to the driver. The bot will also
 * cede control to the driver once trajectory following is done.
 * <p>
 * The following may be a little off with this method as the trajectory follower and turn
 * function assume the bot starts at rest.
 * <p>
 * This sample utilizes the SampleMecanumDriveCancelable.java class.
 */
@TeleOp(group = "advanced")
public class TeleOpAugmentedDriving extends LinearOpMode {
    enum Mode {
        INTAKING,
        TELEOP_SHOOTING,
        ENDGAME
    }

    Mode currentMode = Mode.INTAKING;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(45, 45);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(90);

    // The location we want the bot to automatically go to when we press the B button
    Vector2d targetBVector = new Vector2d(-15, 25);

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(45);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            switch (currentMode) {
                case INTAKING:
                    currentMode = Mode.INTAKING;

                    if(gamepad1.y) {
                        currentMode = Mode.TELEOP_SHOOTING;
                    }
                    if (gamepad1.a) {
                        currentMode = Mode.ENDGAME;
                    }

                    /*if (gamepad1.a) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to TELEOP_SHOOTING

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(targetAVector, targetAHeading)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.TELEOP_SHOOTING;
                    } else if (gamepad1.b) {
                        // If the B button is pressed on gamepad1, we generate a lineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to TELEOP_SHOOTING

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .lineTo(targetBVector)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.TELEOP_SHOOTING;
                    } else if (gamepad1.y) {
                        // If Y is pressed, we turn the bot to the specified angle to reach
                        // targetAngle (by default, 45 degrees)

                        drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));

                        currentMode = Mode.TELEOP_SHOOTING;
                    }*/
                    break;
                case TELEOP_SHOOTING:
                    currentMode = Mode.TELEOP_SHOOTING;

                    if (gamepad1.x && drive.isBusy()) {
                        drive.cancelFollowing();
                        currentMode = Mode.INTAKING;
                    } else if (gamepad1.x && !drive.isBusy()) {
                        currentMode = Mode.INTAKING;
                    }
                    if (gamepad1.a && drive.isBusy()) {
                        drive.cancelFollowing();
                        currentMode = Mode.ENDGAME;
                    } else if (gamepad1.a && !drive.isBusy()) {
                        currentMode = Mode.ENDGAME;
                    }

                    break;
                case ENDGAME:
                    currentMode = Mode.ENDGAME;

                    if (gamepad1.x && drive.isBusy()) {
                        drive.cancelFollowing();
                        currentMode = Mode.INTAKING;
                    } else if (gamepad1.x && !drive.isBusy()) {
                        currentMode = Mode.INTAKING;
                    }
                    if (gamepad1.y && drive.isBusy()) {
                        drive.cancelFollowing();
                        currentMode = Mode.TELEOP_SHOOTING;
                    } else if (gamepad1.y && !drive.isBusy()) {
                        currentMode = Mode.TELEOP_SHOOTING;
                    }

                    break;
            }
        }
    }
}
