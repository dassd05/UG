package org.firstinspires.ftc.teamcode.drive.stuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
@TeleOp(group = "testing")
public class Tele extends LinearOpMode {

    public static final int RED_ALLIANCE_DRIVING = 90;
    public static final int BLUE_ALLIANCE_DRIVING = 270;
    public static double fieldCentricDriveAngle = RED_ALLIANCE_DRIVING;

    @Override
    public void runOpMode() throws InterruptedException {
        //------------------------------------------------------------------------------------------
        // INIT
        //------------------------------------------------------------------------------------------

        UltimateGoalBase robot = new UltimateGoalBase(new UltimateGoalBase.Parameters(this));

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(Math.toRadians(fieldCentricDriveAngle) - robot.getPosition().getHeading());

            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() * .7,
                            input.getY() * .7,
                            (-gamepad1.right_stick_x * .7)
                    )
            );
        }
    }
}
