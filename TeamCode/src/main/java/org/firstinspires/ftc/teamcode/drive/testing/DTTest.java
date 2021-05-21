package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "DTTest", group = "testing")
public class DTTest extends LinearOpMode {

    private DcMotor frontLeft, backLeft;
    private DcMotor frontRight, backRight;

    public static double speedFR = 0.0;
    public static double speedBL = 0.0;
    public static double speedFL = 0.0;
    public static double speedBR = 0.0;

    public FtcDashboard dashboard;


    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");


        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                frontLeft.setPower(speedFL);
                frontRight.setPower(speedFR);
                backLeft.setPower(speedBL);
                backRight.setPower(speedBR);

                telemetry.update();
            }
        }
    }
}

