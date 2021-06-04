package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "MotorsTest", group = "testing")
public class MotorsTest extends LinearOpMode {

    private DcMotor frontLeft, backLeft;
    private DcMotor frontRight, backRight;

    private DcMotor shooter1, shooter2;
    private DcMotor intake, bottomRoller;

    public static double speedFR = 0.0;
    public static double speedBL = 0.0;
    public static double speedFL = 0.0;
    public static double speedBR = 0.0;

    public static double speedShooter1 = 0.0;
    public static double speedShooter2 = 0.0;
    public static double speedIntake = 0.0;
    public static double speedBottomRoller = 0.0;

    public FtcDashboard dashboard;


    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        shooter1 = hardwareMap.dcMotor.get("shooter1");
        shooter2 = hardwareMap.dcMotor.get("shooter2");

        intake = hardwareMap.dcMotor.get("intake");
        bottomRoller = hardwareMap.dcMotor.get("bottomRoller");


        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        bottomRoller.setDirection(DcMotorSimple.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                frontLeft.setPower(speedFL);
                frontRight.setPower(speedFR);
                backLeft.setPower(speedBL);
                backRight.setPower(speedBR);

                shooter2.setPower(speedShooter2);
                shooter1.setPower(speedShooter1);
                intake.setPower(speedIntake);
                bottomRoller.setPower(speedBottomRoller);

                telemetry.update();
            }
        }
    }
}

