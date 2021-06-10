package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
@TeleOp(group = "testing")
public class Intake extends LinearOpMode {

    private DcMotor motor3;
    private DcMotor motor2;


    public static double speed = 0;
    public static double speed2 = 0;

    public FtcDashboard dashboard;


    @Override
    public void runOpMode() throws InterruptedException {
        motor3 = hardwareMap.get(DcMotor.class, "bottomRoller");
        motor2 = hardwareMap.get(DcMotor.class, "intake");
        dashboard = FtcDashboard.getInstance();

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            motor3.setPower(speed);
            motor2.setPower(speed2);

            telemetry.addData("voltage", voltageSensor.getVoltage());
            telemetry.update();
        }
    }
}
