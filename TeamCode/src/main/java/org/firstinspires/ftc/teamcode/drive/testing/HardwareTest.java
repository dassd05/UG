package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(group = "testing")
public class HardwareTest extends LinearOpMode {

    private DcMotor motor0, motor1, motor2, motor3;

    public static double motorPowers = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");

//        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        sleep(2000);
//
//        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            motor2.setPower(motorPowers);
            motor3.setPower(motorPowers);
        }
    }
}
