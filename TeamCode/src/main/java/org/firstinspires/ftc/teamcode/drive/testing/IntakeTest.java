package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "IntakeTest", group = "testing")
public class IntakeTest extends LinearOpMode {

    private DcMotor intake1, intake2;
    private DcMotor frontShoot, backShoot;

    private Servo liftServo;

    @Override
    public void runOpMode() {
        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");

        frontShoot = hardwareMap.dcMotor.get("frontShoot");
        backShoot = hardwareMap.dcMotor.get("backShoot");

        liftServo = hardwareMap.servo.get("liftServo");

        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                frontShoot.setPower(0);
                backShoot.setPower(0);

                liftServo.setPosition(0);

                intake1.setPower(.5);
                intake2.setPower(.5);

                telemetry.update();
            }
        }
    }
}
