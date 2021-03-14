package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "IntakeTest", group = "testing")
public class ShootingTest extends LinearOpMode {

    private DcMotor intake1, intake2;
    private DcMotor frontShoot, backShoot;
    private Servo liftServo, shootFlicker;

    @Override
    public void runOpMode() {
        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");

        frontShoot = hardwareMap.dcMotor.get("frontShoot");
        backShoot = hardwareMap.dcMotor.get("backShoot");

        liftServo = hardwareMap.servo.get("wobbleClawServo");
        shootFlicker = hardwareMap.servo.get("wobbleArmServo");

        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                intake1.setPower(0);
                intake2.setPower(0);

                frontShoot.setPower(.65);
                backShoot.setPower(.65); //change to PID thing

                liftServo.setPosition(1);

                if (gamepad1.b) {
                    shootFlicker.setPosition(1);
                    sleep(50);
                    shootFlicker.setPosition(0);
                }

                telemetry.update();
            }
        }
    }
}
