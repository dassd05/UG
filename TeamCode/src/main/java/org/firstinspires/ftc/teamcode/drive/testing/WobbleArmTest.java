package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "WobbleArmTest", group = "testing" )
public class WobbleArmTest extends LinearOpMode {

    private Servo wobbleClawServo, wobbleArmServo;
    private DcMotor intake1, intake2;
    private DcMotor frontShoot, backShoot;

    @Override
    public void runOpMode() {

        wobbleClawServo = hardwareMap.servo.get("wobbleClawServo");
        wobbleArmServo = hardwareMap.servo.get("wobbleArmServo");

        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");

        frontShoot = hardwareMap.dcMotor.get("frontShoot");
        backShoot = hardwareMap.dcMotor.get("backShoot");


        waitForStart();

        if (opModeIsActive()) {
            while(opModeIsActive()) {
                intake1.setPower(0);
                intake2.setPower(0);

                frontShoot.setPower(0);
                backShoot.setPower(0);

                if (gamepad1.dpad_up) {
                    wobbleClawServo.setPosition(.8);
                    sleep(300);
                    wobbleArmServo.setPosition(.8);
                }
                //lifting wobble goal

                if (gamepad1.dpad_down) {
                    wobbleArmServo.setPosition(.3);
                    wobbleClawServo.setPosition(.3);
                }
                //setting wobble goal down

                if (gamepad1.dpad_right) {
                    wobbleArmServo.setPosition(.5);
                    sleep(200);
                    wobbleClawServo.setPosition(.3);
                }
                //dropping wobble goal down
            }
        }
    }
}