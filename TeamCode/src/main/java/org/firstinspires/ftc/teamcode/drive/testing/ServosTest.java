package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "testing")
public class ServosTest extends LinearOpMode {

    Servo liftServo, wobbleClawServo, wobbleArmServo, shootFlicker;

    @Override
    public void runOpMode() throws InterruptedException {
        liftServo = hardwareMap.get(Servo.class, "liftServo");
        wobbleClawServo = hardwareMap.get(Servo.class, "wobbleClawServo");
        wobbleArmServo = hardwareMap.get(Servo.class, "wobbleArmServo");
        shootFlicker = hardwareMap.get(Servo.class, "shootFlicker");
        Servo[] servos = {liftServo, wobbleClawServo, wobbleArmServo, shootFlicker};

        waitForStart();

        int s = -1;
        boolean aPressed = false;
        boolean stickMoved = false;
        float lastStickPos;

        while (opModeIsActive()) {

            if (gamepad1.a) aPressed = true;
            else if (aPressed) s ++; stickMoved = false;
            lastStickPos = gamepad1.right_stick_y;
            if (lastStickPos != gamepad1.right_stick_y) stickMoved = true;


            for (Servo servo : servos) {
                servo.setPosition(servo.getPosition());
            }
            if (s != -1 && stickMoved) servos[s].setPosition(gamepad1.right_stick_y);

            telemetry.addData("liftServo", liftServo.getPosition());
            telemetry.addData("wobbleClawServo", wobbleClawServo.getPosition());
            telemetry.addData("wobbleArmServo", wobbleArmServo.getPosition());
            telemetry.addData("shootFlicker", shootFlicker.getPosition());
            telemetry.update();
        }
    }
}
