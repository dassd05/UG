package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(group = "testing")
public class ServosTest extends LinearOpMode {

    Servo liftServo, wobbleClawServo, wobbleArmServo, shootFlicker;

    public static double liftServoVal = 0; // 0.08 (up) - 0.58 (down)
    public static double wobbleClawVal = 0; // 0.5 (open) - 0.9 (closed) - 1 (auton start)
    public static double wobbleArmVal = 0; // 0.03 (down) - 0.35 (drop-off) - 0.5 (up) - 1.0 (auton start)
    public static double shootFlickerVal = 0; // 0.10 (shot) - 0.45 (back)

    public FtcDashboard dashboard;


    @Override
    public void runOpMode() throws InterruptedException {
        liftServo = hardwareMap.get(Servo.class, "liftServo");
        wobbleClawServo = hardwareMap.get(Servo.class, "wobbleClawServo");
        wobbleArmServo = hardwareMap.get(Servo.class, "wobbleArmServo");
        shootFlicker = hardwareMap.get(Servo.class, "shootFlicker");
        Servo[] servos = {liftServo, wobbleClawServo, wobbleArmServo, shootFlicker};

        dashboard = FtcDashboard.getInstance();


        waitForStart();

        while (opModeIsActive()) {
            if (liftServoVal != -1) liftServo.setPosition(liftServoVal);
            if (wobbleClawVal != -1) wobbleClawServo.setPosition(wobbleClawVal);
            if (wobbleArmVal != -1) wobbleArmServo.setPosition(wobbleArmVal);
            if (shootFlickerVal != -1) shootFlicker.setPosition(shootFlickerVal);
        }


        int s = -1;
        boolean aPressed = false;
        boolean stickMoved = false;

//        while (opModeIsActive()) {
//
//            if (gamepad1.a) aPressed = true;
//            else if (aPressed) {
//                s ++;
//                aPressed = false;
//                stickMoved = false;
//                while (gamepad1.right_stick_y != 0) { sleep(100); }
//            }
//            if (s >= 4) s -= 4;
//            if (gamepad1.right_stick_y > 0.015) stickMoved = true;
//
//            if (s >= 0 & s < 4 && stickMoved) servos[s].setPosition(Range.clip(gamepad1.right_stick_y, 0, 1));
//            for (Servo servo : servos) {
//                servo.setPosition(servo.getPosition());
//            }
//
//            telemetry.addData("s", s);
//            telemetry.addData("liftServo", liftServo.getPosition());
//            telemetry.addData("wobbleClawServo", wobbleClawServo.getPosition());
//            telemetry.addData("wobbleArmServo", wobbleArmServo.getPosition());
//            telemetry.addData("shootFlicker", shootFlicker.getPosition());
//            telemetry.update();
//        }
    }
}
