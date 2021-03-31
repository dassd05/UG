package org.firstinspires.ftc.teamcode.drive.testing;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import java.lang.annotation.Annotation;

@Config
@TeleOp(group = "testing")
public class HardwareTest extends LinearOpMode {

    private DcMotor motor0, motor1, motor2, motor3;
    private Servo servo0, servo1, servo2, servo3;
    private CRServo servo0CR;

    private ServoControllerEx servoController;
    ServoConfigurationType servoConfigType;

    public static double motorPowers = 0.1;

    public static ServoFlavor servoFlavor = ServoFlavor.STANDARD;
    public static double servoPosition = 0;
    public static double servoPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {
//        motor2 = hardwareMap.get(DcMotor.class, "motor2");
//        motor3 = hardwareMap.get(DcMotor.class, "motor3");

        servo0 = hardwareMap.get(Servo.class, "servo0");
//        servo0CR = hardwareMap.get(CRServo.class, "servo0");
        servoConfigType = ServoConfigurationType.getStandardServoType();

        servoController = (ServoControllerEx) servo0.getController();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
//            motor2.setPower(motorPowers);
//            motor3.setPower(motorPowers);

            servoConfigType.processAnnotation(new ServoType() {
                @Override
                public Class<? extends Annotation> annotationType() {
                    return ServoType.class;
                }

                @NonNull
                @Override
                public ServoFlavor flavor() {
                    return servoFlavor;
                }

                @Override
                public double usPulseLower() {
                    return servoConfigType.getUsPulseLower();
                }

                @Override
                public double usPulseUpper() {
                    return servoConfigType.getUsPulseUpper();
                }

                @Override
                public double usPulseFrameRate() {
                    return servoConfigType.getUsFrame();
                }
            });
            servoController.setServoType(servo0.getPortNumber(), servoConfigType);
            if (servoFlavor == ServoFlavor.STANDARD) servoController.setServoPosition(servo0.getPortNumber(), servoPosition);
            else if (servoFlavor == ServoFlavor.CONTINUOUS) servoController.setServoPosition(servo0.getPortNumber(), servoPower/2 + 0.5);
//            switch (servoFlavor) {
//                case STANDARD:
//                    servo0.setPosition(servoPosition);
//                case CONTINUOUS:
//                    servo0CR.setPower(servoPower);
//            }

            telemetry.update();
        }
    }
}
