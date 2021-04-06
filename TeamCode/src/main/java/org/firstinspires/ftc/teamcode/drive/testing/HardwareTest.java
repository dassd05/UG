package org.firstinspires.ftc.teamcode.drive.testing;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import java.lang.annotation.Annotation;

@SuppressWarnings({"FieldCanBeLocal", "CanBeFinal", "unused"})
@Config
@TeleOp(group = "testing")
public class HardwareTest extends LinearOpMode {

    private FtcDashboard dashboard;
    TelemetryPacket packet = new TelemetryPacket();

    private DcMotor motor0, motor1, motor2, motor3;
    private Servo servo0, servo2, servo3;
    private CRServo servo1;

    private LynxServoController servoController;
    private ServoConfigurationType servoConfigType0;
    private ServoConfigurationType servoConfigType1;
    private PwmControl.PwmRange pwmRange0;
    private PwmControl.PwmRange pwmRange1;
    private double[] pwmRangeVals0;
    private double[] pwmRangeVals1;

    public static double motorPowers = 0.0;

    public static ServoFlavor servoFlavor0 = ServoFlavor.STANDARD;
    public static ServoFlavor servoFlavor1 = ServoFlavor.CONTINUOUS;
    private ServoFlavor lastFlavor0 = servoFlavor0;
    private ServoFlavor lastFlavor1 = servoFlavor1;
    public static double servoPowPos0 = 0;
    public static double servoPowPos1 = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
//        motor2 = hardwareMap.get(DcMotor.class, "motor2");
//        motor3 = hardwareMap.get(DcMotor.class, "motor3");

        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servoConfigType0 = ServoConfigurationType.getStandardServoType();
        servoConfigType1 = ServoConfigurationType.getStandardServoType();

        servoController = (LynxServoController) servo0.getController();

        pwmRange0 = servoController.getServoPwmRange(0);
        pwmRange1 = servoController.getServoPwmRange(1);

        pwmRangeVals0 = new double[]{pwmRange0.usPulseLower, pwmRange0.usPulseUpper, pwmRange0.usFrame};
        pwmRangeVals1 = new double[]{pwmRange1.usPulseLower, pwmRange1.usPulseUpper, pwmRange1.usFrame};

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
//            motor2.setPower(motorPowers);
//            motor3.setPower(motorPowers);

            if (lastFlavor0 != servoFlavor0) {
                servoConfigType0.processAnnotation(servoFlavor0 == ServoFlavor.STANDARD ? standardServo() : continuousServo());
                servoController.setServoType(0, servoConfigType0);
                servoController.initializeHardware();
            } else if (lastFlavor1 != servoFlavor1) {
                servoConfigType1.processAnnotation(servoFlavor1 == ServoFlavor.STANDARD ? standardServo() : continuousServo());
                servoController.setServoType(1, servoConfigType1);
                servoController.initializeHardware();
            }

            servoController.setServoPosition(0, servoPowPos0);
            servoController.setServoPosition(1, servoPowPos1);

            telemetry_addData("servoConfigType0", servoConfigType0);
            telemetry_addData("servoConfigType1", servoConfigType1);
            telemetry_addData("servoConfigTypeFlavor0", servoConfigType0.getServoFlavor());
            telemetry_addData("servoConfigTypeFlavor1", servoConfigType1.getServoFlavor());
            telemetry_addData("servoConfigs equal", servoConfigType0.equals(servoConfigType1));
            telemetry_addData("pwm enabled", servoController.isServoPwmEnabled(0));
            telemetry_addData("pwmRangeVals0", pwmRangeVals0);
            telemetry_addData("pwmRangeVals1", pwmRangeVals1);
            telemetry_update();
        }
    }

    public ServoType standardServo() {
        return new ServoType() {
            @Override
            public Class<? extends Annotation> annotationType() {
                return ServoType.class;
            }

            @NonNull
            @Override
            public ServoFlavor flavor() {
                return ServoFlavor.STANDARD;
            }

            @Override
            public double usPulseLower() {
                return pwmRangeVals0[0];
            }

            @Override
            public double usPulseUpper() {
                return pwmRangeVals0[1];
            }

            @Override
            public double usPulseFrameRate() {
                return pwmRangeVals0[2];
            }
        };
    }
    public ServoType continuousServo() {
        return new ServoType() {
            @Override
            public Class<? extends Annotation> annotationType() {
                return ServoType.class;
            }

            @NonNull
            @Override
            public ServoFlavor flavor() {
                return ServoFlavor.CONTINUOUS;
            }

            @Override
            public double usPulseLower() {
                return pwmRangeVals1[0];
            }

            @Override
            public double usPulseUpper() {
                return pwmRangeVals1[1];
            }

            @Override
            public double usPulseFrameRate() {
                return pwmRangeVals1[2];
            }
        };
    }

    public void telemetry_addData(String caption, Object value) {
        telemetry.addData(caption, value);
        packet.put(caption, value);
    }
    public void telemetry_update() {
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
}
