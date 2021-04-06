package org.firstinspires.ftc.teamcode.legacycode;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

@Disabled
@TeleOp(name = "OdometryTest", group = "test")
public class OdometryTest extends LinearOpMode {

    private DcMotor frontLeft; // In port order
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    private BNO055IMU imu;

    double posX = 0; // inches
    double posY = 0;

    double deltaVerticalLeftEncoder;
    double deltaVerticalRightEncoder;
    double deltaHorizontalEncoder;
    double deltaHeading;
    double heading;
    double deltaX;
    double deltaY;

    double headingCorrection;
    double startHeading;
    public Vector2D position;

    double lastAngle = 0;

    double horizontal;
    double vertical;
    double turn;
    boolean RunToPointTeleOp = false;
    boolean Powershot = false;




    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersIMU.loggingEnabled = true;
        parametersIMU.loggingTag = "imu";
        parametersIMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);

        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        final int TICKS_PER_REV = 8192; // 8192 ticks per revolution
        final double ODO_WHEEL_CIRCUM = 35 / 25.4 * Math.PI; // Wheels are 35 mm diameter
        final double odo_constant = ODO_WHEEL_CIRCUM / TICKS_PER_REV;

        double horizontalEncoderPrevious = -backLeft.getCurrentPosition() * odo_constant;
        double verticalLeftEncoderPrevious = frontLeft.getCurrentPosition() * odo_constant;
        double verticalRightEncoderPrevious = backRight.getCurrentPosition() * odo_constant;
        double horizontalEncoderCurrent;
        double verticalLeftEncoderCurrent;
        double verticalRightEncoderCurrent;
        double verticalEncoderCurrent;

        long startTime = System.currentTimeMillis();
        long timeElapsed = startTime;
        int frameCount = 0;
        int fps = 0;
        long lastFrame = System.nanoTime();
        long timeBetweenFrames;

        double currentAngle;

        boolean firstLoop = true;

        boolean readWriteFiles = true;
        String fileName = "OdometryLogs";
        int fileVers = 0;
        String absFilePath = String.format("%s/FIRST/data/"+fileName+fileVers+".txt", Environment.getExternalStorageDirectory().getAbsolutePath());
        String[] fileCreationError = new String[0];

        position = new Vector2D(0,0);

        while (opModeIsActive()) {
            currentAngle = getAngle();

            setStartLocation(position,0);

            if (firstLoop) {
                if (readWriteFiles) {
                    try {
                        File file = new File(absFilePath);
                        while (!file.createNewFile()) {
                            fileVers ++;
                            absFilePath = String.format("%s/FIRST/data/"+fileName+fileVers+".txt", Environment.getExternalStorageDirectory().getAbsolutePath());
                            file = new File(absFilePath);
                            telemetry.addData("File Creation", "Already Exists");
                            telemetry.addData("file", absFilePath);
                            telemetry.update();
                        }
                        telemetry.addData("File Creation", "Success");
                        telemetry.update();
                    } catch (IOException e) {
                        telemetry.addData("File Creation", "Failed");
                        fileCreationError = new String[]{e.getLocalizedMessage(), e.getMessage()};
                    }
                }

                firstLoop = false;
            }

            horizontalEncoderCurrent = -backLeft.getCurrentPosition() * odo_constant - horizontalEncoderPrevious;
            verticalLeftEncoderCurrent = frontLeft.getCurrentPosition() * odo_constant - verticalLeftEncoderPrevious;
            verticalRightEncoderCurrent = backRight.getCurrentPosition() * odo_constant - verticalRightEncoderPrevious;
            verticalEncoderCurrent = (verticalLeftEncoderCurrent + verticalRightEncoderCurrent) / 2;

            if (Math.abs(currentAngle - lastAngle) > 180) {
                lastAngle += currentAngle > lastAngle ? 360 : -360;
            }

            //double hAdjustment = Math.toRadians(currentAngle - lastAngle) * 6.625;
            double hAdjustment = (getAngle() - lastAngle)/360 * 2 * 6.625 * Math.PI;
            double h = horizontalEncoderCurrent + hAdjustment;
            double v = verticalEncoderCurrent;
            double a = Math.toRadians(currentAngle);
            double d = Math.sqrt(Math.pow(h, 2) + Math.pow(v, 2));
            double b = Math.atan2(v, h) - a;
            posX += 2 * Math.cos(b) * d;
            posY += 2 * Math.sin(b) * d;

            horizontalEncoderPrevious = -backLeft.getCurrentPosition() * odo_constant;
            verticalLeftEncoderPrevious = frontLeft.getCurrentPosition() * odo_constant;
            verticalRightEncoderPrevious = backRight.getCurrentPosition() * odo_constant;//frontRight
            lastAngle = getAngle();


            deltaVerticalLeftEncoder =  verticalLeftEncoderCurrent - verticalLeftEncoderPrevious;
            deltaVerticalRightEncoder = verticalRightEncoderCurrent - verticalRightEncoderPrevious;
            deltaHorizontalEncoder = horizontalEncoderCurrent - horizontalEncoderPrevious;



            deltaHeading = (deltaVerticalRightEncoder - deltaVerticalLeftEncoder)/(2.0 * 8.15625 /* ENCODER_COUNTS_PER_INCH*/); //it's in radians
            heading = normalizeRadians((verticalRightEncoderCurrent - verticalLeftEncoderCurrent)/(2.0 * 8.15625 /* ENCODER_COUNTS_PER_INCH*/) + startHeading + headingCorrection);

            if(deltaHeading == 0){
                deltaX = deltaHorizontalEncoder;
                deltaY = (deltaVerticalLeftEncoder + deltaVerticalRightEncoder)/2;
            } else{
                double turnRadius = 8.15625 /* ENCODER_COUNTS_PER_INCH*/ * (deltaVerticalLeftEncoder + deltaVerticalRightEncoder)/(deltaVerticalRightEncoder - deltaVerticalLeftEncoder);
                double strafeRadius = deltaHorizontalEncoder/deltaHeading - 6.625 /* ENCODER_COUNTS_PER_INCH*/;

                deltaX = turnRadius * (Math.cos(deltaHeading) - 1) + strafeRadius * Math.sin(deltaHeading);
                deltaY = turnRadius * Math.sin(deltaHeading) + strafeRadius * (1 - Math.cos(deltaHeading));
            }

            horizontal = 0.7 * gamepad1.right_stick_x;
            vertical = -0.7 * gamepad1.right_stick_y;
            turn = -0.7 * gamepad1.left_stick_x;
            frontLeft.setPower(adjust((vertical + horizontal + turn), -1, 1));
            backLeft.setPower(adjust((vertical - horizontal + turn), -1, 1));
            frontRight.setPower(adjust((vertical - horizontal - turn), -1, 1));
            backRight.setPower(adjust((vertical + horizontal - turn), -1, 1));

            //regular tele-op shooting
            //may change to gamepad 2
            if (!RunToPointTeleOp && gamepad1.a) {
                //testing coordinates
                //not actual coordinates for high goal
                runToPosition(20, 20, .75, .5, 0,.8);
            } else if(gamepad1.a && RunToPointTeleOp) {
                //haven't coded shooting yet
                //this should allow for the servo to move and push the rings into the shooter

            }

            //power shot
            //may change to gamepad 2
            if (!RunToPointTeleOp && gamepad1.b) {
                //not decided yet for power shot
                runToPosition(0,0,0,0,0,0);
            } else if(gamepad1.b && RunToPointTeleOp) {
                //haven't coded shooting yet
                //this should allow for the servo to move and push the rings into the shooter
                //also need to program turning to shoot (might need to vary shooter motor power slightly)

            }

            frameCount++;
            if (System.currentTimeMillis() - timeElapsed > 1000) {
                fps = frameCount;
                frameCount = 0;
                timeElapsed = System.currentTimeMillis();
            }
            timeBetweenFrames = System.nanoTime() - lastFrame;
            lastFrame = System.nanoTime();

            //telemetry.addData("hAdjustment", hAdjustment);
            telemetry.addData("deltaX", deltaX);
            telemetry.addData("deltaY", deltaY);
            telemetry.addData("other heading heading", heading);
            telemetry.addData("possibly other heading deltaHeading", deltaHeading);
            telemetry.addData("imu heading", currentAngle);
            telemetry.addData("posX", getposX());
            telemetry.addData("posY", getposY());
            /*telemetry.addData("h", h);
            telemetry.addData("horAdd", (currentAngle - lastAngle) / 360 * 2 * 6.5 * Math.PI);
            telemetry.addData("v", v);
            telemetry.addData("a", a);
            telemetry.addData("b", b);
            telemetry.addData("d", d);
            telemetry.addData("fps", fps);
            telemetry.addData("speed", 1000000000 * d / timeBetweenFrames);
            telemetry.addData("timeBetweenFrames", timeBetweenFrames);*/

            for (String error : fileCreationError) {
                telemetry.addData("Error", error);
            }
            writeFile(absFilePath, "angle:" + currentAngle + "\nposX: " + getposX() + "\nposY: " + getposY()
                    + "\ntime: " + (System.currentTimeMillis() - startTime) + " ms\n");

            telemetry.update();
        }
    }

    public double getAngle() {
        return -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle;
    }

    public void setStartLocation(Vector2D startPosition, double startHeading){ //inches, and degrees
        position = new Vector2D(startPosition);
        this.startHeading = Math.toRadians(startHeading);
        heading = normalizeRadians(heading + this.startHeading);
    }

    public static double AngleWrap(double angle) {
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }
    public double normalizeRadians(double angle){
        while(angle >= 2*Math.PI) {
            angle -= 2*Math.PI;
        }
        while(angle < 0.0) {
            angle += 2*Math.PI;
        }
        return angle;
    }
    public void setHeading(double heading){ //degrees
        this.heading = normalizeRadians(Math.toRadians(heading));
    }
    public void setHeadingCorrection(double correct){
        headingCorrection = normalizeRadians(correct - this.heading + headingCorrection);
    }

    public double getposX() {
        return posX;
    }

    public double getposY() {
        return posY;
    }

    public double adjust(double number, double min, double max) {
        if (number < min) {
            number = min;
        }
        if (number > max) {
            number = max;
        }
        return number;
    }

    public void runToPosition(double x, double y, double movementSpeed, double turnSpeed, double desiredAngle,  double distanceErrorMargin) {
        //double preferredAngle = 0;
        double movement_x;
        double movement_y;
        double movement_turn;

        double distanceToTarget = Math.hypot(x - getposX(), y - getposY());
        double absoluteAngleToTarget = Math.atan2(y - getposY(), x - getposX());
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (Math.toRadians(getAngle()) - Math.toRadians(0)));
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;
        //double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180.0) + preferredAngle;
        movement_turn = adjust(-getAngle() / 10, -1.0, 1.0) * turnSpeed;

        /*if (Math.abs(distanceToTarget) < 10.0) {
            movement_turn = 0.0;
            movement_x = movementXPower * movementSpeed * .3;
            movement_y = movementYPower * movementSpeed * .3;
        }
        frontLeft.setPower((movement_y + movement_x + movement_turn));
        backLeft.setPower((movement_y - movement_x + movement_turn));
        frontRight.setPower((movement_y - movement_x - movement_turn));
        backRight.setPower((movement_y + movement_x - movement_turn));

        if (Math.abs(distanceToTarget) < distanceErrorMargin) {
            frontLeft.setPower(adjust(-(.04 * (getAngle() - desiredAngle)), -1, 1));
            backLeft.setPower(adjust(-(.04 * (getAngle() - desiredAngle)), -1, 1));
            frontRight.setPower(adjust((.04 * (getAngle() - desiredAngle)), -1, 1));
            backRight.setPower(adjust((.04 * (getAngle() - desiredAngle)), -1, 1));
        }
        if (Math.abs(distanceToTarget) < distanceErrorMargin && Math.abs(getAngle()) < .5) {
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

            RunToPointTeleOp = true;
        } else {
            RunToPointTeleOp = false;
        }*/
    }

    public void writeFile(String filePath, String data) {
        try {
            FileWriter myWriter = new FileWriter(filePath, true);
            myWriter.write(data + "\n");
            myWriter.close();
            telemetry.addData("File Writing", "Success");
        } catch (IOException e) {
            telemetry.addData("File Writing", "Fail");
            telemetry.addData("Error", e.getStackTrace());
            telemetry.addData("Error", e.getLocalizedMessage());
            telemetry.addData("Error", e.getMessage());
            telemetry.addData("Error", e.getSuppressed());
        }
    }
}