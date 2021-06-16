/* Uses contouring to actively identify and draw a box around the largest ring.
Then uses the Cb channel value of the area to check for orange color and
the aspect ratio of the drawn box to build a "confidence value" from 0 to 1 for ring positions
FOUR and ONE and a hesitance value for ring position NONE. These confidence values
are then used to determine the most accurate Ring Position.
 */



package org.firstinspires.ftc.teamcode.drive.vision;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.stuff.PIDController;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;


/**
 * Determine distance, yaw, and pitch
 *
 */

@Config
public class BlueGoalVisionPipeline extends OpenCvPipeline {

    static class Fraction {
        private int numerator, denominator;

        Fraction(long a, long b) {
            numerator = (int) (a / gcd(a, b));
            denominator = (int) (b / gcd(a, b));
        }

        /**
         * @return the greatest common denominator
         */
        private long gcd(long a, long b) {
            return b == 0 ? a : gcd(b, a % b);
        }

        public int getNumerator() {
            return numerator;
        }

        public int getDenominator() {
            return denominator;
        }
    }

    //---------------- EDITABLE CONSTANTS --------------

    //PID Controller
    public static double kP = 0.02;
    public static double kI = 0;
    public static double kD = 0.0005;
    public static double TOLERANCE = 0.5;

    public PIDController headingController = new PIDController(kP, kI, kD);

    public static double CAMERA_HEIGHT = 29.5;//0; //camera height inches for distance calculation
    public static double HIGH_GOAL_CENTER_HEIGHT = 35.875; //camera height inches for distance calculation

    //in degrees
    public static int CAMERA_PITCH_OFFSET = 12; // GAH WHY DOES THIS HAVE TO BE SO PRECISE. one deg off, and whole thing changes by several inches
    public static int CAMERA_YAW_OFFSET = 0;

    public static double CENTER_X_OFFSET = 0;
    public static double CENTER_Y_OFFSET = 0;


    public static double FOV = 55;//90;


    //Boundary Line (Only detects above this to eliminate field tape)
    public static int BOUNDARY = 500;//130;


    //Mask constants to isolate blue coloured subjects
    public static double UPPER_BLUE_THRESH = 250;//200;
    public static double LOWER_BLUE_THRESH = 170;//145;


    //Countour Filter Constants
    public static double CONTOUR_AREA_MIN = 30;
    public static double CONTOUR_ASPECT_RATIO_MIN  = 1;
    public static double CONTOUR_ASPECT_RATIO_MAX = 2;

    //degrees
    public static double HIGH_GOAL_SETPOINT = -14;
    public static double minimumMotorPower = 0.05;



    // --------------- WORKING VARIBALES -----------------------

    //Pinhole Camera Variables
    protected double centerX;
    protected double centerY;

    public int imageWidth;
    public int imageHeight;

    //Aspect ratio (3 by 2 by default)
    public static int horizontalRatio;
    public static int verticalRatio;


    private double horizontalFocalLength;
    private double verticalFocalLength;


    //working mat variables
    public Mat YCrCbFrame;
    public Mat CbFrame;
    public Mat MaskFrame;
    public Mat ContourFrame;


    //Contour Analysis Variables
    public List<MatOfPoint> blueContours;
    public MatOfPoint biggestBlueContour;
    public Rect blueRect;

    Point upperLeftCorner;
    Point upperRightCorner;
    Point lowerLeftCorner;
    Point lowerRightCorner;

    Point upperMiddle;
    Point lowerMiddle;
    Point leftMiddle;
    Point rightMiddle;

    //Viewfinder tracker
    public static int viewfinderIndex = 1;



    public BlueGoalVisionPipeline () {
        YCrCbFrame = new Mat();
        CbFrame = new Mat();
        MaskFrame = new Mat();
        ContourFrame = new Mat();

        blueContours = new ArrayList<MatOfPoint>();
        biggestBlueContour = new MatOfPoint();
        blueRect =  new Rect();

        upperLeftCorner = new Point();
        upperRightCorner = new Point();
        lowerLeftCorner = new Point();
        lowerRightCorner = new Point();

        upperMiddle  = new Point();
        lowerMiddle  = new Point();
        leftMiddle = new Point();
        rightMiddle = new Point();

    }

    @Override
    public void init(Mat mat) {
        super.init(mat);

        imageWidth = mat.width();
        imageHeight = mat.height();

        //Center of frame
        centerX = ((double) imageWidth / 2) - 0.5;
        centerY = ((double) imageHeight / 2) - 0.5;


        // pinhole model calculations
        double diagonalView = Math.toRadians(FOV); // 55
        Fraction aspectFraction = new Fraction(this.imageWidth, this.imageHeight);
        int horizontalRatio = aspectFraction.getNumerator(); // 4
        int verticalRatio = aspectFraction.getDenominator(); // 3
        double diagonalAspect = Math.hypot(horizontalRatio, verticalRatio); // 5
        double horizontalView = Math.atan(Math.tan(diagonalView / 2) * (horizontalRatio / diagonalAspect)) * 2;
        double verticalView = Math.atan(Math.tan(diagonalView / 2) * (verticalRatio / diagonalAspect)) * 2;
        horizontalFocalLength = this.imageWidth / (2 * Math.tan(horizontalView / 2));
        verticalFocalLength = this.imageHeight / (2 * Math.tan(verticalView / 2));

    }

    @Override
    public Mat processFrame(Mat input) {

        //Convert to YCrCb
        Imgproc.cvtColor(input, YCrCbFrame, Imgproc.COLOR_RGB2YCrCb);

        //Extract Cb channel (how close to blue)
        Core.extractChannel(YCrCbFrame, CbFrame, 2);

        //Threshhold using Cb channel
        Imgproc.threshold(CbFrame, MaskFrame, LOWER_BLUE_THRESH, UPPER_BLUE_THRESH, Imgproc.THRESH_BINARY);

        //clear previous contours to remove false positives if goal isn't in frame
        blueContours.clear();

        Imgproc.findContours(MaskFrame, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        blueContours = blueContours.stream().filter(i -> {
            boolean appropriateAspect = ((double) Imgproc.boundingRect(i).width / Imgproc.boundingRect(i).height > CONTOUR_ASPECT_RATIO_MIN)
                    && ((double) Imgproc.boundingRect(i).width / Imgproc.boundingRect(i).height < CONTOUR_ASPECT_RATIO_MAX);
            boolean aboveBoundaryLine = Imgproc.boundingRect(i).y + Imgproc.boundingRect(i).height < BOUNDARY;
            return aboveBoundaryLine && filterContours(i) && appropriateAspect;
        }).collect(Collectors.toList());



        if (blueContours.size() != 0) {
            // Comparing width instead of area because wobble goals that are close to the camera tend to have a large area
            biggestBlueContour = Collections.max(blueContours, (t0, t1) -> {
                return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
            });
            blueRect = Imgproc.boundingRect(biggestBlueContour);
            findCorners();
        } else {
            blueRect = null;
        }

        //-------------------DRAW ONTO FRAME -------------------------

        //draw center
        Imgproc.line(input, new Point(centerX, centerY+5), new Point(centerX, centerY-5), new Scalar(0,0,0));  //crosshair vertical
        Imgproc.line(input, new Point(centerX+5, centerY), new Point(centerX-5, centerY), new Scalar(0,0,0));  //crosshair horizontal

        //draw center offset (where you want the goal to be)
        Imgproc.line(input, new Point(centerX + CENTER_X_OFFSET, centerY + CENTER_Y_OFFSET + 5), new Point(centerX + CENTER_X_OFFSET, centerY + CENTER_Y_OFFSET -5), new Scalar(255,0,0));  //crosshair vertical
        Imgproc.line(input, new Point(centerX + CENTER_X_OFFSET + 5, centerY + CENTER_Y_OFFSET), new Point(centerX + CENTER_X_OFFSET - 5, centerY + CENTER_Y_OFFSET), new Scalar(255,0,0));  //crosshair horizontal

        //draw boundary line
        Imgproc.line(input, new Point(0, BOUNDARY), new Point(this.imageWidth, BOUNDARY), new Scalar(0, 0, 255));

        if(isGoalVisible()){


            //draw contours
            Imgproc.drawContours(input, blueContours, -1, new Scalar(255, 255, 0));

            //draw bounding box
//            Imgproc.rectangle(input, blueRect, new Scalar(0,255,0), 1);

            //Draw Corners and Middle
            Imgproc.circle(input, upperLeftCorner,2, new Scalar(0,255,0),2);
            Imgproc.circle(input, upperRightCorner,2, new Scalar(0,255,0),2);
            Imgproc.circle(input, lowerLeftCorner,2, new Scalar(0,255,0),2);
            Imgproc.circle(input, lowerRightCorner,2, new Scalar(0,255,0),2);

            Imgproc.circle(input, upperMiddle,2, new Scalar(255,0,0),2);
            Imgproc.circle(input, lowerMiddle,2, new Scalar(255,0,0),2);

            //draw center line
            Imgproc.line(input, lowerMiddle, upperMiddle,  new Scalar(255,0,0));

        }


//        telemetry.addData("Goal Visibility", isGoalVisible());
//        telemetry.addData("Distance (in)", getXDistance());
////        telemetry.addData("Goal Height (px)", getGoalHeight());
////        telemetry.addData("Goal Pitch (degrees)", getPitch());
//        telemetry.addData("Goal Yaw (degrees)", getYaw());
////        telemetry.addData("Width:", input.width());
////        telemetry.addData("Height:", input.height());
//        telemetry.addData("Motor Power", getMotorPower());
//        telemetry.addData("At Set Point", isGoalCentered());




//        telemetry.update();


        //Return MaskFrame for tuning purposes
//        return MaskFrame;
        if(viewfinderIndex % 2 == 0){
            return input;
        } else {
            return MaskFrame;
        }
    }


    public boolean filterContours(MatOfPoint contour) {
        return Imgproc.contourArea(contour) > CONTOUR_AREA_MIN;
    }

    public Pose2d getFieldPositionFromGoal() {
        return new Pose2d(72 - getDistanceToGoalWall(), 35.25 + 4 - getDistanceToGoalWall() / Math.tan(Math.toRadians(90 + getYaw())));
    }




    //helper method to check if rect is found
    public boolean isGoalVisible(){
        return blueRect != null;
    }

    public boolean isGoalCentered() {
        if (!isGoalVisible()) {
            return false;
        }
        return headingController.atSetPoint();
    }

    public boolean isCornersVisible(){
        return upperLeftCorner != null && upperRightCorner != null && lowerLeftCorner != null && lowerRightCorner != null && upperMiddle != null && lowerMiddle != null;
    }

//    public double getDistanceToGoalWall(){
//        if (!isGoalVisible()){
//            return 0;
//        }
//        return (3869/getGoalHeight()) + -12.1;
//    }

    public double getDistanceToGoalWall() {
        return (HIGH_GOAL_CENTER_HEIGHT - CAMERA_HEIGHT) / Math.tan(Math.toRadians(getPitch()));
    }


    public double[] getPowerShotAngles(double distanceFromGoalCenter, double distanceToGoalWall) {
        double distanceFromFirstPowerShot = 16.0 - distanceFromGoalCenter - HIGH_GOAL_SETPOINT;
        double distanceFromSecondPowerShot = 7.5 + 16.0 - distanceFromGoalCenter - HIGH_GOAL_SETPOINT;
        double distanceFromThirdPowerShot = 7.5 + 7.5 + 16.0 - distanceFromGoalCenter - HIGH_GOAL_SETPOINT;

        double angleToFirstPowerShot = Math.toDegrees(Math.atan(distanceFromFirstPowerShot / distanceToGoalWall));
        double angleToSecondPowerShot = Math.toDegrees(Math.atan(distanceFromSecondPowerShot / distanceToGoalWall));
        double angleToThirdPowerShot = Math.toDegrees(Math.atan(distanceFromThirdPowerShot / distanceToGoalWall));

        double[] result = {-angleToFirstPowerShot, -angleToSecondPowerShot, -angleToThirdPowerShot};

        return result;

    }


    //    "Real Height" gives returns an accurate goal height in pixels even when goal is viewed at an angle by calculating distance between middle two points
    public double getGoalHeight(){
        if (!isGoalVisible() && isCornersVisible()){
            return 0;
        }
        return Math.sqrt(Math.pow(Math.abs(lowerMiddle.y - rightMiddle.y), 2) + Math.pow(Math.abs(lowerMiddle.x - rightMiddle.x), 2));
    }

    public double getYaw() {
        if (!isGoalVisible()){
            return 0;
        }

        double targetCenterX = getCenterofRect(blueRect).x;

        return Math.toDegrees(
                Math.atan((targetCenterX - (centerX + CENTER_X_OFFSET)) / horizontalFocalLength) + CAMERA_YAW_OFFSET
        );
    }


    public double getPitch() {
        if (!isGoalVisible()){
            return 0;
        }

        double targetCenterY = getCenterofRect(blueRect).y;

        return -Math.toDegrees(
                Math.atan(((targetCenterY - centerY + CENTER_Y_OFFSET)) / verticalFocalLength) // should be right idk
        ) + CAMERA_PITCH_OFFSET;
        // this.imageHeight / (2 * Math.tan(fov(55) / 2) * (verticalRatio(3) / diagonalAspect(5)))
    }

    public double getMotorPower() {
        //set heading controller
        headingController.setSetPoint(HIGH_GOAL_SETPOINT);
        headingController.setTolerance(TOLERANCE);
        headingController.setPID(kP, kI, kD);
        double motorPower = headingController.calculate(getYaw());

        if (motorPower > 0 && motorPower < minimumMotorPower)
            motorPower += minimumMotorPower;
        else if (motorPower < 0 && motorPower > -minimumMotorPower)
            motorPower -= minimumMotorPower;

        return Range.clip(motorPower, -1.0, 1.0);
    }


//    public double getShotMotorPower(double degree) {
//        double motorPower = headingController.calculate(degree);
//
//        if (motorPower > 0 && motorPower < minimumMotorPower)
//            motorPower += minimumMotorPower;
//        else if (motorPower < 0 && motorPower > -minimumMotorPower)
//            motorPower -= minimumMotorPower;
//
//        return UtilMethods.ensureRange(motorPower, -1.0, 1.0);
//    }


    /*
     * Changing the displayed color space on the viewport when the user taps it
     */
    @Override
    public void onViewportTapped() {
        viewfinderIndex++;
    }

    public void findCorners(){
        RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(biggestBlueContour.toArray()));

        Point[] corners = new Point[4];
        rect.points(corners);

        if (inRange(rect.angle, -90.9, -45)){
            lowerLeftCorner = corners[1];
            upperLeftCorner = corners[2];
            upperRightCorner = corners[3];
            lowerRightCorner = corners[0];
        } else {
            lowerLeftCorner = corners[0];
            upperLeftCorner = corners[1];
            upperRightCorner = corners[2];
            lowerRightCorner = corners[3];
        }


        //Calculate "middle points" for true height calculation
        upperMiddle = new Point((upperLeftCorner.x + upperRightCorner.x) / 2, (upperLeftCorner.y + upperRightCorner.y) / 2);
        lowerMiddle = new Point((lowerLeftCorner.x + lowerRightCorner.x) / 2, (lowerLeftCorner.y + lowerRightCorner.y) / 2);

        leftMiddle = new Point((upperLeftCorner.x + lowerLeftCorner.x) / 2, (upperLeftCorner.y + lowerLeftCorner.y) / 2);
        rightMiddle = new Point((upperRightCorner.x + lowerRightCorner.x) / 2, (upperRightCorner.y + lowerRightCorner.y) / 2);

    }

    public Point getCenterofRect(Rect rect) {
        if (rect == null) {
            return new Point(centerX, centerY);
        }
        return new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
    }


    public static boolean inRange(double number, double min, double max) {
        return number >= min && number <= max;
    }

}
