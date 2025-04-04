package org.firstinspires.ftc.teamcode.sys;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.sys.RectDrawer;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

@Config
public class SampleOrientationProcessor implements VisionProcessor {
    public enum SampleColor {
        YELLOW(),
        BLUE(),
        RED();
    }

    private Mat frame;

    private Telemetry telemetry;

    public static Scalar lowerYellow = new Scalar(19.0, 102.0, 130.1); // hsv
    public static Scalar upperYellow = new Scalar(30.0, 255.0, 255.0); // hsv
    public static Scalar lowerBlue = new Scalar(90.0, 90.0, 90.0); // hsv
    public static Scalar upperBlue = new Scalar(120.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedH = new Scalar(10.0, 0.0, 0.0); // hsv
    public static Scalar upperRedH = new Scalar(170.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedSV = new Scalar(0.0, 130.0, 100.0); // hsv
    public static Scalar upperRedSV = new Scalar(255.0, 255.0, 255.0); // hsv

    private double sampleAngle = 0;
    private double averageBrightness = 0;

    private double sample_position = 0;

    public static RectDrawer.SampleColor colorType = RectDrawer.SampleColor.YELLOW;

    private ArrayList<RotatedRect> rects;

    public SampleOrientationProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        frame = input;
        Mat hsv = new Mat(); // convert to hsv
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Mat gray = new Mat(); // convert to hsv
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

        // Color threshold
        Mat inRange = new Mat();
//        Core.inRange(hsv, PixelColor.YELLOW.LOWER, PixelColor.YELLOW.UPPER, inRange);
        if (colorType.equals(RectDrawer.SampleColor.BLUE)) {
            Core.inRange(hsv, lowerBlue, upperBlue, inRange);
        } else if (colorType.equals(RectDrawer.SampleColor.RED)) {
            Mat inHRange = new Mat();
            Mat inSVRange = new Mat();
            Core.inRange(hsv, lowerRedH, upperRedH, inHRange);
            Core.bitwise_not(inHRange, inHRange);
            Core.inRange(hsv, lowerRedSV, upperRedSV, inSVRange);
            Core.bitwise_and(inHRange, inSVRange, inRange);
        } else {
            Core.inRange(hsv, lowerYellow, upperYellow, inRange);
        }

        // Morphology
        Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(25, 25));
        Mat kernel2 = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(10, 10));

//        Imgproc.erode(inRange, inRange, kernel);
//        Imgproc.dilate(inRange, inRange, kernel2);

        // Find all contours
        List<MatOfPoint> unfilteredContours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(inRange, unfilteredContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter contours by size and get rotated rects
        int minArea = 2500;
        ArrayList<RotatedRect> rotatedRects = new ArrayList<>();
        List<MatOfPoint> filteredContours = new ArrayList<>();
        for (MatOfPoint contour : unfilteredContours) {
            RotatedRect minAreaRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            double area = minAreaRect.size.area();
            if (area > minArea) {
                filteredContours.add(contour);
                rotatedRects.add(minAreaRect);
            }
        }
        Imgproc.drawContours(frame, filteredContours, -1, new Scalar(0, 255, 255), 2);

        // Get overlapping rotated rect groups
        double overlapThreshold = 0.2; // % of smaller box covered
        Set<Integer> toSkip = new HashSet<>();
        ArrayList<ArrayList<Double[]>> overlapGroups = new ArrayList<>();
        for (int i = 0; i < rotatedRects.size(); i++) {
            if (toSkip.contains(i)) continue;
            toSkip.add(i);
            ArrayList<Double[]> overlapGroup = new ArrayList<>();
            double iArea = rotatedRects.get(i).size.area();
            overlapGroup.add(new Double[]{(double)i, iArea});
            for (int j = i+1; j < rotatedRects.size(); j++) {
                if (toSkip.contains(j)) continue;
                double jArea = rotatedRects.get(j).size.area();
                for (Double[] rect : overlapGroup) {
                    double overlapArea = getIntersectionArea(rotatedRects.get(rect[0].intValue()), rotatedRects.get(j));
                    if (overlapArea / Math.min(rect[1], jArea) >= overlapThreshold) {
                        overlapGroup.add(new Double[]{(double)j, jArea});
                        toSkip.add(j);
                        break;
                    }
                }
            }
            overlapGroups.add(overlapGroup);
        }

        // telemetry
        ArrayList<ArrayList<Double>> overlapGroups2 = new ArrayList<>();
        for (ArrayList<Double[]> overlapGroup : overlapGroups) {
            overlapGroups2.add(new ArrayList<>());
            for (Double[] index : overlapGroup) {
                overlapGroups2.get(overlapGroups2.size()-1).add(index[0]);
            }
        }
        telemetry.addData("overlapGroups", overlapGroups2);

        // Filter out overlapping rotated rects
        ArrayList<RotatedRect> filteredRects = new ArrayList<>();
        for (ArrayList<Double[]> overlapGroup : overlapGroups) {
            int maxIndex = overlapGroup.get(0)[0].intValue();
            double maxArea = overlapGroup.get(0)[1];
            for (Double[] rect : overlapGroup) {
                if (rect[1] > maxArea) {
                    maxArea = rect[1];
                    maxIndex = rect[0].intValue();
                }
            }
            filteredRects.add(rotatedRects.get(maxIndex));
        }

        telemetry.addData("filteredRects.size()", filteredRects.size());


        // Draw unfiltered rects as blue
        for (RotatedRect rotatedRect : rotatedRects) {
            Point[] vertices = new Point[4];
            rotatedRect.points(vertices);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(frame, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 0, 255), 2);
            }
        }

        // Draw filtered rects as green
        for (RotatedRect rotatedRect: filteredRects) {
            Point[] vertices = new Point[4];
            rotatedRect.points(vertices);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(frame, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 2);
            }
            Imgproc.circle(frame, rotatedRect.center, 5, new Scalar(255, 0, 0));
        }
        rects = new ArrayList<RotatedRect>(filteredRects);


        // telemetry
        if (!filteredRects.isEmpty()) {
            telemetry.addData("width ", filteredRects.get(0).size.width);
            telemetry.addData("height ", filteredRects.get(0).size.height);
            telemetry.addData("angle ", filteredRects.get(0).angle);
            telemetry.addData("center ", filteredRects.get(0).center);

            FtcDashboard.getInstance().getTelemetry().addData("width ", filteredRects.get(0).size.width);
            FtcDashboard.getInstance().getTelemetry().addData("height ", filteredRects.get(0).size.height);
            FtcDashboard.getInstance().getTelemetry().addData("angle ", filteredRects.get(0).angle);
            FtcDashboard.getInstance().getTelemetry().addData("center ", filteredRects.get(0).center);
            FtcDashboard.getInstance().getTelemetry().addData("sample position x", filteredRects.get(0).center.x);
            FtcDashboard.getInstance().getTelemetry().addData("sample position y", filteredRects.get(0).center.y);
            FtcDashboard.getInstance().getTelemetry().update();
            double procAngle = filteredRects.get(0).angle;

            double sample_position_x = 0;
            double sample_position_y = 0;
            double screen_width = 640;
            double screen_hight = 480;
            sample_position_y = screen_hight / 2;
            sample_position_x = screen_width /2;
            double sample_finalx = filteredRects.get(0).center.x - sample_position_x, sample_finaly = filteredRects.get(0).center.y - sample_position_y;
            if (filteredRects.get(0).size.width > filteredRects.get(0).size.height)
                procAngle *= -1;
            else
                procAngle = 90-procAngle;
            telemetry.addData("procAngle ", procAngle);
            FtcDashboard.getInstance().getTelemetry().addData("rawAngle", filteredRects.get(0).angle);
            sampleAngle = Math.toRadians(procAngle);
        }
        telemetry.addData("sampleAngle", sampleAngle);


        // Getting representative brightness of image
        MatOfDouble muMat = new MatOfDouble();
        MatOfDouble sigmaMat = new MatOfDouble();
        Core.meanStdDev(gray, muMat, sigmaMat);

        double mu = muMat.get(0,0)[0];
        double sigma = sigmaMat.get(0,0)[0];
        double k = 1;
        Scalar lowerBound = new Scalar(mu-k*sigma);
        Scalar upperBound = new Scalar(mu+k*sigma);

        Mat mask = new Mat();
        Core.inRange(gray, lowerBound, upperBound, mask);
        Scalar maskedMean = Core.mean(gray, mask);
        double averageInRange = maskedMean.val[0];

        averageBrightness = averageInRange;
        telemetry.addData("averageBrightness", averageBrightness);


        telemetry.update();



        return frame;
    }

    public double getSampleAngle() {
        return sampleAngle;
    }

    public double getAverageBrightness() {
        return averageBrightness;
    }

    private double getIntersectionArea(RotatedRect rect1, RotatedRect rect2) {
        // Get vertices of the rectangles
        Point[] vertices1 = new Point[4];
        rect1.points(vertices1);

        Point[] vertices2 = new Point[4];
        rect2.points(vertices2);

        // Convert vertices arrays to MatOfPoint2f
        MatOfPoint2f poly1 = new MatOfPoint2f(vertices1);
        MatOfPoint2f poly2 = new MatOfPoint2f(vertices2);

        // Output MatOfPoint2f for the intersection polygon
        MatOfPoint2f intersection = new MatOfPoint2f();

        // Calculate intersection
        return Imgproc.intersectConvexConvex(poly1, poly2, intersection, true);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public ArrayList<Point> getOffsets() { // scaled to inches
        ArrayList<RotatedRect> input = new ArrayList<RotatedRect>(rects); // already filtered by color
        ArrayList<Point> output = new ArrayList<Point>();
        double cameraAngle = 0;

        double height = 3.0; // in inches


        double canvasVertical = height*3.0/8.0; // inches
        double canvasHorizontal = height / 2.0;

        for (RotatedRect i : input) {
            // real center is (320, 480), positive direction is right and down
//            output.add(new Point(i.center.x - 320, -(i.center.y - 240)));
            output.add(new Point((i.center.x - 320) / 320 * canvasHorizontal, -(i.center.y - 240) / 240 * canvasVertical));


            // 4 in height = 1.5 width vertical (half width, not full)
            // 6 : 2.25
            // 2 : 0.75
            // 8 : 3
            // horizontal: 8 / 4
        }


        return output;
    }
}