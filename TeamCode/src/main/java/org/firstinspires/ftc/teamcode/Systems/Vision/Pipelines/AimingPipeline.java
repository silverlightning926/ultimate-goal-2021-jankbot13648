package org.firstinspires.ftc.teamcode.Systems.Vision.Pipelines;

import org.opencv.core.*;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.imgproc.Imgproc;

import java.util.*;

public class AimingPipeline extends OpenCvPipeline {
    public enum Color {
        BLUE, RED
    }

    public static Scalar HSV_LOWER_BLUE = new Scalar(100, 70, 50);
    public static Scalar HSV_UPPER_BLUE = new Scalar(130, 255, 255);

    public static Scalar HSV_LOWER_RED = new Scalar(170, 70, 50);
    public static Scalar HSV_UPPER_RED = new Scalar(50, 255, 255);

    private static Mat ERODE_ELEMENT;
    private static Mat DIALATE_ELEMENT;

    public static int MIN_Y = 100;
    public static int MAX_Y = 200;

    // width / height ratio
    public static double MIN_POWERSHOT_BOX_RATIO = 1.2;
    public static double MAX_POWERSHOT_BOX_RATIO = 10.0;

    // width / height ratio
    public static double MIN_GOAL_BOX_RATIO = 1.0 / 4.0;
    public static double MAX_GOAL_BOX_RATIO = 1.0;

    public static double MIN_POWERSHOT_WIDTH = 2.0;
    public static double MAX_POWERSHOT_WIDTH = 10.0;

    public static double MIN_GOAL_WIDTH = 30.0;
    public static double MAX_GOAL_WIDTH = 160.0;

    public static double X_SPACING_TOLERANCE = 0.15;

    // max difference in widths between powershots abs(h1 - h2) / (h1 + h2)
    public static double MAX_WIDTH_RATIO = 0.25;

    // y spacing difference / x spacing
    public static double MAX_SPACING_RATIO = 0.1;
    public static double MAX_COMBINED_SPACING_RATIO = 0.2;

    // spacing between powershots
    public static double MIN_POWERSHOT_SPACING = 10.0;

    private Mat filteredMat = new Mat();
    private Mat hsvMat = new Mat();
    private Mat rangeMat = new Mat();
    private Mat range1Mat = new Mat();
    private Mat range2Mat = new Mat();
    private Mat denoisedMat = new Mat();
    private Mat contourMat = new Mat();
    private Mat finalMat = new Mat();
    private Mat hierarchy = new Mat();

    private List<MatOfPoint> contours = new ArrayList<>();
    private List<RotatedRect> powershotRects = new ArrayList<>();

    // static so they're controllable through dashboard
    public static int stageNum = 0;
    public static Color color = Color.RED;

    private double goalCenterX;
    private double powershot1CenterX;
    private double powershot2CenterX;
    private double powershot3CenterX;

    public AimingPipeline() {
        super();

        ERODE_ELEMENT = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(1, 3));
        DIALATE_ELEMENT = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 3));
    }

    public void setColor(Color color) {
        AimingPipeline.color = color;
    }

    private Size adjustedRotatedRectSize(RotatedRect rotatedRect) {
        Size size = rotatedRect.size;
        if (rotatedRect.angle < -45) {
            return new Size(size.height, size.width);
        } else {
            return size;
        }
    }

    private Point[] getVertices(RotatedRect rotatedRect) {
        Point[] vertices = new Point[4];
        rotatedRect.points(vertices);
        return vertices;
    }

    private void drawRotatedRect(Mat out, RotatedRect rotatedRect, Scalar color, int thickness) {
        Point[] vertices = getVertices(rotatedRect);
        List<MatOfPoint> boxContours = new ArrayList<>();
        boxContours.add(new MatOfPoint(vertices));
        Imgproc.drawContours(out, boxContours, -1, color, thickness);
    }

    @Override
    public void onViewportTapped() {
        stageNum++;
    }

    @Override
    public Mat processFrame(Mat input) {
        contours.clear();
        powershotRects.clear();

        input.copyTo(finalMat);
        input.copyTo(contourMat);


        Imgproc.line(finalMat, new Point(0, MIN_Y), new Point(finalMat.width(), MIN_Y), new Scalar(255.0, 165.0, 0.0), 2);
        Imgproc.line(finalMat, new Point(0, MAX_Y), new Point(finalMat.width(), MAX_Y), new Scalar(255.0, 165.0, 0.0), 2);

        // initial noise reduction, bilateral is good for noise reduction while preserving boundaries
        //Imgproc.bilateralFilter(input, filteredMat, 5, 100, 100);
        input.copyTo(filteredMat);

        // rgb -> hsv
        Imgproc.cvtColor(filteredMat, hsvMat, Imgproc.COLOR_RGB2HSV);

        Scalar hsvLower = color == Color.BLUE ? HSV_LOWER_BLUE : HSV_LOWER_RED;
        Scalar hsvUpper = color == Color.BLUE ? HSV_UPPER_BLUE : HSV_UPPER_RED;

        // hsv filtering
        if (hsvLower.val[0] > hsvUpper.val[0]) {
            // this "wraps" the hue range (channel 0) into a two part range on both extremes of the h spectrum
            Core.inRange(hsvMat, hsvLower, new Scalar(180.0, hsvUpper.val[1], hsvUpper.val[2]), range1Mat);
            Core.inRange(hsvMat, new Scalar(0.0, hsvLower.val[1], hsvLower.val[2]), hsvUpper, range2Mat);
            Core.bitwise_or(range1Mat, range2Mat, rangeMat);
        } else {
            Core.inRange(hsvMat, hsvLower, hsvUpper, rangeMat);
        }

        // erode + dilate remove small areas and fill gaps
        Imgproc.erode(rangeMat, denoisedMat, ERODE_ELEMENT, new Point(-1, -1), 2);
        Imgproc.dilate(denoisedMat, denoisedMat, DIALATE_ELEMENT, new Point(-1, -1), 2);

        // finds contours in the filtered mat
        Imgproc.findContours(denoisedMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(contourMat, contours, -1, new Scalar(0.0, 255.0, 0.0), 3);

        double maxHighgoalWidthDetected = 0;

        for (MatOfPoint c : contours) {
            // fit rotated rect to contour
            MatOfPoint2f points = new MatOfPoint2f(c.toArray());
            RotatedRect rect = Imgproc.minAreaRect(points);
            points.release();

            Size rectSize = adjustedRotatedRectSize(rect);
            double ratio = rectSize.height / rectSize.width;

            // check if the ratio of height to width are within the defined range and that the width is within range
            if (ratio > MIN_POWERSHOT_BOX_RATIO
                    && ratio < MAX_POWERSHOT_BOX_RATIO
                    && rectSize.width > MIN_POWERSHOT_WIDTH
                    && rectSize.width < MAX_POWERSHOT_WIDTH
                    && rect.center.y >= MIN_Y
                    && rect.center.y <= MAX_Y) {
                drawRotatedRect(finalMat, rect, new Scalar(0.0, 255.0, 0.0), 3);
                powershotRects.add(rect);
            }

            if (ratio > MIN_GOAL_BOX_RATIO
                    && ratio < MAX_GOAL_BOX_RATIO
                    && rectSize.width > MIN_GOAL_WIDTH
                    && rectSize.width < MAX_GOAL_WIDTH
                    && rect.center.y >= MIN_Y
                    && rect.center.y <= MAX_Y
                    && rectSize.width > maxHighgoalWidthDetected) {
                drawRotatedRect(finalMat, rect, new Scalar(255.0, 0.0, 255.0), 3);
                goalCenterX = rect.center.x;
                maxHighgoalWidthDetected = rectSize.width;
            }
            c.release();
        }

        // need to see at least 3 potential powershots
        if (powershotRects.size() >= 3) {
            // sort contours left to right
            powershotRects.sort(Comparator.comparingDouble(a -> a.center.x));

            // initialize search params
            double minDetectedSpacing = Double.MAX_VALUE;
            RotatedRect match1 = powershotRects.get(0);
            RotatedRect match2 = powershotRects.get(1);
            RotatedRect match3 = powershotRects.get(2);

            // process to check all combinations of three rects that meet certain criteria and minimize
            for (int i = 0; i < powershotRects.size(); i++) {
                for (int j = i + 1; j < powershotRects.size(); j++) {
                    for (int k = j + 1; k < powershotRects.size(); k++) {
                        RotatedRect rectA = powershotRects.get(i);
                        RotatedRect rectB = powershotRects.get(j);
                        RotatedRect rectC = powershotRects.get(k);

                        // point zero is always the bottom
                        Point bottomA = getVertices(rectA)[0];
                        Point bottomB = getVertices(rectB)[0];
                        Point bottomC = getVertices(rectC)[0];

                        Size sizeA = adjustedRotatedRectSize(rectA);
                        Size sizeB = adjustedRotatedRectSize(rectB);
                        Size sizeC = adjustedRotatedRectSize(rectC);

                        double spacingXAB = rectB.center.x - rectA.center.x;
                        double spacingXBC = rectC.center.x - rectB.center.x;

                        double spacingYAB = bottomB.y - bottomA.y;
                        double spacingYBC = bottomC.y - bottomB.y;

                        double spacingXCombined = spacingXAB + spacingXBC;
                        double spacingYCombined = Math.abs(spacingYAB) + Math.abs(spacingYBC);
                        double spacingYDifference = Math.abs(spacingYAB - spacingYBC);

                        // spacing in the x direction should be similar between each pair
                        if (Math.abs(spacingXAB - spacingXBC) / spacingXCombined < X_SPACING_TOLERANCE
                                // difference in y spacing between each pair should be low relative to the x spacing
                                && Math.abs(sizeA.width - sizeB.width) / (sizeA.width + sizeB.width) < MAX_WIDTH_RATIO
                                && Math.abs(sizeB.width - sizeC.width) / (sizeB.width + sizeC.width) < MAX_WIDTH_RATIO
                                && Math.abs(sizeA.width - sizeC.width) / (sizeA.width + sizeC.width) < MAX_WIDTH_RATIO
                                // the widths of each rect should be similar
                                && spacingYDifference / spacingXCombined < MAX_SPACING_RATIO
                                // the ratio between y and x spacing should be less than a max value
                                && spacingYCombined / spacingXCombined < MAX_COMBINED_SPACING_RATIO
                                // x spacing should be within a min value
                                && spacingXCombined > MIN_POWERSHOT_SPACING * 2.0
                                // find the combination that fits the previous criteria and minimizes overall spacing
                                && spacingXCombined + spacingYCombined < minDetectedSpacing) {
                            match1 = rectA;
                            match2 = rectB;
                            match3 = rectC;
                            minDetectedSpacing = spacingXCombined + spacingYCombined;
                        }
                    }
                }
            }

            if (minDetectedSpacing != Double.MAX_VALUE){
                powershot1CenterX = match1.center.x;
                powershot2CenterX = match2.center.x;
                powershot3CenterX = match3.center.x;

                drawRotatedRect(finalMat, match1, new Scalar(255.0, 0.0, 0.0), 3);
                drawRotatedRect(finalMat, match2, new Scalar(255.0, 0.0, 0.0), 3);
                drawRotatedRect(finalMat, match3, new Scalar(255.0, 0.0, 0.0), 3);
            }
        }

        Mat[] mats = new Mat[] { finalMat, input, filteredMat, rangeMat, denoisedMat, contourMat };
        return mats[stageNum % mats.length];
    }

    public double getGoalCenterX() {
        return goalCenterX;
    }

    public double[] getPowershotsCenterX() {
        return new double[] { powershot1CenterX, powershot2CenterX, powershot3CenterX };
    }
}
