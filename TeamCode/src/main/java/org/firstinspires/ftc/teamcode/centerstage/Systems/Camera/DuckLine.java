package org.firstinspires.ftc.teamcode.centerstage.Systems.Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Range;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class DuckLine extends OpenCvPipeline {
    private static AutoFlow.PropPos propPos;
    public int width;
    public int height;
    volatile public double propPosX;
    Mat hsv;
    Mat mask;
    Mat subMat;
    Rect subRect;
    AutoFlow.Alliance alliance;
    Telemetry telemetry;
    Scalar min_red = new Scalar(0, 100, 100);
    Scalar max_red = new Scalar(10, 255, 255);
    Scalar min_blue = new Scalar(90, 100, 100);
    Scalar max_blue = new Scalar(130, 255, 255);
    volatile private Point targetPos = null;
    //Rect(x, y, width, height)             x, y - position. width, height - dimensions.
    volatile private Rect targetRect = null;
    private double divider_bottom_middle;
    private double divider_middle_top;

    public DuckLine(AutoFlow.Alliance alliance, Telemetry telemetry) {
        this.alliance = alliance;
        this.telemetry = telemetry;
    }

    public static AutoFlow.PropPos getPropPos() {
        return propPos;
    }

//	public ABC getDuck(int screenWidth) {
//		if (targetPos.x <= screenWidth / 3) { // First 3rd of the screen
//			return ABC.RIGHT;
//		} else if (targetPos.x <= screenWidth / 3 * 2) { // Second 3rd of the screen
//			return ABC.MID;
//		} else if (targetPos.x <= screenWidth) { // Detects on the whole screen
//			return ABC.LEFT;
//		} else {
//			return null;
//		}
//	}

    public Point getTargetPos() {
        return targetPos;
    }

    public Rect getTargetRect() {
        return targetRect;
    }

    public SH_Levels getDuck() {
        if (targetPos == null) return null;
        else {
            if (targetPos.x <= this.divider_bottom_middle) {
                return SH_Levels.Bottom;
            } else if (targetPos.x >= this.divider_middle_top) {
                return SH_Levels.Top;
            } else {
                return SH_Levels.Middle;
            }
        }
    }

    @Override
    public void init(Mat firstFrame) {
        super.init(firstFrame);

        mask = new Mat();
        hsv = new Mat();
        Imgproc.cvtColor(firstFrame, hsv, Imgproc.COLOR_RGB2HSV);

        width = hsv.width();
        height = hsv.height();
        subRect = new Rect(new Point(width * 2 / 5, height * 5 / 12), new Point(width * 3 / 5, height * 6 / 12));
        subMat = hsv.submat(subRect);

        this.divider_bottom_middle = width / 3.0;
        this.divider_middle_top = (width / 3.0) * 2;
    }

    /**
     * Processes a frame, detects and highlights the target rectangle,
     * and displays x-coordinates next to the rectangles.
     *
     * @param frame The input frame.
     * @return The processed frame.
     */
    @Override
    public Mat processFrame(Mat frame) {
        Mat smallFrame = new Mat(frame, new Range(frame.height() / 2, frame.height()));
        if (alliance == AutoFlow.Alliance.RED)
            Core.flip(smallFrame, smallFrame, 1);
        //        Mat smallFrame = frame;
        Imgproc.cvtColor(smallFrame, hsv, Imgproc.COLOR_RGB2HSV);  // Convert to HSV color set
        Scalar min_ = this.alliance == AutoFlow.Alliance.BLUE ?
                min_blue : min_red, max_ = this.alliance == AutoFlow.Alliance.BLUE ? max_blue : max_red;

        Core.inRange(hsv, min_, max_, mask);   // Mask all orange
//        Scalar min_yellow = new Scalar(8, 130, 160);
//        Scalar max_yellow = new Scalar(60, 255, 255);
//        Core.inRange(hsv, min_yellow, max_yellow, mask);   // Mask all orange
        Mat kernel = Mat.ones(5, 5, CvType.CV_8UC1);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel, new Point(2, 2));

        smallFrame.setTo(new Scalar(0, 0, 0), mask);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        ArrayList<Rect> rects = new ArrayList<>();
        Rect rect;
        double minArea = 600;

        // Find and store rectangles
        for (int i = 0; i < contours.size(); i++) {
            Mat contour = contours.get(i);
            double contourArea = Imgproc.contourArea(contour);
            if (contourArea > minArea) {
                rect = Imgproc.boundingRect(contours.get(i));
                rects.add(rect);
            }
        }

        int biggestIndex = 0;
        double biggestArea = 0;
        double highestY = 0;
        // Find the biggest rectangle
        for (int i = 0; i < rects.size(); i++) {
            Rect curr = rects.get(i);
            if (curr.area() > biggestArea) {
                biggestIndex = i;
                biggestArea = curr.area();
                highestY = curr.y + curr.height;
            }
        }

        if (rects.size() > 0) {
            targetRect = rects.get(biggestIndex);

            int width = targetRect.width;
            int height = targetRect.height;

            int centerX = targetRect.x + width / 2;
            int centerY = targetRect.y + height / 2;

            Point pt1 = new Point(targetRect.x, targetRect.y);
            Point pt2 = new Point(targetRect.x + width, targetRect.y + height);
            Imgproc.rectangle(smallFrame, pt1, pt2, new Scalar(255, 0, 0), 2);

            targetPos = new Point(centerX, centerY);

            // Draw x-coordinate next to the rectangle
            String xCoordinateText = "X: " + centerX + ", " + "Y: " + targetRect.y + height;
            propPosX = centerX;
            if (propPosX <= 200) {
                propPos = AutoFlow.PropPos.LEFT;
            } else if (propPosX >= 430) propPos = AutoFlow.PropPos.RIGHT;
            else {
                propPos = AutoFlow.PropPos.MID;
            }
            AutoFlow.telemetry.addData("PROP POS", propPos);
            AutoFlow.telemetry.addData("PROP X POS", propPosX);
            AutoFlow.telemetry.addData("PROP Y POS", targetRect.y + height);
            AutoFlow.telemetry.addData("PROP HEIGHT", height);
            AutoFlow.telemetry.addData("PROP WIDTH", width);
            AutoFlow.telemetry.addData("PROP AREA", biggestArea);
            AutoFlow.telemetry.update();
            xCoordinateText += "L: " + propPosX;
            Point textPoint = new Point(targetRect.x - 30, targetRect.y - 10); // Adjust the position as needed
            Imgproc.putText(smallFrame, xCoordinateText, textPoint, Imgproc.FONT_HERSHEY_PLAIN, 1, new Scalar(0, 100, 100), 1);
        } else {
            targetPos = null;
            targetRect = null;
        }

//        // Draw lines on the frame
        Imgproc.line(smallFrame, new Point(this.divider_bottom_middle, 0), new Point(this.divider_bottom_middle, this.height), new Scalar(86, 123, 47));
        Imgproc.line(smallFrame, new Point(this.divider_middle_top, 0), new Point(this.divider_middle_top, this.height), new Scalar(86, 123, 47));
        return smallFrame;
    }

    // SH - Shipping Hub
    public enum SH_Levels {
        Top, Middle, Bottom, Collect, CollectAuto, TopTeleop, ReleaseShared, EndGamePark
    }

}
