package org.firstinspires.ftc.teamcode.power_play.util;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
public class SleevePipeline extends OpenCvPipeline {
    public static volatile int startRow = 180;
    public static volatile int endRow = 280;
    public static volatile int startCol = 260;
    public static volatile int endCol = 380;

    // These coordinates are after the frame is `frame = submat`
    public static volatile int middleX = 40;
    public static volatile int middleY = 40;
    public static volatile int middleW = 20;
    public static volatile int middleH = 20;

    //    public static volatile int minh1, maxh1, minh2, maxh2, minh3, maxh3;
    public static volatile int ming = 24, maxg = 40; // Green
    public static volatile int minp = 145, maxp = 175; // Pink
    public static volatile int minb = 90, maxb = 115; // Blue

    // Orange
//	public static volatile Scalar min_color1 = new Scalar(10, 125, 125);
//	public static volatile Scalar max_color1 = new Scalar(30, 255, 255);

    // Purple / Pink
//    public static volatile Scalar min_color2 = new Scalar(145, 125, 125);
//    public static volatile Scalar max_color2 = new Scalar(175, 255, 255);

    // Green
//    public static volatile Scalar min_color3 = new Scalar(40, 125, 125);
//    public static volatile Scalar max_color3 = new Scalar(75, 255, 255);

    public enum ParkingLocation {
        None,
        One,
        Two,
        Three,
    }

    private volatile ParkingLocation parkingLocation = null;

    public ParkingLocation getParkingLocation() {
        return this.parkingLocation;
    }

    private Mat hsv = null;
    private Mat thresholded = null;

    @Override
    public void init(Mat firstFrame) {
        super.init(firstFrame);

        int newWidth = endCol - startCol;
        int newHeight = endRow - startRow;

        hsv = new Mat(newHeight, newWidth, CvType.CV_8UC3);
        thresholded = new Mat(newHeight, newWidth, CvType.CV_8UC1);
    }

    private MatOfPoint processColor(Scalar min, Scalar max) {
        Core.inRange(hsv, min, max, thresholded);
        Imgproc.morphologyEx(thresholded, thresholded, Imgproc.MORPH_CLOSE, Mat.ones(5, 5, CvType.CV_8UC1));

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.size() > 0) {
            double max_area = 0;
            MatOfPoint max_contour = null;
            for (MatOfPoint contour : contours) {
                if (contour == null) continue;

                double area = Imgproc.contourArea(contour);
                if (area > max_area) {
                    max_area = area;
                    max_contour = contour;
                }
            }
            return max_contour;
        }
        return null;
    }

    @Override
    public Mat processFrame(Mat frame) {
        try {
            Mat subFrame = frame.submat(startRow, endRow, startCol, endCol);

            Imgproc.cvtColor(subFrame, hsv, Imgproc.COLOR_RGB2HSV);

//			MatOfPoint contour1 = processColor(new Scalar(ming, 75, 100), new Scalar(maxg, 255, 255)); // PREVIOUSLY 75, 100 -> 255, 255
//			MatOfPoint contour2 = processColor(new Scalar(minp, 125, 125), new Scalar(maxp, 255, 255));// PREVIOUSLY 125, 125 -> 255, 255
//			MatOfPoint contour3 = processColor(new Scalar(minb, 100, 100), new Scalar(maxb, 255, 255)); // PREVIOUSLY 100, 100 -> 255, 255
            MatOfPoint contour1 = processColor(new Scalar(60, 80, 100), new Scalar(90, 255, 255)); // PREVIOUSLY 75, 100 -> 255, 255
            MatOfPoint contour2 = processColor(new Scalar(145, 80, 125), new Scalar(175, 255, 255));// PREVIOUSLY 125, 125 -> 255, 255
            MatOfPoint contour3 = processColor(new Scalar(5, 100, 160), new Scalar(30, 255, 255)); // PREVIOUSLY 100, 100 -> 255, 255

            double color1Area = contour1 == null ? 0 : Imgproc.contourArea(contour1);
            double color2Area = contour2 == null ? 0 : Imgproc.contourArea(contour2);
            double color3Area = contour3 == null ? 0 : Imgproc.contourArea(contour3);

            int maxN = 0;
            MatOfPoint maxContour = null;
            if (color1Area == 0 && color2Area == 0 && color3Area == 0) {
                maxN = 0;
            } else {
                if (color1Area > 200 || color2Area > 200 || color3Area > 200) {
                    {
                        MatOfPoint a = color1Area > color2Area ? contour1 : contour2;
                        maxN = color1Area > color2Area ? 1 : 2;

                        double aarea = Imgproc.contourArea(a);
                        maxContour = aarea > color3Area ? a : contour3;
                        maxN = aarea > color3Area ? maxN : 3;
                    }

                    if (maxN == 1) this.parkingLocation = ParkingLocation.One;
                    else if (maxN == 2) this.parkingLocation = ParkingLocation.Two;
                    else if (maxN == 3) this.parkingLocation = ParkingLocation.Three;
                } else {
                    // either set parkingLocation to null or leave it as is
                    this.parkingLocation = ParkingLocation.None;
                    maxN = 0;
                }
            }

            Rect middleRect = new Rect(middleX, middleY, middleW, middleH);
            Scalar avg = Core.mean(hsv.submat(middleRect));
            middleRect.x += startCol;
            middleRect.y += startRow;
            Imgproc.rectangle(frame, middleRect, new Scalar(255, 0, 0), 1);
            Imgproc.putText(frame, String.format("HSV: %3.2f, %3.2f, %3.2f", avg.val[0], avg.val[1], avg.val[2]), new Point(10, 30), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 0, 0), 2);

            Rect rect = new Rect(new Point(startCol, startRow), new Point(endCol, endRow));
            Imgproc.rectangle(frame, rect, new Scalar(0, 0, 255), 2);
            Imgproc.putText(frame, String.format("Detected: %d", maxN), new Point(10, 60), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 2);

            if (maxContour != null) {
                Rect r = Imgproc.boundingRect(maxContour);
                r.x += startCol;
                r.y += startRow;
                Imgproc.rectangle(frame, r, new Scalar(255, 255, 0), 2);
            }

            return frame;

        } catch (Exception e) {
            Log.e("Sci", "Error occurred when processing frame in SleevePipeline, " + e);
            return frame;
        }
    }
}
