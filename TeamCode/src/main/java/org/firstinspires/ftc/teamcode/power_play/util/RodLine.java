package org.firstinspires.ftc.teamcode.power_play.util;

import android.util.Log;

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

public class RodLine extends OpenCvPipeline {
    private boolean usingYellow = false;

    public RodLine useYellow() {
        this.usingYellow = true;
        return this;
    }

    private int frameWidth;

    private Mat hsb;

    private Mat yellowLayer;
    private Mat blueLayer;
    private Mat redLayer;
    private Mat bluePlusRedLayer;

    private Mat rod;

    private volatile boolean isYellow = false;
    private volatile boolean rodDetected = false;
    private volatile Rect rodRect = new Rect();
    private volatile double distFromRod = 0;

    public boolean isYellow() {
        return this.isYellow;
    }

    public boolean isRodDetected() {
        return this.rodDetected;
    }

    public Rect getRodRect() {
        return this.rodRect;
    }

    public double getRodWidth() {
        return this.rodRect.width;
    }

    /**
     * -1 <= d <= 1
     * If `d > 0` then the rod is in the right half of the screen
     */
    public double getRodCenterOffset() {
        return this.distFromRod;
    }

    @Override
    public void init(Mat frame) {
        this.frameWidth = frame.width();

        hsb = new Mat(frame.rows(), frame.cols(), CvType.CV_8UC3);

        yellowLayer = new Mat(frame.rows(), frame.cols(), CvType.CV_8UC1);
        blueLayer = new Mat(frame.rows(), frame.cols(), CvType.CV_8UC1);
        redLayer = new Mat(frame.rows(), frame.cols(), CvType.CV_8UC1);
        bluePlusRedLayer = new Mat(frame.rows(), frame.cols(), CvType.CV_8UC1);
        rod = new Mat(frame.rows(), frame.cols(), CvType.CV_8UC1);
    }

    @Override
    public Mat processFrame(Mat frame) {
        Imgproc.cvtColor(frame, hsb, Imgproc.COLOR_RGB2HSV);

        if (usingYellow)
            Core.inRange(hsb, new Scalar(16, 125, 125), new Scalar(35, 255, 255), yellowLayer);
        Core.inRange(hsb, new Scalar(95, 125, 75), new Scalar(130, 255, 255), blueLayer);
        Core.inRange(hsb, new Scalar(165, 125, 75), new Scalar(180, 255, 255), redLayer);

        Core.bitwise_or(blueLayer, redLayer, bluePlusRedLayer);
        if (usingYellow) Core.bitwise_or(bluePlusRedLayer, yellowLayer, rod);

        Imgproc.morphologyEx(rod, rod, Imgproc.MORPH_OPEN, Mat.ones(7, 7, CvType.CV_8UC1));
        Imgproc.dilate(rod, rod, Mat.ones(12, 2, CvType.CV_8UC1));

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(rod, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.size() > 0) {
            int max_i = 0;
            double max_a = 0;

            for (int i = 1; i < contours.size(); i++) {
                double a = Imgproc.contourArea(contours.get(i));
                if (a > max_a) {
                    max_a = a;
                    max_i = i;
                }
            }

            Rect boundingRect = Imgproc.boundingRect(contours.get(max_i));
            double centerRodX = boundingRect.x + (boundingRect.width / 2.0);
            this.distFromRod = (centerRodX - (this.frameWidth / 2.0)) / (this.frameWidth / 2.0);
            this.rodRect = boundingRect;
            this.rodDetected = true;

            if (usingYellow) {
                int yellowCount = Core.countNonZero(yellowLayer.submat(boundingRect));
                int bluePlusRedCount = Core.countNonZero(bluePlusRedLayer.submat(boundingRect));
                this.isYellow = yellowCount > bluePlusRedCount;
            }

            {
                Imgproc.rectangle(frame, boundingRect, new Scalar(0, 255, 0), 2);
                Imgproc.putText(frame, "" + boundingRect.width, new Point(10, 60), Imgproc.FONT_HERSHEY_PLAIN, 1.2, new Scalar(255, 0, 0), 2);
                Imgproc.putText(frame, "" + this.distFromRod, new Point(10, 80), Imgproc.FONT_HERSHEY_PLAIN, 1.2, new Scalar(255, 0, 0), 2);
                if (usingYellow)
                    Imgproc.putText(frame, this.isYellow ? "Rod" : "Cone", new Point(10, 100), Imgproc.FONT_HERSHEY_PLAIN, 1.2, new Scalar(255, 0, 0), 2);
            }
        } else {
            this.rodDetected = false;
            Log.e("Sci", "no");
        }

        return frame;
    }
}
