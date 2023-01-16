package org.firstinspires.ftc.teamcode.power_play.util;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
public class SleevePipeline extends OpenCvPipeline {
	//    public static volatile int minh1, maxh1, minh2, maxh2, minh3, maxh3;
	public static volatile int minp = 145, maxp = 175, ming = 40, maxg = 75;

	// Orange
	public static volatile Scalar min_color1 = new Scalar(10, 125, 125);
	public static volatile Scalar max_color1 = new Scalar(30, 255, 255);

	// Purple / Pink
//    public static volatile Scalar min_color2 = new Scalar(145, 125, 125);
//    public static volatile Scalar max_color2 = new Scalar(175, 255, 255);

	// Green
//    public static volatile Scalar min_color3 = new Scalar(40, 125, 125);
//    public static volatile Scalar max_color3 = new Scalar(75, 255, 255);

	public enum ParkingLocation {
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

		hsv = new Mat(firstFrame.rows(), firstFrame.cols(), CvType.CV_8UC3);
		thresholded = new Mat(firstFrame.rows(), firstFrame.cols(), CvType.CV_8UC1);

		// for testing
//        if (true) {
//            min_color1.val[0] -= 5;
//            max_color1.val[0] += 5;
//            min_color1.val[1] -= 25;
//            min_color1.val[2] -= 25;
//
//            min_color2.val[0] -= 5;
//            max_color2.val[0] += 5;
//            min_color2.val[1] -= 25;
//            min_color2.val[2] -= 25;
//
//            min_color3.val[0] -= 5;
//            max_color3.val[0] += 5;
//            min_color3.val[1] -= 25;
//            min_color3.val[2] -= 25;
//        }

//        width = firstFrame.width();
//        height = firstFrame.height();
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
			Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

			MatOfPoint contour1 = processColor(min_color1, max_color1);
			//        MatOfPoint contour2 = processColor(min_color2, max_color2);
			//        MatOfPoint contour3 = processColor(min_color3, max_color3);

			MatOfPoint contour2 = processColor(new Scalar(minp, 125, 125), new Scalar(maxp, 255, 255));
			MatOfPoint contour3 = processColor(new Scalar(ming, 100, 100), new Scalar(maxg, 255, 255));

			double color1Area = contour1 == null ? 0 : Imgproc.contourArea(contour1);
			double color2Area = contour2 == null ? 0 : Imgproc.contourArea(contour2);
			double color3Area = contour3 == null ? 0 : Imgproc.contourArea(contour3);

			if (color1Area == 0 && color2Area == 0 && color3Area == 0) return frame;

			if (color1Area > 200 || color2Area > 200 || color3Area > 200) {
				MatOfPoint maxContour;
				int maxN;
				{
					MatOfPoint a = color1Area > color2Area ? contour1 : contour2;
					maxN = color1Area > color2Area ? 1 : 2;

					double aarea = Imgproc.contourArea(a);
					maxContour = aarea > color3Area ? a : contour3;
					maxN = aarea > color3Area ? maxN : 3;
				}

				{
					Rect r = Imgproc.boundingRect(maxContour);
					Imgproc.rectangle(frame, r, new Scalar(255, 255, 0), 2);
				}

				if (maxN == 1) this.parkingLocation = ParkingLocation.One;
				else if (maxN == 2) this.parkingLocation = ParkingLocation.Two;
				else if (maxN == 3) this.parkingLocation = ParkingLocation.Three;
			} else {
				// either set parkingLocation to null or leave it as is
			}

			Log.d("Sci", "pipeline result: " + this.parkingLocation);

			//        Runtime.getRuntime().gc();
			return frame;
		} catch (Exception e) {
			Log.e("Sci", "Error occurred when processing frame, " + e);
			return frame;
		}
	}
}
