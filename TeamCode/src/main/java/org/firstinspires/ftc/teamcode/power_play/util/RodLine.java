package org.firstinspires.ftc.teamcode.power_play.util;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class RodLine extends OpenCvPipeline {
	private Mat hsb;

	private Mat yellowLayer;
	private Mat blueLayer;
	private Mat redLayer;

	private Mat rod;

	@Override
	public void init(Mat frame) {
		hsb = new Mat(frame.rows(), frame.cols(), CvType.CV_8UC3);

		yellowLayer = new Mat(frame.rows(), frame.cols(), CvType.CV_8UC1);
		blueLayer = new Mat(frame.rows(), frame.cols(), CvType.CV_8UC1);
		redLayer = new Mat(frame.rows(), frame.cols(), CvType.CV_8UC1);
		rod = new Mat(frame.rows(), frame.cols(), CvType.CV_8UC1);
	}

	@Override
	public Mat processFrame(Mat frame) {
		Imgproc.cvtColor(frame, hsb, Imgproc.COLOR_RGB2HSV);

		Core.inRange(hsb, new Scalar(16, 125, 125), new Scalar(35, 255, 255), yellowLayer);
		Core.inRange(hsb, new Scalar(95, 125, 125), new Scalar(130, 255, 255), blueLayer);
		Core.inRange(hsb, new Scalar(165, 125, 125), new Scalar(180, 255, 255), redLayer);

		Core.bitwise_or(yellowLayer, blueLayer, rod);
		Core.bitwise_or(rod, redLayer, rod);

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

			Imgproc.rectangle(frame, Imgproc.boundingRect(contours.get(max_i)), new Scalar(0, 255, 0), 2);
		}

		return frame;
	}
}
