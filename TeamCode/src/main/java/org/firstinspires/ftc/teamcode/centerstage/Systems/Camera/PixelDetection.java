package org.firstinspires.ftc.teamcode.centerstage.Systems.Camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import java.util.ArrayList;
import java.util.List;

public class PixelDetection extends LinearOpMode {
    private VideoCapture webcam;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            Mat frame = new Mat();
            if (webcam.read(frame)) {
                processFrame(frame);
                // Display the processed frame
//                HighGui.imshow("Processed Frame", frame);
//                HighGui.waitKey(1);
            }
        }
    }

    private void initialize() {
        String webcamName = "cam";
        HardwareMap hardwareMap = this.hardwareMap;

        // Initialize the webcam
        webcam = new VideoCapture();
        webcam.open(0);

        // Wait for the camera to be opened
        while (!webcam.isOpened()) {
            telemetry.addData("Status", "Camera initializing...");
            telemetry.update();
            sleep(100);
        }

        telemetry.addData("Status", "Camera initialized");
        telemetry.update();

    }

    private void processFrame(Mat frame) {
        // Convert the frame to grayscale
        Mat gray = new Mat();
        Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGB2GRAY);

        // Apply a Gaussian blur to reduce noise
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(gray, blurred, new Size(5, 5), 0);

        // Threshold the image to create a binary image
        Mat thresholded = new Mat();
        Imgproc.threshold(blurred, thresholded, 150, 255, Imgproc.THRESH_BINARY);

        // Find contours in the binary image
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresholded, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Loop through the detected contours
        for (MatOfPoint contour : contours) {
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            double arcLength = Imgproc.arcLength(contour2f, true);
            Imgproc.approxPolyDP(contour2f, approxCurve, 0.04 * arcLength, true);

            // Check if the contour is a hexagon
            if (approxCurve.total() == 6) {
                // Check for a small hexagon in the middle
                double area = Imgproc.contourArea(contour);
                if (area < 500) {
                    // You have detected a hexagon with a hole in the middle.
                    // Draw a bounding box around it
                    Rect boundingBox = Imgproc.boundingRect(contour);
                    Imgproc.rectangle(frame, boundingBox.tl(), boundingBox.br(), new Scalar(0, 255, 0), 2);

                    // Display information about the detected hexagon
                    telemetry.addData("Detected Hexagon", "Area: " + area);
                    telemetry.addData("Bounding Box", "X: " + boundingBox.x + " Y: " + boundingBox.y +
                            " Width: " + boundingBox.width + " Height: " + boundingBox.height);
                    telemetry.update();
                }
            }
        }
    }
}
