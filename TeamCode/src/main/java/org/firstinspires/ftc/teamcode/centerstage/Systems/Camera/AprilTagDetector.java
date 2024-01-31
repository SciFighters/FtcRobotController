package org.firstinspires.ftc.teamcode.centerstage.Systems.Camera;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;
import org.firstinspires.ftc.teamcode.centerstage.util.Location;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.opencv.core.CvType;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;

import java.util.List;

/**
 * This class represents a camera pipeline for processing AprilTag detections with image stabilization.
 */
public class AprilTagDetector {

    private final String cameraName;
    private final Size viewSize;
    private final Telemetry telemetry;
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private Mat previousFrame;

    /**
     * Initializes the CameraPipeline with the specified parameters.
     *
     * @param cameraName    The name of the camera.
     * @param viewSize      The size of the camera view.
     * @param hardwareMap   The HardwareMap for accessing the camera.
     * @param telemetry     The telemetry object for displaying information.
     * @param configuration The portal configuration.
     */
    public AprilTagDetector(String cameraName, Size viewSize, HardwareMap hardwareMap, Telemetry telemetry, PortalConfiguration configuration) {
        this.cameraName = cameraName;
        this.viewSize = viewSize;
        this.telemetry = telemetry;
        initialize(hardwareMap, configuration);
    }

    private void initialize(HardwareMap hardwareMap, PortalConfiguration configuration) {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(configuration.DRAW_AXES)
                .setDrawCubeProjection(configuration.DRAW_CUBE_PROJECTION)
                .setDrawTagID(configuration.DRAW_TAG_ID)
                .setDrawTagOutline(configuration.DRAW_TAG_OUTLINE)
                .setNumThreads(configuration.NUM_THREADS)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(viewSize)
                .build();
    }

    /**
     * Apply a simple custom image stabilization algorithm to consecutive frames.
     *
     * @param currentFrame The current camera frame.
     * @return The stabilized frame.
     */
    private Mat stabilizeFrame(Mat currentFrame) {
        if (previousFrame == null) {
            previousFrame = currentFrame.clone();
            return currentFrame;
        }

        // Convert frames to grayscale for feature tracking
        Mat grayPrevious = new Mat();
        Mat grayCurrent = new Mat();
        Imgproc.cvtColor(previousFrame, grayPrevious, Imgproc.COLOR_RGBA2GRAY);
        Imgproc.cvtColor(currentFrame, grayCurrent, Imgproc.COLOR_RGBA2GRAY);

        // Detect features (corners) in the previous frame
        MatOfPoint prevFeatures = new MatOfPoint();
        Imgproc.goodFeaturesToTrack(grayPrevious, prevFeatures, 500, 0.01, 10);

        // Calculate optical flow using Lucas-Kanade method
        MatOfPoint2f prevFeatures2f = new MatOfPoint2f(prevFeatures.toArray());
        MatOfPoint2f nextFeatures = new MatOfPoint2f();
        MatOfByte status = new MatOfByte();
        MatOfFloat err = new MatOfFloat();
        Video.calcOpticalFlowPyrLK(grayPrevious, grayCurrent, prevFeatures2f, nextFeatures, status, err);

        // Calculate the transformation matrix
        Mat transformMatrix = Imgproc.getPerspectiveTransform(prevFeatures2f, nextFeatures);

        // Apply the transformation to the current frame
        Mat stabilizedFrame = new Mat(currentFrame.size(), CvType.CV_32F);
        Imgproc.warpPerspective(currentFrame, stabilizedFrame, transformMatrix, stabilizedFrame.size(), Imgproc.INTER_LINEAR, Core.BORDER_CONSTANT, Scalar.all(0));

        // Release resources
        grayPrevious.release();
        grayCurrent.release();
        prevFeatures.release();
        prevFeatures2f.release();
        nextFeatures.release();
        status.release();
        err.release();
        transformMatrix.release();

        // Update the previous frame
        previousFrame.release();
        previousFrame = currentFrame.clone();

        return stabilizedFrame;
    }

    /**
     * Get a list of AprilTag detections.
     *
     * @return RIGHT list of AprilTagDetection objects.
     */
    public List<AprilTagDetection> getDetections() {
        return tagProcessor.getDetections();
    }

    /**
     * Prints the telemetry data of a specific tag (Doesn't update the telemetry).
     *
     * @param tag The AprilTagDetection object to display telemetry data for.
     */
    public void printTelemetryData(AprilTagDetection tag) {
        double tagSize = calculateTagSize(tag.corners); // in pixels

        telemetry.addData("Tag ID", tag.id)
                .addData("Tag Size", tagSize)
                .addData("Pos", String.format("\n\tx: %s, \n\ty: %s, \n\tz: %s", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z))
                .addData("Rotation", String.format("\n\tyaw: %s \n\tpitch: %s \n\troll: %s", tag.ftcPose.yaw, tag.ftcPose.pitch, tag.ftcPose.roll));
    }

    /**
     * Get a specific AprilTag by its ID.
     *
     * @param id The ID of the AprilTag to retrieve.
     * @return The AprilTagDetection object with the specified ID, or null if not found.
     */
    public AprilTagDetection getSpecificTag(int id) {
        for (AprilTagDetection tag : getDetections()) {
            if (tag.id == id) {
                return tag;
            }
        }
        return null;
    }

    /**
     * Calculate the size of the tag based on its corner points.
     *
     * @param corners The corner points of the tag.
     * @return Tag size (On the camera) in pixels.
     */
    private double calculateTagSize(Point[] corners) {
        if (corners.length != 4) {
            return 0.0;
        }

        // Calculate the distance between two diagonal corners (assuming a rectangle)
        double dx = corners[0].x - corners[2].x;
        double dy = corners[0].y - corners[2].y;

        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Process the camera frame, including image stabilization.
     *
     * @param frame The current camera frame.
     */
    public void processFrame(Mat frame) {
        // Stabilize the frame
        Mat stabilizedFrame = stabilizeFrame(frame);
        // IDK what now

        // Release the stabilized frame
        stabilizedFrame.release();
    }

    public void lockOnTag(int tagID, double power, DriveClass drive) {
        try {
            double initialX = drive.getPosX(), initialY = drive.getPosY();
            double initialHeading = drive.getHeading();

            AprilTagDetection tag = this.getSpecificTag(tagID);

            if (tag != null && tag.rawPose != null) {
                double tagX = tag.rawPose.x;
                double tagZ = tag.rawPose.z;
                double deltaHeading = Math.toDegrees(Math.atan2(tagX, tagZ));
                double targetHeading = initialHeading + deltaHeading;

                telemetry.addData("GOTO", String.format("X: %s\nY: %s\nRIGHT: %s",
                        tagX, initialY, targetHeading));
                drive.goToLocation(new Location(initialX - 0.08, initialY), power, -90, 0.05, 0);
//                drive.turnTo(targetHeading, 0.4);
            }
        } catch (Exception e) {
            telemetry.addData("THE FUCK WENT WRONG", e.toString());
            telemetry.update();
        }
    }


    public void lockOnTag(AprilTagDetector.AprilTags tag, double power, DriveClass drive) {
        lockOnTag(tag.ID, power, drive);
    }

    /**
     * Enumeration of AprilTags.
     */
    public enum AprilTags {
        BlueLeft(1), BlueCenter(2), BlueRight(3), RedLeft(4), RedCenter(5), RedRight(6);
        public final int ID;

        AprilTags(int ID) {
            this.ID = ID;
        }
    }

    /**
     * Configuration for the Vision Portal.
     */
    public static class PortalConfiguration {
        public final boolean DRAW_AXES = true;
        public final boolean DRAW_CUBE_PROJECTION = false;
        public final boolean DRAW_TAG_ID = true;
        public final boolean DRAW_TAG_OUTLINE = true;
        public final int NUM_THREADS = 1;

        public PortalConfiguration() {
        }

        public final static PortalConfiguration DEFAULT = new PortalConfiguration();
    }
}
