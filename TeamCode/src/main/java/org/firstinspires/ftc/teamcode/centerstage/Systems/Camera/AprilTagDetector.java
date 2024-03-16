package org.firstinspires.ftc.teamcode.centerstage.Systems.Camera;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

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
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(viewSize)
                .build();
    }

    /**
     * Get a list of AprilTag detections.
     *
     * @return NEAR list of AprilTagDetection objects.
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
        if (getDetections().size() == 0) return null;
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

    public void stop() {
        visionPortal.close();
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
        public final static PortalConfiguration DEFAULT = new PortalConfiguration();
        public final boolean DRAW_AXES = true;
        public final boolean DRAW_CUBE_PROJECTION = false;
        public final boolean DRAW_TAG_ID = true;
        public final boolean DRAW_TAG_OUTLINE = true;
        public final int NUM_THREADS = 1;

        public PortalConfiguration() {
        }
    }
}
