package org.firstinspires.ftc.teamcode.centerstage.util;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.List;

public class CameraPipeline {
    private String cameraName;
    private Size viewSize;
    private Telemetry telemetry;
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;

    public CameraPipeline(String cameraName, Size viewSize, HardwareMap hardwareMap, Telemetry telemetry) {
        this.cameraName = cameraName;
        this.viewSize = viewSize;
        this.telemetry = telemetry;
        initialize(hardwareMap);
    }

    void initialize(HardwareMap hardwareMap) {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(viewSize)
                .build();
    }

    public List<AprilTagDetection> getDetections() {
        return tagProcessor.getDetections();
    }

    /**
     * Prints the telemetry data of a specific tag (Doesn't update the telemetry)
     *
     * @param tag Tag detection reference
     */
    public void printTelemetryData(AprilTagDetection tag) {
        double tagSize = calculateTagSize(tag.corners); // in pixels

        telemetry.addData("Tag ID", tag.id);
        telemetry.addData("Tag Size", tagSize);
        telemetry.addData("Pos", "\n\tx: " + tag.ftcPose.x
                + ", \n\ty: " + tag.ftcPose.y +
                ", \n\tz: " + tag.ftcPose.z);
        telemetry.addData("Rotation", "\n\tyaw: " + tag.ftcPose.yaw +
                "\n\tpitch: " + tag.ftcPose.pitch +
                "\n\troll: " + tag.ftcPose.roll);
    }

    public AprilTagDetection getSpecificTag(int id) {
        for (AprilTagDetection tag :
                getDetections()) {
            if (tag.id == id) {
                return tag;
            }
        }
        return null;
    }

    /**
     * Calculate the size of the tag based on its corner points
     *
     * @param corners
     * @return Tag size (On the camera) in pixels
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
}
