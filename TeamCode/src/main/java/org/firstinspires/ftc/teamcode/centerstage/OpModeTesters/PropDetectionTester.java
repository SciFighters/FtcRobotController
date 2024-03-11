//package org.firstinspires.ftc.teamcode.centerstage.OpModeTesters;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.DuckLine;
//import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;
//import org.opencv.core.Scalar;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//@TeleOp
//@Config
//public class PropDetectionTester extends Robot {
//    public static Scalar min_red = new Scalar(0, 100, 100);
//    public static Scalar max_red = new Scalar(10, 255, 255);
//    public static Scalar min_blue = new Scalar(90, 100, 100);
//    public static Scalar max_blue = new Scalar(130, 255, 255);
//    final int screenWidth = 640;
//    final int screenHeight = 360;
//    DuckLine duckLine;
//    MultipleTelemetry multipleTelemetry;
//    OpenCvWebcam webcam;
//
//    @Override
//    public void initRobot() {
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = dashboard.getTelemetry();
//        multipleTelemetry = new MultipleTelemetry(dashboardTelemetry, telemetry);
//        duckLine = new DuckLine(alliance, multipleTelemetry);
//        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "cam");
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewID);
//
//        this.duckLine = new DuckLine(this.alliance, dashboardTelemetry);
//        webcam.setPipeline(this.duckLine);
//    }
//
//    @Override
//    public void startRobot() {
//        webcam.openCameraDevice();
//        webcam.startStreaming(screenWidth, screenHeight, OpenCvCameraRotation.UPRIGHT);
//    }
//}
