//package org.firstinspires.ftc.teamcode.ultimate_goal;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//import org.firstinspires.ftc.teamcode.ultimate_goal.study.VuforiaTargeting;
//import org.firstinspires.ftc.teamcode.ultimate_goal.util.BananaPipeline;
//import org.firstinspires.ftc.teamcode.ultimate_goal.util.Toggle;
//import org.opencv.core.Point;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//
//@TeleOp(group = "Test")
//@Disabled
//public class Vuforia_Scorpion extends LinearOpMode {
//    BananaPipeline pipeline;
//    OpenCvInternalCamera phoneCam;
//
//    VuforiaTargeting vuforia = new VuforiaTargeting();
//
//    // Declare OpMode members.
//    private final ElapsedTime runtime = new ElapsedTime();
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightBackDrive = null;
//    private DcMotor leftFrontDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private BNO055IMU imu = null;
//    private int leftBackStartPos = 0;
//    private int rightBackStartPos = 0;
//    private int leftFrontStartPos = 0;
//    private int rightFrontStartPos = 0;
//    private final Toggle fieldOriented = new Toggle(false);
//
//    public void initIMU() {
//        // Set up the parameters with which we will use our IMU. Note that integration
//        // algorithm here just reports accelerations to the logcat log; it doesn't actually
//        // provide positional information.
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        // parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = false;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = null; // new JustLogginngAccelerationIntegrator();
//
//        imu.initialize(parameters);
//        // Start the logging of measured acceleration
//        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);
//
//        telemetry.addData("Gyro", "calibrating...");
//        telemetry.update();
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//        while (!imu.isGyroCalibrated() && !isStopRequested() && timer.seconds() < 12) {
//            sleep(1100);
//        }
//        if (imu.isGyroCalibrated())
//            telemetry.addData("Gyro", "IMU Ready");
//        else
//            telemetry.addData("Gyro", "Gyro IMU Calibration FAILED !!!!!!!!!!!!!!");
//
//        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);
//
//        telemetry.update();
//
//        RobotLog.d("IMU status: %s", imu.getSystemStatus().toShortString());
//        RobotLog.d("IMU calib: %s", imu.getCalibrationStatus().toString());
//        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);
//
//    }
//
//    private void initCvCamera() {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        pipeline = new BananaPipeline();
//        phoneCam.setPipeline(pipeline);
//
//        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//
//        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                requestOpModeStop();
//            }
//        });
//    }
//
//    public void stopPower() {
//        setPower(0, 0, 0);
//    }
//
//
////    public void drive(double meter) {
////        ElapsedTime timer = new ElapsedTime();
////        double k = 2500; // [meter pre sec]
////        int timetometer = (int) (meter * k);
////        while (timer.milliseconds() < Math.abs(timetometer)) {
////            double power;
////            if (meter > 0) {
////                power = 0.5;
////            } else {
////                power = -0.5;
////            }
////            leftBackDrive.setPower(power);
////            rightBackDrive.setPower(power);
////            leftFrontDrive.setPower(power);
////            rightFrontDrive.setPower(power);
////
////        }
////        stopPower();
////    }
//
//
//    public void resetPosition() {
//        leftBackStartPos = leftBackDrive.getCurrentPosition();
//        rightBackStartPos = rightBackDrive.getCurrentPosition();
//        leftFrontStartPos = leftFrontDrive.getCurrentPosition();
//        rightFrontStartPos = rightFrontDrive.getCurrentPosition();
//
//    }
//
//    public void strafe(double meter, double power) {
//        double targetAngle = getHeading(); // zeroAngle
//        strafe(meter, power, targetAngle);
//    }
//
//
//    public void strafe(double meter, double power, double targetAngle) {
//        double s = (meter < 0) ? -1 : 1;
//
//        resetPosition();
//
//        while ((getStrafeDistance() * s < meter * s) && opModeIsActive()) {
//            double currentAngle = getHeading();
//            double err = getDeltaHeading(targetAngle);
//            double gain = 0.035;
//            double correction = gain * err;
//
//            setPower(0, correction, power * s);
//
//            telemetry.addData("distance", getStrafeDistance());
//            telemetry.addData("target", targetAngle);
//            telemetry.addData("current", currentAngle);
//            telemetry.addData("Error", err);
//            telemetry.update();
//        }
//        stopPower();
//    }
//
//    public double getForwardDistance() {
//        double polsMeter = 1150;
//        int leftBack_tick = leftBackDrive.getCurrentPosition() - leftBackStartPos;
//        int rightBack_tick = rightBackDrive.getCurrentPosition() - rightBackStartPos;
//        int leftFront_tick = leftFrontDrive.getCurrentPosition() - leftFrontStartPos;
//        int rightFront_tick = rightFrontDrive.getCurrentPosition() - rightFrontStartPos;
//        double leftFrontDist = leftFront_tick / polsMeter;
//        double rightFrontDist = rightFront_tick / polsMeter;
//        double leftBackDist = leftBack_tick / polsMeter;
//        double rightBackDist = rightBack_tick / polsMeter;
//        double evgPols = (leftBackDist + rightBackDist + rightFrontDist + leftFrontDist) / 4;
//        return evgPols;
//    }
//
//    public double getStrafeDistance() {
//        double polsMeter = 1000;
//        int leftBack_tick = leftBackDrive.getCurrentPosition() - leftBackStartPos;
//        int rightBack_tick = rightBackDrive.getCurrentPosition() - rightBackStartPos;
//        int leftFront_tick = leftFrontDrive.getCurrentPosition() - leftFrontStartPos;
//        int rightFront_tick = rightFrontDrive.getCurrentPosition() - rightFrontStartPos;
//        double leftFrontDist = leftFront_tick / polsMeter;
//        double rightFrontDist = rightFront_tick / polsMeter;
//        double leftBackDist = leftBack_tick / polsMeter;
//        double rightBackDist = rightBack_tick / polsMeter;
//        // double forwardDistance = (leftBackDist + rightBackDist + rightFrontDist + leftFrontDist) / 4;
//        double strafeDistance = (leftBackDist + rightBackDist - rightFrontDist + leftFrontDist) / 4;
//        return strafeDistance;
//    }
//
//    public void setPower(double forward, double turn, double strafe) {
//        leftFrontDrive.setPower(forward + turn + strafe);
//        rightFrontDrive.setPower(forward - turn - strafe);
//
//        leftBackDrive.setPower(forward + turn - strafe);
//        rightBackDrive.setPower(forward - turn + strafe);
//    }
//
//
//    public double getImuDistance(Position target) {
//        Position current = imu.getPosition();
//        double dx = current.x - target.x;
//        double dy = current.y - target.y;
//        double sqrt = Math.pow(dy, 2) + Math.pow(dx, 2);
//        double distance = Math.sqrt(sqrt);
//        return distance;
//    }
//
//
//    public double getDeltaHeading(double target) {
//        double currentAngle = getHeading();
//        double delta = target - currentAngle;
//
//        if (delta < 180) {
//            delta = delta + 360;
//        }
//        if (delta > 180) {
//            delta = delta - 360;
//        }
//
//        return delta;
//    }
//
//
//    double getHeading() {
//        Orientation orie = imu.getAngularOrientation();
//        double angle = -orie.firstAngle;
//        return angle;
//    }
//
//
//    public void driveForward(double meter, double power) {
//        double targetAngle = getHeading(); // zeroAngle
//        driveForward(meter, power, targetAngle);
//    }
//
//
//    public void driveForward(double meter, double targetPower, double targetAngle) {
//        double s = (meter < 0) ? -1 : 1;
//        resetPosition();
//
//        while ((getForwardDistance() * s < meter * s) && opModeIsActive()) {
//            double power = targetPower;
//            double acclGain = 2;
//            double acclPower = Math.abs(getForwardDistance()) * acclGain + 0.2;
//            if (acclPower < power)
//                power = acclPower;
//
//            double breakgain = 1;
//            double deltaForward = Math.abs(meter - getForwardDistance());
//            double breakpower = deltaForward * breakgain;
//            if (breakpower < power)
//                power = breakpower;
//
//            double currentAngle = getHeading();
//            double err = getDeltaHeading(targetAngle);
//            double gain = 0.040;
//            double correction = gain * err;
//
//            setPower(power * s, correction, 0);
//
//            telemetry.addData("deltaForward", deltaForward);
//            telemetry.addData("acclPower", acclPower);
//            telemetry.addData("breakPower", breakpower);
//            telemetry.addData("power", power);
//
//            telemetry.addData("target", targetAngle);
//            telemetry.addData("current", currentAngle);
//            telemetry.addData("Error", err);
//            telemetry.update();
//        }
//        stopPower();
//    }
//
//    public double turnToTargetCV() {
//
//        Point target = pipeline.getTargetPos();
//        if (target != null) {
//            int center = 240 / 2;
//            double deltaFromCenter = target.x - center;
//            double gain = 0.4;
//            return (deltaFromCenter / center) * gain;
//        }
//
//        else {return 0;}
//    }
//
//
//    public double turnToTargetVuforia() {
//        float deg = vuforia.getTargetHeading();
//        double gain = -0.01;
//        return deg * gain;
//    }
//
//
//    public double turnToTargetImu(double targetAngle, double targetPower) {
//        double delta = getDeltaHeading(targetAngle);
//
//        if (Math.abs(delta) < 1)
//            return 0;
//
//        double gain = 0.01;
//        double power = gain * delta * targetPower;
////        if (Math.abs(power) < 0.1)
////            power = 0.1 * Math.signum(power);
//        return power;
//    }
//
//
//    public void turn(double deg, double power) {
//        double targetAngle = getHeading() + deg; // zeroAngle
//        turnTo(targetAngle, power);
//    }
//
//
//    public void turnTo(double targetAngle, double targetPower) {
//        double delta = getDeltaHeading(targetAngle);
//        double s = (delta < 0) ? -1 : 1;
//        while ((delta * s > 5 * s) && opModeIsActive()) {
//
//            delta = getDeltaHeading(targetAngle);
//            double gain = 0.04;
//            double power = gain * delta * targetPower;
//            if (Math.abs(power) < 0.1)
//                power = 0.1 * Math.signum(power);
//
//            setPower(0, power, 0);
//
//            telemetry.addData("target", targetAngle);
//            telemetry.addData("current", getHeading());
//            telemetry.addData("delta", delta);
//            telemetry.addData("power", power);
//            telemetry.update();
//
//        }
//        stopPower();
//    }
//
//    public void square() {
//        double heading = getHeading();
//        for (int i = 0; i < 4; i++) {
//            driveForward(1, heading);
//            heading += 90;
//            turnTo(heading, 0.5);
//        }
//    }
//
//
//    public void triangle() {
//        double heading = getHeading();
//        driveForward(1, heading);
//        heading += 90;
//        turnTo(heading, 0.5);
//        driveForward(1, heading);
//        heading += 135;
//        turnTo(heading, 0.5);
//        driveForward(1.35, heading);
//    }
//
//    public void strafeSquare() {
//        double heading = getHeading();
//        driveForward(2, 0.5, heading);
//        strafe(2, 0.5, heading);
//        driveForward(-2, 0.5, heading);
//        strafe(-2, 0.5, heading);
//    }
//
//
//    @Override
//    public void runOpMode() {
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must correspond to the names assigned during the robot configuration
//        // step (using the FTC Robot Controller app on the phone).
//        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//        // Most robots need the motor on one side to be reversed to drive forward
//        // Reverse the motor that runs backwards when connected directly to the battery
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        initIMU();
////        initCvCamera();
//
//        vuforia.initVuforia(hardwareMap);
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//        runtime.reset();
//
//        // run until the end of the match (driver presses STOP)
//        // START
//
//        while (opModeIsActive()) {
//            vuforia.update(telemetry);
//            // Setup a variable for each drive wheel to save power level for telemetry
//            double leftPower;
//            double rightPower;
//
//            // Choose to drive using either Tank Mode, or POV Mode
//            // Comment out the method that's not used.  The default below is POV.
//
//            // POV Mode uses left stick to go forward, and right stick to turn.
//            // - This uses basic math to combine motions and is easier to drive straight.\
//
//            double boost = gamepad1.left_trigger * 0.5 + 0.5;
//            double drive = -gamepad1.left_stick_y * boost;
//            double turn = gamepad1.right_stick_x * boost;
//            double strafe = gamepad1.left_stick_x * boost;
//
//            if (gamepad1.y) {
//                turn = turnToTargetCV();
//            }
//
//            if (gamepad1.a) {
//                strafe = turnToTargetVuforia();
//                turn = turnToTargetImu(0, 1);
//            }
//
//            double alpha = getHeading();
//            double forward = Math.cos(alpha) * drive;
//            double side = Math.sin(alpha) * strafe;
//
//
//            if (fieldOriented.getState() == true)
//                setPower(forward, turn, side);
//            else
//                setPower(drive, turn, strafe);
//
//            // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("StrafePos", getStrafeDistance());
//            telemetry.addData("Position", getForwardDistance());
//            telemetry.addData("Acceleration", imu.getAcceleration());
//            telemetry.addData("Heading", getHeading());
//            telemetry.update();
//
//
//        }
//        vuforia.end();
//    }
//}