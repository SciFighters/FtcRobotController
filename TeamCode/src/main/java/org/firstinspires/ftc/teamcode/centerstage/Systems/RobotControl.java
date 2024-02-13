package org.firstinspires.ftc.teamcode.centerstage.Systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Component;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.RobotTelemetry;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.ThreadedComponent;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.Input;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.Toggle;
import org.firstinspires.ftc.teamcode.centerstage.util.Location;

@ThreadedComponent
public class RobotControl extends Component {
    Gamepad gamepad1, gamepad2;
    // Robot components
    DriveClass drive;
    Arm arm;
    IntakeSystem intakeSystem;
//    AprilTagDetector aprilTagDetector;

    // Telemetry and dashboard
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    @RobotTelemetry
    MultipleTelemetry multipleTelemetry;

    // Control variables
    boolean fieldOriented = true;
    boolean allowMovement = true;
    double angle = 0;
    double targetHeading;
    int turningCount = 0;
    Gamepad.RumbleEffect rumbleEffect;

    Toggle turningToggle = new Toggle();
    Toggle robotOrientedToggle = new Toggle();
    double armBoost;
    DroneLauncher droneLauncher;
    double allianceDefaultHeading = 90;

    @Override
    public void init() {
        gamepad1 = robot.gamepad1;
        gamepad2 = robot.gamepad2;
// Initialize robot components
        initSystems();
        rumbleEffect = new Gamepad.RumbleEffect.Builder().addStep(1.0, 1.0, 300).build();
    }

    @Override
    public void start() {
        // Initialize input and start subsystems
        Input.init(robot, robot.gamepad1, robot.gamepad2);
        targetHeading = drive.getHeading();
        drive.resetOrientation(allianceDefaultHeading);
        intakeSystem.start();
        arm.start();
    }


    public void setAlliance(AutoFlow.Alliance alliance) {
        robot.alliance = alliance;
        if (alliance == AutoFlow.Alliance.BLUE) {
            allianceDefaultHeading = 90;
        } else {
            allianceDefaultHeading = -90;
        }
        drive.resetHeading(allianceDefaultHeading);
    }

    @Override
    public void update() {
        // Handle force quit command
        handleForceQuit();
        // Update Arm and Intake subsystems
        updateArmAndIntake();

        // Update driving controls
        updateDriving();
    }

    // Method to update Arm and Intake subsystems
    private void updateArmAndIntake() {
        if (gamepad2.start) {
            return;
        }
        // Arm control based on gamepad input
        armBoost = (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) ? 0.4 : 1;
        if (Math.abs(gamepad2.left_stick_y) > 0.05) {
            arm.setPower(pow(-gamepad2.left_stick_y) * armBoost);
        } else if (gamepad2.y) {
            armGotoLevel(Arm.Position.Three);
        } else if (gamepad2.b) {
            armGotoLevel(Arm.Position.Two);
        } else if (gamepad2.a) {
            armGotoLevel(Arm.Position.One);
        } else if (gamepad2.x) {
            armGotoLevel(Arm.Position.Home);
        } else {
            arm.setMotorsPower(0);
        }
        // Claw control based on gamepad input
        if (gamepad2.right_bumper) {
            arm.closeClaw(true); // close claw
//            gamepad2.runRumbleEffect(rumbleEffect);
        } else if (gamepad2.left_bumper) {
            arm.closeClaw(false); // open claw
        } else if (gamepad2.dpad_up) {
            intakeSystem.stopIntake(); // stop intake
            arm.closeClaw(true);
        } else if (gamepad2.dpad_down) {
            intakeSystem.collect(); // start intake
            arm.closeClaw(false);
        } else if (gamepad2.right_stick_button) {
            intakeSystem.spit(); // start spit
        } else if (gamepad2.dpad_left) {
            arm.dropBottomPixel();
        }
    }

    // Method to update driving controls
    private void updateDriving() {
        robotOrientedToggle.update(gamepad1.ps);
        // Check for reorientation command
        if (gamepad1.start && gamepad1.x) {
            drive.resetHeading(allianceDefaultHeading);
//            gamepad1.runRumbleEffect(rumbleEffect);
            targetHeading = drive.getHeading();
            return;
        }
        // Boost factor for driving speed
        double boost = (gamepad1.left_trigger > 0.05 || gamepad1.right_trigger > 0.05) ? 1.5 : 0.6;

        // Drive control based on gamepad input
        double y = pow(-gamepad1.left_stick_y) * boost;
        double x = pow(gamepad1.left_stick_x) * boost;
        double turn = pow(gamepad1.right_stick_x) * boost;

        // Update turning toggle state
        turningToggle.update(Math.abs(turn) > 0.02 || gamepad1.a);

//         Reset turning count if toggle is released
        if (turningToggle.isReleased()) {
            turningCount = 8;
        }

        // Decrease turning count if toggle is not pressed
        if (!turningToggle.isPressed()) {
            turningCount--;
        }

//         Set target heading if turning count is zero
        if (turningCount == 0) {
            targetHeading = drive.getHeading();
        }
        // Apply rotation fix if turning count is less than zero
        if (gamepad1.a && !gamepad1.share) {
            double delta = drive.getDeltaHeading(robot.alliance == AutoFlow.Alliance.BLUE ? 180 : -180);
            targetHeading = robot.alliance == AutoFlow.Alliance.BLUE ? 180 : -180;
            double gain = 0.02;
            turn = delta * gain;
        } else if (!turningToggle.isPressed() && turningCount < 0 && !gamepad1.a) {
            double delta = drive.getDeltaHeading(targetHeading);
            double gain = 0.02;
            turn = delta * gain;
        }

        if (gamepad1.touchpad_finger_1) {
            if (gamepad1.b) setAlliance(AutoFlow.Alliance.RED);
            else if (gamepad1.a) setAlliance(AutoFlow.Alliance.BLUE);
            drive.resetHeading(allianceDefaultHeading);
        }
        if (robotOrientedToggle.isClicked()) {
            fieldOriented = !fieldOriented;
        }
        // Perform dpad-specific actions
        if (gamepad1.dpad_left) drive.setPowerOriented(0, -0.3, 0, fieldOriented);
        else if (gamepad1.dpad_right) drive.setPowerOriented(0, 0.3, 0, fieldOriented);
        else if (gamepad1.dpad_up) drive.setPowerOriented(0.3, 0, 0, fieldOriented);
        else if (gamepad1.dpad_down) drive.setPowerOriented(-0.3, 0, 0, fieldOriented);
        else if (!arm.isPlacePixelBusy) {
            drive.setPowerOriented(y, x, turn, fieldOriented);
            allowMovement = true;
            // Update telemetry
            telemetry.addData("Y POWER", y);
            updateTelemetry();
        }

        //region hang and drone
        if (gamepad1.y && !gamepad1.start && !gamepad1.share) {
            droneLauncher.launch();
        } else if (gamepad1.b && !gamepad1.start) {
            arm.hang();
        }
        //endregion
    }

    // Method to initialize robot subsystems
    private void initSystems() {
        telemetry.addData(">", "Init in progress...");
        // Initialize telemetry
        telemetryInit();
        robot.addComponent(Arm.class, new Arm());
        arm = robot.getComponent(Arm.class);
        // Add robot components
        robot.addComponent(IntakeSystem.class, new IntakeSystem());
        intakeSystem = robot.getComponent(IntakeSystem.class);
        drive = robot.addComponent(DriveClass.class, new DriveClass(DriveClass.ROBOT.GLADOS, new Location(-0.9, 0.4404 / 2, 180), DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE, DriveClass.DriveMode.LEFT));

        // Set intake system state and start motors
        intakeSystem.stopIntake();
        intakeSystem.spinMotor();
        robot.addComponent(DroneLauncher.class, new DroneLauncher());
        droneLauncher = robot.getComponent(DroneLauncher.class);
        // Initialize AprilTag detector
//        aprilTagDetector = new AprilTagDetector("cam", new Size(800, 448), hardwareMap, multipleTelemetry,
//                AprilTagDetector.PortalConfiguration.DEFAULT);

        // Display initialization completion message
        telemetry.addData(">", "Init finished. Press START");
        telemetry.update();
    }

    // Method to apply power function
    public double pow(double x) {
        return Math.pow(x, 2) * Math.signum(x);
    }

    // Method to handle force quit command
    private void handleForceQuit() {
        if (gamepad1.y && gamepad1.start && gamepad1.options) {
            robot.requestOpModeStop();
        }
    }

    // Method to initialize telemetry
    private void telemetryInit() {
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(dashboardTelemetry, telemetry);
    }

    // Method to update telemetry data
    private void updateTelemetry() {
        // Display relevant telemetry data
        multipleTelemetry.addData("drive distance", drive.getDistanceRightSensorDistance());
//        multipleTelemetry.addData("tags detected ", aprilTagDetector.getDetections().size());
        multipleTelemetry.addData("Allow Movement", allowMovement);
        multipleTelemetry.addData("Motor ticks: ", drive.fl.getCurrentPosition());
        multipleTelemetry.addData("Gather state", intakeSystem.state().toString());
        multipleTelemetry.addData("Field Oriented state", fieldOriented);
        multipleTelemetry.addData("Arm pos", arm.pos());
        multipleTelemetry.addData("Arm lift1 power", arm.lift1.getPower());
        multipleTelemetry.addData("Arm target pos", arm.targetPos());
        multipleTelemetry.addData("Field oriented", fieldOriented);
        multipleTelemetry.addData("Arm controlled power", pow(-robot.gamepad2.left_stick_y) * armBoost);
        multipleTelemetry.addData("Arm distance", arm.distanceSensorDistance());
        multipleTelemetry.addData("Alliance", robot.alliance.toString());
        multipleTelemetry.update();
    }

    public void armGotoLevel(Arm.Position position) {
        if (position == Arm.Position.Home) {
            arm.closeClaw(false);
        } else {
            arm.closeClaw(true);
            intakeSystem.stopIntake();
        }
        if (position.distanceFromBackboard != -1) {
            arm.placePixels(position);
        } else {
            arm.goToPos(position);
        }
    }
}
