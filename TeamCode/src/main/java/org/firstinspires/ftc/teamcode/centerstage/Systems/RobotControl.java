package org.firstinspires.ftc.teamcode.centerstage.Systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Component;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.RobotTelemetry;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.ThreadedComponent;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.Toggle;
import org.firstinspires.ftc.teamcode.centerstage.util.Location;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.MathUtil;

@ThreadedComponent
public class RobotControl extends Component {
    public static int boardAlignSensor = -1;
    @RobotTelemetry
    public static MultipleTelemetry multipleTelemetry;
    Gamepad gamepad1, gamepad2;
    // Robot components
    DriveClass drive;
    Arm arm;
    IntakeSystem intakeSystem;
    // Telemetry and dashboard
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    // Control variables
    boolean fieldOriented = true;
    double angle = 0;
    double targetHeading;
    int turningCount = 0;
    Gamepad.RumbleEffect rumbleEffect;
    Toggle turningToggle = new Toggle();
    Toggle robotOrientedToggle = new Toggle();
    double armBoost;
    DroneLauncher droneLauncher;
    double allianceDefaultHeading = 90;
    ElapsedTime lastFrameTimer;
    double lastTime;
    Toggle boardAlignToggle = new Toggle();

    @Override
    public void init() {
        gamepad1 = robot.gamepad1;
        gamepad2 = robot.gamepad2;
// Initialize robot components
        initSystems();
        rumbleEffect = new Gamepad.RumbleEffect.Builder().addStep(1.0, 1.0, 300).build();
        telemetry.addData(">", "Init finished. Press START");
        telemetry.update();
    }

    @Override
    public void start() {
        lastFrameTimer = new ElapsedTime();

        telemetry.addData(">", "DRIVE START");
        telemetry.update();
        drive.start();
        drive.resetOrientation(allianceDefaultHeading);
        telemetry.addData(">", "DRIVE START DONE");

        telemetry.addData(">", "INTAKE SYSTEM START");
        telemetry.update();
        intakeSystem.start();
        intakeSystem.stopIntake();
        intakeSystem.spinMotor();
        telemetry.addData(">", "INTAKE SYSTEM START DONE");
        telemetry.update();

        arm.start();
        // Initialize input and start subsystems
//        Input.init(robot, robot.gamepad1, robot.gamepad2);
        targetHeading = drive.getHeading();
        gamepad1.runRumbleEffect(rumbleEffect);
        Thread armControlThread = new Thread(() -> {
            while (robot.opModeIsActive() && !robot.isStopRequested()) {
                this.updateArmAndIntakeJoystick();
            }
        });
        armControlThread.start();
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
        lastTime = lastFrameTimer.seconds();
        updateDriverJoystick();
        updateTelemetry();
    }

    // Method to update Arm and Intake subsystems
    private void updateArmAndIntakeJoystick() {
        armBoost = (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) ? 0.4 : 1;

        if (gamepad2.start) {
            return;
        }
        // Arm control based on gamepad input
        else if (Math.abs(gamepad2.left_stick_y) > 0.05) {
            arm.setPower(pow(-gamepad2.left_stick_y) * armBoost);
        } else if (gamepad2.a) {
            armGotoLevel(Arm.Position.Home);
        } else if (gamepad2.b) {
            arm.hang();
        } else {
            arm.setMotorsPower(0);
        }
        // Claw control based on gamepad input
        if (gamepad2.right_bumper) {
            arm.openClaw(true); // close claw
//            gamepad2.runRumbleEffect(rumbleEffect);
        } else if (gamepad2.left_bumper) {
            arm.openClaw(false); // open claw
        } else if (gamepad2.dpad_up) {
            intakeSystem.stopIntake(); // stop intake
        } else if (gamepad2.dpad_down) {
            intakeSystem.collect(); // start intake
            arm.openClaw(false);
        } else if (gamepad2.right_stick_button) {
            intakeSystem.spit(); // start spit
        } else if (gamepad2.y) {
            arm.dropBottomPixel();
        }
    }

    // Method to update driving controls
    private void updateDriverJoystick() {
        if (gamepad1.start) {
            if (gamepad1.x) {
                drive.resetHeading(allianceDefaultHeading);
                gamepad1.runRumbleEffect(rumbleEffect);
                targetHeading = drive.getHeading();
            }
            return;
        }

        boardAlignToggle.update(gamepad1.a || gamepad1.b || gamepad1.y);
        robotOrientedToggle.update(gamepad1.ps);

        double fixToBackdrop = 0;
        // Check for reorientation command
        if (gamepad1.a) {
            fixToBackdrop = arm.alignToBoardTeleOp(Arm.Position.One);
        } else if (gamepad1.b) {
            fixToBackdrop = arm.alignToBoardTeleOp(Arm.Position.Two);
        } else if (gamepad1.y) {
            fixToBackdrop = arm.alignToBoardTeleOp(Arm.Position.Three);
        }
        if (boardAlignToggle.isReleased()) {
            boardAlignSensor = -1;
        }
        // Boost factor for driving speed
        double boost = (gamepad1.left_trigger > 0.05 || gamepad1.right_trigger > 0.05) ? 1.5 : 0.6;

        // Drive control based on gamepad input
        double y = pow(-gamepad1.left_stick_y) * boost;
        double x = pow(gamepad1.left_stick_x) * boost + fixToBackdrop;
        double turn = pow(gamepad1.right_stick_x /
                ((gamepad1.left_trigger > 0.05 || gamepad1.right_trigger > 0.05) ? 1 : 1.7)) * boost;
        // Update turning toggle state
        turningToggle.update(Math.abs(turn) > 0.02 || gamepad1.a);

//         Reset turning count if toggle is released
        if (turningToggle.isReleased()) {
            turningCount = 12;
        }

        // Decrease turning count if toggle is not pressed
        else if (!turningToggle.isPressed()) {
            turningCount--;
        }

//         Set target heading if turning count is zero
        if (turningCount == 0) {
            targetHeading = drive.getHeading();
        }
        // Apply rotation fix if turning count is less than zero
        if (gamepad1.right_bumper) {
            targetHeading = robot.alliance == AutoFlow.Alliance.BLUE ? 180 : -180;
            double delta = drive.getDeltaHeading(targetHeading);
            double gain = 0.015;
            turn = MathUtil.clamp(delta * gain, -0.5, 0.5);
        } else if (gamepad1.left_bumper) {
            targetHeading = 90 * (robot.alliance == AutoFlow.Alliance.RED ? -1 : 1);
            double delta = drive.getDeltaHeading(targetHeading);
            double gain = 0.02;
            turn = delta * gain;
        } else if (!turningToggle.isPressed() && turningCount < 0 && !gamepad1.a) {
            double delta = drive.getDeltaHeading(targetHeading);
            double gain = 0.02;
            turn = delta * gain;
        }
        if (robotOrientedToggle.isClicked()) {
            fieldOriented = !fieldOriented;
        }
        // Perform dpad-specific actions
        else if (gamepad1.dpad_left) drive.setPowerOriented(0, -0.3, 0, fieldOriented);
        else if (gamepad1.dpad_right) drive.setPowerOriented(0, 0.3, 0, fieldOriented);
        else if (gamepad1.dpad_up) drive.setPowerOriented(0.3, 0, 0, fieldOriented);
        else if (gamepad1.dpad_down) drive.setPowerOriented(-0.3, 0, 0, fieldOriented);
        else drive.setPowerOriented(y, x, turn, fieldOriented);
        //region hang and drone
        if (gamepad1.share) {
            droneLauncher.launch();
        }
        //endregion
    }

    // Method to initialize robot subsystems
    private void initSystems() {
        telemetry.addData(">", "Init in progress...");
        telemetry.update();
        // Initialize telemetry
        telemetryInit();
        arm = robot.addComponent(Arm.class);
        intakeSystem = robot.addComponent(IntakeSystem.class);
        drive = robot.addComponent(DriveClass.class,
                new DriveClass(DriveClass.ROBOT.GLADOS, new Location(-0.9, 0.4404 / 2, 180),
                        DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE, DriveClass.DriveMode.LEFT));
        droneLauncher = robot.addComponent(DroneLauncher.class);
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
        multipleTelemetry.addData("Time since last update", 1 / (lastFrameTimer.seconds() - lastTime));
        multipleTelemetry.addData("DISTANCE SENSOR", boardAlignSensor);
//        multipleTelemetry.addData("drive distance", drive.getDistanceRightSensorDistance());
//        multipleTelemetry.addData("Gather state", intakeSystem.state().toString());
//        multipleTelemetry.addData("Field Oriented state", fieldOriented);
        multipleTelemetry.addData("Arm pos", arm.pos());
//        multipleTelemetry.addData("Arm lift1 power", arm.lift1.getPower() * 100);
//        multipleTelemetry.addData("Arm distance", arm.distanceSensorDistance());
//        multipleTelemetry.addData("Arm velocity", arm.calculateVelocity() / 10);
//        multipleTelemetry.addData("Arm target pos", arm.targetPos());
        multipleTelemetry.update();
    }

    public void armGotoLevel(Arm.Position position) {
        if (position == Arm.Position.Home) {
            arm.openClaw(false);
        } else {
            arm.openClaw(true);
            intakeSystem.stopIntake();
        }
        arm.goToPos(position);
    }
}
