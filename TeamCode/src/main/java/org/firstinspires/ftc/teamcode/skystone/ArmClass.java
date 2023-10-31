package org.firstinspires.ftc.teamcode.skystone;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

public class ArmClass extends Thread {

    volatile private boolean stopFlag = true;

    volatile private DcMotorEx arm0 = null;
    volatile private DcMotorEx arm1 = null;
    volatile private DigitalChannel zeroArm0 = null;
    volatile private DigitalChannel zeroArm1 = null;

    volatile private Servo clamps = null;
    volatile private Servo clampsRotate = null;

    volatile private Toggle clampsState = new Toggle();
    volatile private Toggle clampsRotateState = new Toggle();


    volatile private double power = 1;
    volatile private double speed_boost = 0.5;

    public enum Mode {
        IDLE, MANUAL, HOME, PICK, STRAIGHT, BUILD, DROP,
        SKY1_STRETCH, SKY2_FOLD, SKY3_READY, SKY4_DROP, SKY5_DROP_BACK
    }

    volatile private Mode mode = Mode.IDLE;

    // volatile private Toggle SpeedModeArm0 = new Toggle();
    volatile private int posArm0 = 0;
    volatile private int posArm1 = 0;

    volatile private LinearOpMode opMode = null;
    volatile private DriveClass driveClass = null;

    static final int STAY = 999999;

    volatile private int currFloor = 0;

    public ArmClass(LinearOpMode opMode, DriveClass drive) {
        this.setName("ArmClass");
        RobotLog.d("%s", this.getName());

        this.opMode = opMode;
        this.driveClass = drive;
    }

    public void init(HardwareMap hardwareMap) {
        RobotLog.d("ArmClass: Init");
        arm0 = hardwareMap.get(DcMotorEx.class, "arm0");
        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
        zeroArm0 = hardwareMap.get(DigitalChannel.class, "zero_arm0");
        zeroArm1 = hardwareMap.get(DigitalChannel.class, "zero_arm1"); //          TODO change names

        clamps = hardwareMap.get(Servo.class, "clamps");
        clampsRotate = hardwareMap.get(Servo.class, "clamps_rotate");

        openClamps(false);
        rotateClamps(false);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        arm0.setDirection(DcMotorEx.Direction.FORWARD);
        arm1.setDirection(DcMotorEx.Direction.REVERSE);
        zeroArm0.setMode(DigitalChannel.Mode.INPUT);
        zeroArm1.setMode(DigitalChannel.Mode.INPUT);
        arm0.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        arm0.setTargetPositionTolerance(20);
        arm1.setTargetPositionTolerance(20);

        //==============================
        // PIDF control

        RobotLog.d("Arm Velocity PID =======================================================");
        PIDFCoefficients pidf0 = arm0.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf1 = arm1.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        RobotLog.d(pidf0.toString()); // p=2.000000 i=0.500000 d=0.000000 f=11.100006
        RobotLog.d(pidf1.toString()); // p=2.000000 i=0.500000 d=0.000000 f=11.100006

        // Arm 0
        pidf0.p = 5;
        pidf0.i = 5;
        pidf0.d = 0.1;
        arm0.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf0);
        arm0.setPositionPIDFCoefficients(7);

        // Arm 1
        pidf1.p = 5;
        pidf1.i = 5;
        pidf1.d = 0.1;
        arm1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf1);
        arm1.setPositionPIDFCoefficients(7);

        RobotLog.d(pidf0.toString());
        RobotLog.d(pidf1.toString());

        RobotLog.d("ArmClass: /init ");
    }

    public void begin() {
        RobotLog.d("ArmClass: begin");
        reset();
        mode = Mode.MANUAL;
        stopFlag = false;
    }

    public void reset() {
        RobotLog.d("ArmClass: reset");
        arm0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm0.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        setArmDriveMode(true);
    }

    public void openClamps(boolean open) {
        clampsState.set(open);
        if (open) {
            clamps.setPosition(0);
        } else {
            clamps.setPosition(1);
        }
    }

    public void toggleClamps() {
        boolean state = clampsState.toggle();
        openClamps(state);
    }

    public void rotateClamps(boolean rot) {
        clampsRotateState.set(rot);
        if (rot) {
            clampsRotate.setPosition(1);
        } else {
            clampsRotate.setPosition(0);
        }
    }

    public void toggleRotateClamps() {
        boolean state = clampsRotateState.toggle();
        rotateClamps(state);
    }

    public void setArmDriveMode(boolean drive) {
        if (drive) {
            arm0.setPower(0);
            arm0.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            arm1.setPower(0);
            arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } else {
            arm0.setTargetPosition(arm0.getCurrentPosition());
            arm0.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            arm0.setPower(power);
            arm1.setTargetPosition(arm1.getCurrentPosition());
            arm1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            arm1.setPower(power / 2);
        }
    }

    public void setBoost(double boost) {
        speed_boost = 0.4 + boost * 0.6;
    }

    public void moveArm0(double speed) {
        if (mode != Mode.MANUAL) {
//            RobotLog.d("Arm0 move interrupt");
//            this.interrupt();
            return;
        }

        if (zeroArm0.getState() == false) {
            speed = Math.max(0, speed);
        } else {
            // prevent arm from dropping under high moment when rest.
            if (Math.abs(speed) <= 0.01 && arm1.getCurrentPosition() > 500) {
                if (arm0.getCurrentPosition() < 2600) {
                    speed = 0.01; // give arm some minimal speed to hold up its position.
                } else if (arm0.getCurrentPosition() > 3800) {
                    speed = -0.01; // give arm some minimal speed to hold up its position.
                }
            }
        }
        arm0.setPower(speed * speed_boost);
    }

    public void moveArm1(double speed) {
        if (mode != Mode.MANUAL) {
//            RobotLog.d("Arm1 move interrupt");
//            this.interrupt();
            return;
        }
        if (zeroArm1.getState() == false) {
            speed = Math.max(0, speed);
        }
        if (speed > 0 && arm1.getCurrentPosition() > 970 - speed * 50) { // TODO:
            speed = 0;
        }
        arm1.setPower(speed * speed_boost / 2);
    }

    public void checkups() {
        if (zeroArm0.getState() == false && arm0.getTargetPosition() < arm0.getCurrentPosition()) {
            RobotLog.d("Arm0 checkups hit at: %d", arm0.getCurrentPosition());
            arm0.setTargetPosition(arm0.getCurrentPosition());
        }
        if (zeroArm1.getState() == false && arm1.getTargetPosition() < arm1.getCurrentPosition()) {
            RobotLog.d("Arm1 checkups hit at: %d", arm1.getCurrentPosition());
            arm1.setTargetPosition(arm1.getCurrentPosition());
        }
//        //TODO: checkout why the safety bray works when not supposed to.
//        int diff0 = Math.abs(arm0.getCurrentPosition() - posArm0);
//        if (diff0 <= 5) {
//            arm0.setTargetPosition(arm0.getCurrentPosition());
//            RobotLog.d("SAFETY BRAKE: Arm0 diff %d at %d", diff0, arm0.getCurrentPosition());
//        }
//        //
//        int diff1 = Math.abs(arm1.getCurrentPosition() - posArm1);
//        if (diff1 <= 5) {
//            arm1.setTargetPosition(arm1.getCurrentPosition());
//            RobotLog.d("SAFETY BRAKE: Arm1 diff %d at %d", diff1, arm1.getCurrentPosition());
//        }
//
//        posArm0 = arm0.getCurrentPosition();
//        posArm1 = arm1.getCurrentPosition();
//
//        if (opMode != null) {
//            opMode.telemetry.addData("Arms Switch", "Arm0:(%b), Arm1:(%b)", zeroArm0.getState(), zeroArm1.getState());
//            opMode.telemetry.addData("Arms", "Arm0 (%d), Arm1 (%d)", arm0.getCurrentPosition(), arm1.getCurrentPosition());
//            opMode.telemetry.update();
//        }
    }

    public void gootoo(int pos0, int pos1) throws InterruptedException {
        if (pos0 == STAY) pos0 = arm0.getCurrentPosition();
        if (pos1 == STAY) pos1 = arm1.getCurrentPosition();

        arm0.setTargetPosition(pos0);
        arm1.setTargetPosition(pos1);

        //RobotLog.d("Arm goto start to: Arm0:(%d)->(%d), Arm1(%d)->(%d)  ", arm0.getCurrentPosition(), arm0.getTargetPosition(), arm1.getCurrentPosition(), arm1.getTargetPosition());
        while ((arm0.isBusy() || arm1.isBusy()) && opMode.opModeIsActive()) {
            sleep(50);
            //RobotLog.d("Arm goto run at: Arm0:(%d), Arm1(%d)", arm0.getCurrentPosition(), arm1.getCurrentPosition());
            checkups();
        }

        //RobotLog.d("Arm goto complete at: Arm0:(%d), Arm1(%d)", arm0.getCurrentPosition(), arm1.getCurrentPosition());
    }

    public void gootoo(int pos0, int pos1, double timeout) throws InterruptedException {
        ElapsedTime runTime = new ElapsedTime();
        runTime.reset();
        if (pos0 == STAY) pos0 = arm0.getCurrentPosition();
        if (pos1 == STAY) pos1 = arm1.getCurrentPosition();

        arm0.setTargetPosition(pos0);
        arm1.setTargetPosition(pos1);

        //RobotLog.d("Arm goto start to: Arm0:(%d)->(%d), Arm1(%d)->(%d)  ", arm0.getCurrentPosition(), arm0.getTargetPosition(), arm1.getCurrentPosition(), arm1.getTargetPosition());
        while ((arm0.isBusy() || arm1.isBusy()) && runTime.seconds() < timeout && opMode.opModeIsActive()) {
            sleep(50);
            //RobotLog.d("Arm goto run at: Arm0:(%d), Arm1(%d)", arm0.getCurrentPosition(), arm1.getCurrentPosition());
            checkups();
        }

        //RobotLog.d("Arm goto complete at: Arm0:(%d), Arm1(%d)", arm0.getCurrentPosition(), arm1.getCurrentPosition());
    }

    public void pleaseDo(Mode newMode) {
        if (mode != Mode.MANUAL && mode != Mode.IDLE) {
            RobotLog.d("Arm pleaseDo busy doing %s", mode.toString());
            // interrupt();
            return;
        }
        RobotLog.d("Arm pleaseDo: %s", newMode.toString());
        mode = newMode;
        if (newMode != Mode.MANUAL || newMode != Mode.IDLE) {
            start();
        }
    }

    public void linearDo(Mode tmode) {
        mode = tmode;
        if (tmode != Mode.MANUAL || tmode != Mode.IDLE) {
            run();
        }
    }

    @Override
    public void run() {
        ElapsedTime timer = new ElapsedTime();
        try {
            power = 1;
            setArmDriveMode(false);
            switch (mode) {
                case HOME:
                    RobotLog.d("Arm do: HOME");
                    gootoo(500, 0);
                    rotateClamps(false);
                    openClamps(false);
                    driveClass.rollers(true);
                    sleep(500);
                    gootoo(-5000, -500);
                    reset();
                    driveClass.rollers(false);
                    RobotLog.d("Arm do: HOME/");
                    break;

                case PICK:
                    // peak up a cube and get back to drive position.
                    double rollersState = 0;
                    RobotLog.d("Arm do: PICK");
                    rotateClamps(false);

                    RobotLog.d("Arm do: PICK - Open openClamps");
                    openClamps(true);
                    RobotLog.d("Arm do: PICK - Go above");
                    gootoo(500, 0);
                    RobotLog.d("Arm do: PICK - Go down");
                    gootoo(-200, -200);
                    openClamps(false);
                    RobotLog.d("Arm do: PICK - Open Rollers");
                    driveClass.rollers(true);
                    RobotLog.d("Arm do: before sleep");
                    sleep(700);
                    RobotLog.d("Arm do: after sleep");
                    gootoo(400, 0);
                    driveClass.rollers(false);
                    RobotLog.d("Arm do: PICK/");
                    break;

                case BUILD:
                    RobotLog.d("Arm do: BUILD");
                    if (arm0.getTargetPosition() < 100)
                        gootoo(300, 0);

                    switch (currFloor) {
//                        case 1:
//                            RobotLog.d("Arm do: BUILD - floor 1");
//                            gootoo(830, STAY);
//                            gootoo(830, 560);
//                            gootoo(225, 555);
//                            break;
//                        case 2:
//                            RobotLog.d("Arm do: BUILD - floor 2");
//                            gootoo(930, STAY);
//                            gootoo(930, 600);
//                            gootoo(700, 730);
//                            break;
//                        case 3:
//                            RobotLog.d("Arm do: BUILD - floor 3");
//                            gootoo(1260, 80);
//                            gootoo(1030, 760);
//                            break;
//                        case 4:
//                            RobotLog.d("Arm do: BUILD - floor 4");
//                            gootoo(1630, 180);
//                            gootoo(1280, 870);
//                            break;
//                        case 5:
//                            RobotLog.d("Arm do: BUILD - floor 5");
//                            gootoo(1870, 540);
//                            gootoo(1485, 900);
//                            break;
//                        case 6:
//                            RobotLog.d("Arm do: BUILD - floor 6");
//                            gootoo(3000, 510);
//                            gootoo(3700, 540);
//                            break;
//                        case 7:
//                            RobotLog.d("Arm do: BUILD - floor 7");
//                            gootoo(3000, 800);
//                            gootoo(3615, 820);
//                            break;
//                        case 8:
//                            RobotLog.d("Arm do: BUILD - floor 8");
//                            gootoo(3000, 900);
//                            break;
                        default:
                            RobotLog.d("Arm do: BUILD - floor 7");
                            gootoo(3040, 890);
                            sleep(100);
                            gootoo(3470, 770);
                            break;
                    }

                    RobotLog.d("Arm do: BUILD/");
                    break;

                case SKY1_STRETCH: // stretch arm to start position
                    RobotLog.d("Arm do: STRACH");

                    gootoo(700, 0);
                    rotateClamps(true);
                    openClamps(true);
                    arm1.setPower(1);
                    sleep(200);
                    gootoo(STAY, 400);
                    arm0.setPower(0.8);
                    gootoo(480, 380);
                    RobotLog.d("Arm do: STRACH/");
                    break;

                case SKY2_FOLD: // after catch move arm back
                    gootoo(550, 100);
                    break;

                case SKY3_READY: // get ready to catch
                    gootoo(630, 430);
                    gootoo(480, 380);
                    break;

                case SKY4_DROP:
                    timer.reset();
                    gootoo(ArmClass.STAY, 800, 0.5);
                    sleep(50);
                    openClamps(true);
                    sleep(400);
                    openClamps(false);
                    gootoo(ArmClass.STAY, 0);
                    RobotLog.d("SKY4_DROP time: %f", timer.seconds());
                    break;

                case SKY5_DROP_BACK: // get ready to catch
                    RobotLog.d("Arm do: SKY5_DROP_BACK");
                    rotateClamps(false);
                    gootoo(5200, 20);
                    sleep(1000);
                    openClamps(true);
                    sleep(500);
                    linearDo(Mode.HOME);
                    RobotLog.d("Arm do: SKY5_DROP_BACK/");
                    break;

                default:
                    break;
            }
        } catch (InterruptedException e) {
            RobotLog.d("Arm Thread interrupted!");
        } catch (Exception e) {
            RobotLog.logStackTrace(e);
        }
        power = 1;
        setArmDriveMode(true);
        RobotLog.d("Arm Thread complete");
        mode = Mode.MANUAL;
    }

    public void end() {
        RobotLog.d("ArmClass:End()");
        stopFlag = true;
        interrupt();
        arm0.setPower(0);
        arm1.setPower(0);
//        arm0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mode = Mode.IDLE;
    }

    public void resumePower() {
        RobotLog.d("ArmClass: Resume");
        setArmDriveMode(true);
        mode = Mode.MANUAL;
        stopFlag = false;
    }

    public int getArm0Pos() {
        return arm0.getCurrentPosition();
    }

    public int getArm1Pos() {
        return arm1.getCurrentPosition();
    }

    public boolean getArm0Zero() {
        return zeroArm0.getState();
    }

    public boolean getArm1ZeroA() {
        return zeroArm1.getState();
    }

    public void floorPlus() {
        if (currFloor >= 8) {
            currFloor = 8;
        } else {
            currFloor++;
        }
    }

    public void floorMinus() {
        if (currFloor <= 1) {
            currFloor = 1;
        } else {
            currFloor--;
        }
    }
}