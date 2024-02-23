package org.firstinspires.ftc.teamcode.centerstage.Systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Component;

public class IntakeSystem extends Component {
    public WheelsState state = WheelsState.Idle, prevState = WheelsState.Idle;
    public boolean initTime = true;
    Servo intakeServo1, intakeServo2;
    DcMotorEx motor;
    ElapsedTime timer;
    Arm arm;
    private boolean justStopped;
    private boolean isBusy;

    public IntakeSystem() {
    }

    public boolean IsBusy() {
        return isBusy;
    }

    public void stopIntake() {
        if (state() == WheelsState.Collect) {
            justStopped = true;
            timer.reset();
        }
        setState(WheelsState.Idle);
    }

    public void spit() {
        setState(WheelsState.Spit);
    }

    public void collect() {
        setState(WheelsState.Collect);
    }

    @Override
    public void init() {
        initTime = true;
        motor = hardwareMap.get(DcMotorEx.class, "intakeWheelsMotor");

        intakeServo1 = hardwareMap.get(Servo.class, "intakeServo1");
        intakeServo2 = hardwareMap.get(Servo.class, "intakeServo2");
        intakeServo2.setDirection(Servo.Direction.REVERSE);
        stopIntake();
        initTime = false;
    }

    @Override
    public void start() {
        this.arm = Arm.instance;
        timer = new ElapsedTime();
    }

    @Override
    public void update() {
        spinMotor();
        if (timer.seconds() >= 0.4 && justStopped && state() == WheelsState.Idle) {
            spit();
        } else if (timer.seconds() >= 2 && state() == WheelsState.Spit && justStopped) {
            justStopped = false;
            setState(WheelsState.Idle);
        }
    }

    public void setServoPos(double pos) {
        intakeServo1.setPosition(pos);
        intakeServo2.setPosition(pos);
    }

    public void setServoPos(ServoPos pos) {
        setServoPos(pos.servoPos);
    }

    void setState(WheelsState state) {
//        if (state == WheelsState.AvoidArm) return;
        prevState = state;
        this.state = state;
    }

    public void setStateAvoidArm() {
        state = WheelsState.AvoidArm;
    }

    /**
     * Moves the intake wheels
     */
    public void spinMotor() {
        double normalPower = 0.7;
        double spitPower = 0.6;
        if (isBusy) return; // Don't move if the system is busy
//        if (arm != null && arm.pos() < 800 && arm.pos() > 50 && state != WheelsState.Collect && !initTime) {
//            state = WheelsState.AvoidArm;
//        } else {
//        state = prevState;
        if (state == WheelsState.Spit) {
            setServoPos(ServoPos.Close);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setPower(spitPower);
        } else if (state == WheelsState.Idle) {
            setServoPos(ServoPos.Close);
            motor.setPower(0);
        } else if (state == WheelsState.Collect) {
            setServoPos(ServoPos.Open);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setPower(normalPower);
        }
    }

    public WheelsState state() {
        return this.state;
    }

    public enum ServoPos {
        Open(1), Close(0), Mid(0.5);
        private final double servoPos;

        ServoPos(double servoPos) {
            this.servoPos = servoPos;
        }
    }
//        if (state == WheelsState.AvoidArm && !initTime) {
//            setServoPos(ServoPos.Mid);
//            motor.setDirection(DcMotorSimple.Direction.FORWARD);
//            motor.setPower(normalPower);
//        }
//    }

    public enum WheelsState {
        Collect,
        Spit,
        Idle,
        AvoidArm
    }
}
