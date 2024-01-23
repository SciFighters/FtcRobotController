package org.firstinspires.ftc.teamcode.centerstage.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Component;

public class IntakeSystem extends Component {
    Servo intakeServo1, intakeServo2;
    DcMotorEx motor;
    public WheelsState state, prevState;
    boolean initTime = true;
    Arm arm;

    public static enum ServoPos {
        Open(1), Close(0), Mid(0.5);
        private final double servoPos;

        ServoPos(double servoPos) {
            this.servoPos = servoPos;
        }
    }

    public enum WheelsState {
        Collect, Spit, Idle, AvoidArm
    }

    private boolean isBusy;


    public boolean IsBusy() {
        return isBusy;
    }


    public void setStateIdle() {
        setState(WheelsState.Idle);

    }

    public void setStateSpit() {
        setState(WheelsState.Spit);
    }

    public void setStateCollect() {
        setState(WheelsState.Collect);

    }

    @Override
    public void init() {
        motor = hw.get(DcMotorEx.class, "intakeWheelsMotor");

        intakeServo1 = hw.get(Servo.class, "intakeServo1");
        intakeServo2 = hw.get(Servo.class, "intakeServo2");
        intakeServo2.setDirection(Servo.Direction.REVERSE);
        setStateIdle();
        initTime = false;
    }

    @Override
    public void start() {
        arm = robot.getComponent(Arm.class);
    }

    @Override
    public void loop() {
        spinMotor();
    }

    public void setServoPos(double pos) {
        intakeServo1.setPosition(pos);
        intakeServo2.setPosition(pos);
    }

    public void setServoPos(ServoPos pos) {
        setServoPos(pos.servoPos);
    }

    void setState(WheelsState state) {
        if (state == WheelsState.AvoidArm) return;
        prevState = state;
        this.state = state;
    }

    /**
     * Moves the intake wheels
     */
    public void spinMotor() {
        double normalPower = 0.7;
        double spitPower = 0.6;
        if (isBusy) return; // Don't move if the system is busy
        if (arm != null && arm.getPos() < 800 && arm.getPos() > 25 && state != WheelsState.Collect && !initTime) {
            state = WheelsState.AvoidArm;
        } else {
            state = prevState;
            if (state == WheelsState.Spit) {
                setServoPos(ServoPos.Close);
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
                motor.setPower(spitPower);
            } else if (state == WheelsState.Idle) {
                setServoPos(ServoPos.Close);
                motor.setPower(0);
                return;
            } else if (state == WheelsState.Collect) {
                setServoPos(ServoPos.Open);
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
                motor.setPower(normalPower);
            }
        }
        if (state == WheelsState.AvoidArm && !initTime) {
            setServoPos(ServoPos.Mid);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setPower(normalPower);
        }
    }

    public WheelsState getState() {
        return this.state;
    }
}
