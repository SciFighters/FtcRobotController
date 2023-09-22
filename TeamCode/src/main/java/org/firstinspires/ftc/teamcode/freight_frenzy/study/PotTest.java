package org.firstinspires.ftc.teamcode.freight_frenzy.study;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.Toggle;

@Disabled
@TeleOp
public class PotTest extends LinearOpMode {
    //declaring variables
    private DcMotorEx hand = null;
    private AnalogInput pot = null;
    private final double handRang = 5000;
    private DigitalChannel limit_front = null, limit_back = null;
    public static double maxVoltagePot = 0;
    Toggle maxVoltageToggle = new Toggle();

    private int patent(double PowerVoltage, double MaxVoltage) {
        int tick = (int)(handRang * (1 - PowerVoltage / MaxVoltage));
        return tick;
    }



    private void initPot() {
        // Initializing mechanical components
        pot = hardwareMap.get(AnalogInput.class, "potentiometer");
        hand = hardwareMap.get(DcMotorEx.class, "hand");
        limit_back = hardwareMap.get(DigitalChannel.class, "hand_limit_front");
        limit_front = hardwareMap.get(DigitalChannel.class, "hand_limit_back");
        // Setting motors
        hand.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hand.setDirection(DcMotorEx.Direction.FORWARD);

        // Adding telemetry
        telemetry.addData(">", "Hardware Initialized");
        telemetry.update();
    }

    private void hand_drive(double power){
        double powGain = 0.7;
        if(this.hand.getMode().equals(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER)) this.hand.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        if((limit_back.getState() && power < 0) || (limit_front.getState() && power > 0))
            this.hand.setPower(power * powGain);
    }



    private void shutHand() { this.hand.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); }
    @Override
    public void runOpMode() {
        initPot();
        waitForStart();
        while (opModeIsActive()) {
            // hand
            double powerInput = gamepad1.left_stick_y;
            double handPower = Math.abs(Math.pow(powerInput, 2)) * Math.signum(powerInput);
            hand_drive(handPower);
            if(gamepad1.y) this.shutHand();
            // emergency exit
            if(gamepad1.x) break;
            // Potentiometer max voltage value
            maxVoltagePot = Math.max(pot.getVoltage(), maxVoltagePot);
            maxVoltageToggle.update(gamepad1.a);
            if(maxVoltageToggle.isClicked()) telemetry.addData("current max voltage: ", String.valueOf(maxVoltagePot));
            // Telemetry
            telemetry.addData("potentiometer V", pot.getVoltage());
            telemetry.addData("potentiometer %",pot.getVoltage() / pot.getMaxVoltage() * 100);
            telemetry.addData("ticks: ", hand.getCurrentPosition());
            telemetry.addData("current offset", patent(pot.getVoltage(), 3.33));
            telemetry.addData("current diff", hand.getCurrentPosition() - patent(pot.getVoltage(), 3.33));
            telemetry.update();
        } //     |/0\v/0\|
    }
}
