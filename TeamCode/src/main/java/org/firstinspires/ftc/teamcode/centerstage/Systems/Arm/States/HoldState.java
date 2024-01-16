package org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States;

import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.MathUtil;

public class HoldState extends State<Arm> {
    Arm owner;

    @Override
    public void enter(Arm owner) {
        this.owner = owner;
    }

    @Override
    public void execute() {
        owner.setPower(0);
        // Continue running to position with the hold power
//        if (!MathUtil.approximately(owner.getPos(), owner.getTargetPos(), 10)) {
//            if (owner.getPos() > owner.getTargetPos()) {
//                owner.setMotorsPower(-0.3);
//            } else if (owner.getPos() < owner.getTargetPos()) {
//                owner.setMotorsPower(0.3);
//            }
//        }
    }

    @Override
    public void exit() {
        // Stop the motors when exiting the HoldState
        owner.setMotorsPower(0);
    }
}
