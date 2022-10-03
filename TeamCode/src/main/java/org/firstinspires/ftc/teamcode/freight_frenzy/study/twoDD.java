package org.firstinspires.ftc.teamcode.freight_frenzy.study;

import java.util.ArrayList;
import java.util.List;

/*** 2 Dimensional Double */
public class twoDD {
    public double var1 = 0;
    public double var2 = 0;

    public twoDD() {}
    public twoDD(double var1, double var2) {
        this.var1 = var1;
        this.var2 = var2;
    }

    public double getVar1() {
        return var1; }
    public void setVar1(double var1) {
        this.var1 = var1; }
    public double getVar2() {
        return var2; }
    public void setVar2(double var2) {
        this.var2 = var2; }
    public void setVars(double var1, double var2) {
        this.var1 = var1;
        this.var2 = var2;
    }
    public twoDD getVars() {
        return this;
    }
    public double[] toArray() {
        return new double[] {var1,var2}; }
    public List<Double> toList() {
        return new ArrayList<Double>() {

        };
    }

}
