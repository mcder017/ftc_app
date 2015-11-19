package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by a on 11/9/2015.
 */
public class AutoTestOpBlue extends AutoTestOp {
    @Override
    public void init() {
        super.init();
        turnPower1 = 1;
        turnPower2 = -1;
    }

    public void loop () {


        super.loop();
    }



}
