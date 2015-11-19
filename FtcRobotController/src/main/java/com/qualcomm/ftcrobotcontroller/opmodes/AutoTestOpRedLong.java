package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by a on 11/19/2015.
 */
public class AutoTestOpRedLong extends AutoTestOpRed1 {
    public AutoTestOpRedLong() {
        // Make sure the turn direction is set for red OR blue
        super();
        // We found this by dividing 30 ( we got 30inches because we added that to our previous 19 inches) by 1.414
        // ( the square root of 2)
        // got about 21 and multiplied it by 315 ( number of clicks in an inch) and got 6638 clicks.
        firstDistance = firstDistance + 6683;
        // by geometry, secondDistance will be the same amount more ase firstDistance
        secondDistance = secondDistance + 6683;



    }

}
