package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by a on 10/14/2015.
 */
public class AutoTestOp extends PushBotTelemetry {
    private int v_state = 0;
    private Servo v_servo_left_hand;
    private Servo v_servo_right_hand;

void m_myhand_position(double lefthand, double righthand){
    if(v_servo_right_hand != null) {


        v_servo_right_hand.setPosition (righthand);
    }
        if (v_servo_left_hand != null) {
            v_servo_left_hand.setPosition(lefthand);
        }
}

    public void init() {
super.init();
        try
        {
            v_servo_left_hand = hardwareMap.servo.get ("left_hand");

        }
        catch (Exception p_exeception)
        {
            m_warning_message ("left_hand");
            DbgLog.msg(p_exeception.getLocalizedMessage());

            v_servo_left_hand = null;
        }

        try
        {
            v_servo_right_hand = hardwareMap.servo.get ("right_hand");

        }
        catch (Exception p_exeception)
        {
            m_warning_message ("right_hand");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_right_hand = null;
        }
        m_myhand_position(0.0,0.54);

        //state is v_state
        v_state = 0;

    }

    @Override
    public void start() {
        super.start();
        m_myhand_position(0.76,0.76);
    }

    public void loop () {
        float l_left_arm_power;
  double currentTime = getRuntime();
        float   l_left_drive_power;
         float  l_right_drive_power;
        switch (v_state) {
            case 0:
                resetStartTime();
                 l_left_drive_power = (0.5f);
                 l_right_drive_power = (0.5f);
                set_drive_power (l_left_drive_power, l_right_drive_power);
              v_state++;
                break;
            case 1:
                if (currentTime >= 3.0) {

                    set_drive_power(0.0, 0.0);
                    v_state++;
                }
                break;
            case 2:
               resetStartTime();
                 l_left_drive_power = (-0.5f);
                 l_right_drive_power = (0.5f);
                set_drive_power (l_left_drive_power, l_right_drive_power);


                v_state++;

                break;

            case 3:
                if (currentTime >=0.7){
                    set_drive_power (0.0, 0.0);
                    v_state++;
                } break;

            case 4:
                resetStartTime();
                l_left_drive_power = (0.5f);
                l_right_drive_power = (0.5f);
                set_drive_power (l_left_drive_power, l_right_drive_power);
                v_state++;
                break;
            case 5:
                    if(currentTime >= .5)  {
                        set_drive_power (0.0, 0.0);
                         v_state++;
                    }         break;
            case 6:
                 resetStartTime();
                l_left_arm_power = (0.5f);
                 m_left_arm_power(l_left_arm_power);
                v_state++;
                break;
            case 7:
                 if (currentTime >= .5)  {
                     l_left_arm_power = (.0f);
                     m_left_arm_power(l_left_arm_power);
                 }    v_state++;
                break;


            case 8:
             resetStartTime();
              l_left_drive_power = (0.5f);
              l_right_drive_power = (0.5f);
              set_drive_power (l_left_drive_power, l_right_drive_power);
                v_state++;
              break;
            case 9:
                  if(currentTime >= .1)  {
                              set_drive_power (0.0,0.0);
                  }          v_state++; break;
            case 10:
                 resetStartTime();
                l_left_arm_power = (-0.1f);
                 m_left_arm_power(l_left_arm_power);
                v_state++;
            case 11:
            case 12:
                resetStartTime();
                 l_left_drive_power = (-0.5f);
                 l_right_drive_power = (-0.5f);
                 set_drive_power (l_left_drive_power, l_right_drive_power);
                 break;
            case 13:
                if(currentTime >= .5)    {
                    set_drive_power (0.0,0.0)  ;
                }                v_state++;   break;
            case 14:
              resetStartTime();
               l_left_drive_power = (-0.5f);
               l_right_drive_power = (-0.5f);
               set_drive_power (l_left_drive_power, l_right_drive_power);
                 v_state++;
               break;
            case 15:
             if(currentTime >= .5)    {
                 set_drive_power (0.0,0.0)  ;
             }                v_state++;   break;
            case 16:
             resetStartTime();
              l_left_drive_power = (-0.5f);
              l_right_drive_power = (0.5f);
              set_drive_power (l_left_drive_power, l_right_drive_power);
                v_state++;
            case 17:







            default: v_state = 0;
                //Do nothing

        }

        //
        // Send telemetry data to the driver station.
        //
        update_telemetry(); // Update common telemetry
        telemetry.addData
                ( "14"
                        , "State: " + v_state + " Time " + currentTime
                );
    }

}
