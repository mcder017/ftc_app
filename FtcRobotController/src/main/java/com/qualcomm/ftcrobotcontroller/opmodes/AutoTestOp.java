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
    final double DRIVE_DISTANCE_6_INCHES = 2880;

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
        m_myhand_position(0.76, 1.0 - 0.76);

    }

    public void loop () {
        float l_left_arm_power;
  double currentTime = getRuntime();
        float   l_left_drive_power;
         float  l_right_drive_power;
        switch (v_state) {

            case 0:

                if(have_drive_encoders_reset()){




              v_state++;
                }
                break;
            case 1:
                if (drive_using_encoders(1.0, 1.0, DRIVE_DISTANCE_6_INCHES, DRIVE_DISTANCE_6_INCHES)) {
                    v_state++;
                }

                break;
            case 2:
                if (have_drive_encoders_reset()) {
                    v_state++;
                }

                v_state++;

                break;

            case 3:
                if (drive_using_encoders(-1.0, 1.0, -DRIVE_DISTANCE_6_INCHES, DRIVE_DISTANCE_6_INCHES)) {
                    v_state++;
                }
                break;

            case 4:
                if (have_drive_encoders_reset()) {
                    v_state++;
                }
                break;
            case 5:
                if (drive_using_encoders(1, 1, DRIVE_DISTANCE_6_INCHES, DRIVE_DISTANCE_6_INCHES )) {
                    v_state++;
                }
            case 6:
                 resetStartTime();
                l_left_arm_power = (0.5f);
                 m_left_arm_power(l_left_arm_power);
                v_state++;
                break;
            case 7:
                 if (currentTime >= .2)  {
                     l_left_arm_power = (.0f);
                     m_left_arm_power(l_left_arm_power);
                 }    v_state++;
                break;


            case 8:
                if (drive_using_encoders(1.0, 1.0, DRIVE_DISTANCE_6_INCHES, DRIVE_DISTANCE_6_INCHES)) {
                    v_state++;
                }
              break;
            case 9:
                if (have_drive_encoders_reset()) {
                    v_state++;
                }
            case 10:
                 resetStartTime();
                l_left_arm_power = (-0.1f);
                 m_left_arm_power(l_left_arm_power);
                v_state++;
            case 11:
                v_state++;
                break;
            case 12:
                if (have_drive_encoders_reset()) {
                    v_state++;
                }
                 break;
            case 13:
                if (drive_using_encoders(1.0, 1.0, 1000, 1000)) {
                    v_state++;
                }break;
            case 14:
                if (drive_using_encoders(-1.0, -1.0, DRIVE_DISTANCE_6_INCHES, DRIVE_DISTANCE_6_INCHES)) {
                    v_state++;
                }
               break;
            case 15:
                if (have_drive_encoders_reset()) {
                    v_state++;
                }
            case 16:
                if (drive_using_encoders(-1.0, 1.0, DRIVE_DISTANCE_6_INCHES, DRIVE_DISTANCE_6_INCHES)) {
                    v_state++;
                }
            case 17:







            default:
                //Do nothing
               break;
        }

        //
        // Send telemetry data to the driver station.
        //
        update_telemetry(); // Update common telemetry
        telemetry.addData
                ("14"
                        , "State: " + v_state + " Time " + currentTime
                );
        telemetry.addData("15", "Distance L "+a_left_encoder_count()+" R "+ a_right_encoder_count() );
    }

}
