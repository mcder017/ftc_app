package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by a on 10/14/2015.
 */
public class AutoTestOp extends PushBotTelemetry {
    private int v_state = 0;
    private Servo v_servo_left_hand;
    private Servo v_servo_right_hand;
    public double turnPower1 = 1;
    public double turnPower2 = -1;
    double firstDistance = 4.0;
    final double DRIVE_DISTANCE_6_INCHES = 2600;
    double ninety_degree_turn = 4700;
    double arm_up = 500;
    double arm_down = 1000;
    double Arm_to_bar_time = .3;
    double Pull_up_time = 1.0;
    private DcMotor v_motor_right_arm;
    //--------------------------------------------------------------------------
    //
    // v_motor_left_arm
    //
    /**
     * Manage the aspects of the left arm motor.
     */
    private DcMotor v_motor_left_arm;

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
        reset_drive_encoders();
        reset_left_arm_encoder();
        // Connect the arm motor.
        //
        try
        {
            v_motor_right_arm = hardwareMap.dcMotor.get ("right_arm");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("right_arm");
            DbgLog.msg(p_exeception.getLocalizedMessage());

            v_motor_right_arm = null;
        }
        try
        {
            v_motor_left_arm = hardwareMap.dcMotor.get ("left_arm");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("left_arm");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_motor_left_arm = null;
        }

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
                //Drive from wall towards climber
                if (drive_using_encoders(1.0, 1.0, DRIVE_DISTANCE_6_INCHES*9, DRIVE_DISTANCE_6_INCHES*9)) {
                    v_state++;
                }

                break;
            case 2:
                if (have_drive_encoders_reset()) {
                    v_state++;
                }



                break;

            case 3:
                //Turn towards climber\\
                if (drive_using_encoders(turnPower2,turnPower1,ninety_degree_turn,ninety_degree_turn)) {
                    v_state++;
                }
                break;

            case 4:
                if (have_drive_encoders_reset()) {
                    v_state++;
                }
                break;
            case 5:
                //lower arm
                if (arm_using_encoders( -.25, arm_down )) {
                    run_without_left_arm_encoder();
                    m_left_arm_power(0.1);
                    v_state++;
                }
                break;
            case 6:
                if (have_drive_encoders_reset()) {
                    v_state++;
                }


                break;
            case 7:
                //Driving to hit climber
                if (drive_using_encoders(1.0, 1.0, DRIVE_DISTANCE_6_INCHES*1.5, DRIVE_DISTANCE_6_INCHES*1.5)) {
                    v_state++;
                }
                break;
            case 8:
            if (have_drive_encoders_reset()) {
                v_state++;
            }
                break;

            case 9:
                //Lift arm
                if(arm_using_encoders(.25f,arm_up)) {
                 v_state++;
                }

                break;
            case 10:
                if(has_left_arm_encoder_reset()) {
                    v_state++;
                }

                break;


            case 11:
                //Backing up from climber
                if (drive_using_encoders(-1.0, -1.0, DRIVE_DISTANCE_6_INCHES*1, DRIVE_DISTANCE_6_INCHES*1)) {
                    v_state++;
                }
              break;
            case 12:
                if (have_drive_encoders_reset()) {
                    v_state++;
                }
                break;
            case 13:
             v_state++;

                break;
            case 14:
              v_state++;

                break;

            case 15:
                // Reverse turn back towards mountain
                if (drive_using_encoders(turnPower1, turnPower2, ninety_degree_turn,ninety_degree_turn)) {
                    v_state++;
                }break;
            case 16:
                if (have_drive_encoders_reset()) {
                    v_state++;
                }
                break;

            case 17:
                //back up to be parallel with mountain
                if (drive_using_encoders(-1.0, -1.0, DRIVE_DISTANCE_6_INCHES*1, DRIVE_DISTANCE_6_INCHES*1)) {
                    v_state++;
                }
               break;
            case 18:
                if (have_drive_encoders_reset()) {
                    v_state++;
                }
                break;
            case 19:
                // Turn to face mountain
                if (drive_using_encoders(turnPower2, turnPower1,ninety_degree_turn,ninety_degree_turn)) {
                    v_state++;
                }
                break;
            case 20:
                if (have_drive_encoders_reset()) {
                    v_state++;
                    break;
                }
            case 21 :
                //Drive up mountain
                if (drive_using_encoders(1.0, 1.0,DRIVE_DISTANCE_6_INCHES*3,DRIVE_DISTANCE_6_INCHES*3)) {
                    v_state++;
                }



                break;
            case 22:
                //lower arm towards bar
                resetStartTime();
                run_without_left_arm_encoder();
                m_left_arm_power(-1);

                v_state++;
                break;
            case 23:
                //stop arm after short time and begin pulling and driving
                if(currentTime >=Arm_to_bar_time) {
                    m_left_arm_power(0.0);
                    run_without_drive_encoders();
                    m_right_arm_power(1.0);
                    set_drive_power(1.0,1.0);
                    resetStartTime();
                    v_state++;

                }
                break;
            case 24:
                //Stop everything
                if(currentTime >=Pull_up_time) {
                    m_right_arm_power(0.0);
                    set_drive_power(0.0,0.0);
                    v_state++;
                }
                break;








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
        telemetry.addData("16", "Right Arm Power" + a_right_arm_power() );
    }
    //--------------------------------------------------------------------------
    //
    // a_left_encoder_count
    //
    /**
     * Access the left encoder's count.
     */
    int a_left_arm_encoder_count ()
    {
        int l_return = 0;

        if (v_motor_left_arm != null)
        {
            l_return = v_motor_left_arm.getCurrentPosition ();
        }

        return l_return;

    } // a_left_encoder_count

    //--------------------------------------------------------------------------
    //
    // has_left_drive_encoder_reset
    //
    /**
     * Indicate whether the left drive encoder has been completely reset.
     */
    boolean has_left_arm_encoder_reset ()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Has the left encoder reached zero?
        //
        if (a_left_arm_encoder_count () == 0)
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_left_drive_encoder_reset
//--------------------------------------------------------------------------
    //
    // reset_left_drive_encoder
    //
    /**
     * Reset the left drive wheel encoder.
     */
    public void reset_left_arm_encoder ()

    {
        if (v_motor_left_arm != null)
        {
            v_motor_left_arm.setChannelMode
                    ( DcMotorController.RunMode.RESET_ENCODERS
                    );
        }

    } // reset_left_drive_encoder
    public void run_using_left_arm_encoder ()

    {
        if (v_motor_left_arm != null)
        {
            v_motor_left_arm.setChannelMode
                    ( DcMotorController.RunMode.RUN_USING_ENCODERS
                    );
        }

    } // run_using_left_drive_encoder
    public void run_without_left_arm_encoder ()

    {
        if (v_motor_left_arm != null)
        {
            if (v_motor_left_arm.getChannelMode () ==
                    DcMotorController.RunMode.RESET_ENCODERS)
            {
                v_motor_left_arm.setChannelMode
                        ( DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }

    } // run_without_left_drive_encoder
    //--------------------------------------------------------------------------
    //
    // drive_using_encoders
    //
    /**
     * Indicate whether the drive motors' encoders have reached a value.
     */
    boolean arm_using_encoders
    ( double p_left_power

            , double p_left_count

    )

    {
        //
        // Assume the encoders have not reached the limit.
        //
        boolean l_return = false;

        //
        // Tell the system that motor encoders will be used.
        //
        run_using_left_arm_encoder ();

        //
        // Start the drive wheel motors at full power.
        //
        m_left_arm_power (p_left_power);

        //
        // Have the motor shafts turned the required amount?
        //
        // If they haven't, then the op-mode remains in this state (i.e this
        // block will be executed the next time this method is called).
        //
        if (has_left_arm_encoder_reached (p_left_count))
        {
            //
            // Reset the encoders to ensure they are at a known good value.
            //
            reset_left_arm_encoder();

            //
            // Stop the motors.
            //
            m_left_arm_power (0.0f);

            //
            // Transition to the next state when this method is called
            // again.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // drive_using_encoders
    //--------------------------------------------------------------------------
    //
    // has_left_drive_encoder_reached
    //
    /**
     * Indicate whether the left drive motor's encoder has reached a value.
     */
    boolean has_left_arm_encoder_reached (double p_count)

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        if (v_motor_left_arm != null)
        {
            //
            // Has the encoder reached the specified values?
            //
            // TODO Implement stall code using these variables.
            //
            if (Math.abs (v_motor_left_arm.getCurrentPosition ()) > p_count)
            {
                //
                // Set the status to a positive indication.
                //
                l_return = true;
            }
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_left_drive_encoder_reached
    void m_right_arm_power (double p_level)
    {
        if (v_motor_right_arm != null)
        {
            v_motor_right_arm.setPower (p_level);
        }

    } // m_left_arm_power
    double a_right_arm_power ()
    {
        double l_return = 0.0;

        if (v_motor_right_arm != null)
        {
            l_return = v_motor_right_arm.getPower ();
        }

        return l_return;

    } // a_left_drive_power
}
