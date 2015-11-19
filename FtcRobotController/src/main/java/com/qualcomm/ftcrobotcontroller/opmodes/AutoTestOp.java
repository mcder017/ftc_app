package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.Map;

/**
 * Created by a on 10/14/2015.
 */
public class AutoTestOp extends PushBotTelemetry {
    private int v_state = 0;
    private Servo v_servo_left_hand;
    private Servo v_servo_right_hand;
    public double turnPower1 = 1;
    public double turnPower2 = -1;
    final double DRIVE_DISTANCE_6_INCHES = 2600;
    final double ninety_degree_turn = 4250;
    double short_last_turn = 4000;
    final double arm_up = 500;
    final double arm_down = 875;
    double Arm_to_bar_time = .3;
    double Pull_up_time = 1.0;
    private DcMotor v_motor_right_arm;
    boolean left_arm_movement = false;
    double firstDistance =DRIVE_DISTANCE_6_INCHES*9;
    double secondDistance = DRIVE_DISTANCE_6_INCHES*1.25;

    //holding arm in place
    final int LEFT_ARM_BEGINNING = 10000;
    private int left_arm_encoder_old=LEFT_ARM_BEGINNING;
    final double HOLD_UP_ARM = 0.08;
    final double HOLD_HOLD_ARM = 0.03;
    final int HOLD_RANGE_ARM = -10;
    final float ARM_MOVE_SPEED = 0.175f;

    //Arm safety plan
    double save_leftarm_encoder=0;
    double save_armtime=0;
    boolean arm_stalled=false;
    final double WAIT_TIME_ARM=0.5;
    final double LEFT_ARM_THRESHHOLD=30;
    // sensors
    TouchSensor v_sensor_touch = null;
    GyroSensor sensorGyro = null;
    int start_heading = 10000;



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
        try
        {
            v_sensor_touch = hardwareMap.touchSensor.get ("sensor_touch");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("sensor_touch");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_sensor_touch = null;
        }
        try {
            // get a reference to our GyroSensor object.
            sensorGyro = hardwareMap.gyroSensor.get("gyro");

            // calibrate the gyro.
            sensorGyro.calibrate();
        }
        catch (Exception p_exception) {
            m_warning_message ("gyro");
            DbgLog.msg(p_exception.getLocalizedMessage());

            sensorGyro = null;

        }
        m_myhand_position(0.0, 0.54);
        reset_drive_encoders();
        reset_left_arm_encoder();

        //state is v_state
        v_state = 0;

    }

    @Override
    public void start() {
        super.start();
        m_myhand_position(0.76, 1.0 - 0.76);
        reset_drive_encoders();
        reset_left_arm_encoder();

    }

    public void loop () {
        float l_left_arm_power;
  double currentTime = getRuntime();
        float   l_left_drive_power;
         float  l_right_drive_power;
        switch (v_state) {

            case 0:

                if(have_drive_encoders_reset()){
                    if (my_has_left_arm_encoder_reset()) {
                        if(a_gyro_ready()) {
                            v_state++;
                        }

                    }
                }
                break;
            case 1:
                //Drive from wall towards climber
                if (drive_using_encoders(1.0, 1.0, firstDistance, firstDistance)) {
                    v_state++;
                }

                break;
            case 2:
                if (have_drive_encoders_reset()) {
                    v_state++;
                    start_heading = a_current_heading();
                }



                break;

            case 3:
                //Turn towards climber\\
               // if (drive_using_encoders(turnPower2,turnPower1,ninety_degree_turn,ninety_degree_turn)) {
                run_using_encoders();
                set_drive_power(turnPower2,turnPower1 );
                if (magnitude_turned_degrees(start_heading) >= 90) {
                    set_drive_power(0.0,0.0);
                    v_state++;
                    reset_drive_encoders();
                }
                break;

            case 4:
                if (have_drive_encoders_reset()) {
                    v_state++;
                    save_leftarm_encoder=a_left_arm_encoder_count();
                    save_armtime=currentTime;
                }
                break;
            case 5:
                //lower arm
                left_arm_movement = true;
                if (arm_using_encoders( -.25f, arm_down )) {
                    v_state++;
                }
                //check for stall
                if (currentTime >=save_armtime+WAIT_TIME_ARM ){
                    if (Math.abs(a_left_arm_encoder_count()-save_leftarm_encoder)<LEFT_ARM_THRESHHOLD){
                        m_left_arm_power(0.0);
                        reset_left_arm_encoder();
                        arm_stalled=true;
                        v_state++;
                    }
                    else {
                        save_leftarm_encoder=a_left_arm_encoder_count();
                        save_armtime=currentTime;
                    }
                }
                break;
            case 6:
                left_arm_movement = false;
                if (have_drive_encoders_reset()) {
                    v_state++;
                }


                break;
            case 7:
                //Driving to hit climber
                if (drive_using_encoders(1.0, 1.0, secondDistance, secondDistance)) {
                    v_state++;
                }
                break;
            case 8:
            if (have_drive_encoders_reset()) {
                v_state++;
                save_leftarm_encoder=a_left_arm_encoder_count();
                save_armtime=currentTime;
            }
                break;

            case 9:
                //Lift arm
                left_arm_movement = true;
                if(arm_using_encoders(.25f,arm_up)) {
                 v_state++;

                }
                //check for stall
                if (currentTime >=save_armtime+WAIT_TIME_ARM ){
                    if (Math.abs(a_left_arm_encoder_count()-save_leftarm_encoder)<LEFT_ARM_THRESHHOLD){
                        m_left_arm_power(0.0);
                        reset_left_arm_encoder();
                        arm_stalled=true;
                        v_state++;
                    }
                    else {
                        save_leftarm_encoder=a_left_arm_encoder_count();
                        save_armtime=currentTime;
                    }
                }
                break;
            case 10:
               if(my_has_left_arm_encoder_reset()) {
                   v_state++;
               }

                break;


            case 11:
                //Backing up from climber
                left_arm_movement = false;
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
                start_heading = a_current_heading();
              v_state++;

                break;


            case 15:
                // Reverse turn back towards mountain
               // if (drive_using_encoders(turnPower1, turnPower2, ninety_degree_turn,ninety_degree_turn)) {
                run_using_encoders();
                set_drive_power(turnPower1,turnPower2 );
                if (magnitude_turned_degrees(start_heading) >= 90) {
                    set_drive_power(0.0,0.0);
                    v_state++;
                    reset_drive_encoders();
                }


                break;
            case 16:
                if (have_drive_encoders_reset()) {
                    v_state++;
                }
                break;

            case 17:
                //back up to be parallel with mountain
                if (drive_using_encoders(-1.0, -1.0, DRIVE_DISTANCE_6_INCHES*.8, DRIVE_DISTANCE_6_INCHES*.8)) {
                    v_state++;
                }
               break;
            case 18:
                if (have_drive_encoders_reset()) {
                    v_state++;
                    start_heading = a_current_heading();
                }
                break;
            case 19:

                // Turn to face mountain
                //if (drive_using_encoders(turnPower2, turnPower1,short_last_turn,short_last_turn)) {
                run_using_encoders();
                set_drive_power(turnPower2,turnPower1 );
                if (magnitude_turned_degrees(start_heading) >= 90) {
                    set_drive_power(0.0,0.0);
                    v_state++;
                    reset_drive_encoders();
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
                left_arm_movement = true;
                resetStartTime();
                run_without_left_arm_encoder();
                m_left_arm_power(-1);

                v_state++;
                save_leftarm_encoder=a_left_arm_encoder_count();
                save_armtime=currentTime;
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
        // if right arm is not being used move right arm to top
        if(v_state < 23) {
            if(!v_sensor_touch.isPressed() ){
                m_right_arm_power(-1.0);
            }else {
                m_right_arm_power(0.0);
            }

        }




        if (left_arm_encoder_old == LEFT_ARM_BEGINNING) {
            left_arm_encoder_old = a_left_arm_encoder_count();
        }
        if (left_arm_movement) {
            left_arm_encoder_old = a_left_arm_encoder_count();
        }
        else {
            final int currentposition = a_left_arm_encoder_count();
            int arm_change = currentposition - left_arm_encoder_old;
            if (arm_change <= HOLD_RANGE_ARM) {
                m_left_arm_power(HOLD_UP_ARM);
            }
            else if (arm_change >= 0) {
                m_left_arm_power(0.0);
            }
            else {
                // keeping power on might burn out motor
                m_left_arm_power (0.0);
            }
        }

        //
        // Send telemetry data to the driver station.
        //
        update_telemetry(); // Update common telemetry
        telemetry.addData
                ("14"
                        , "State: " + v_state + " Time " + currentTime
                );
        telemetry.addData
                ("12"
                        , "Left Arm encoder: " + a_left_arm_encoder_count()
                );
        telemetry.addData("15", "Distance L "+a_left_encoder_count()+" R "+ a_right_encoder_count() );
        telemetry.addData("16", "Right Arm Power " + a_right_arm_power() );
        telemetry.addData("19", (arm_stalled ? "Left arm STALLED" : "Left arm no stall")
            );
    }

    boolean my_has_left_arm_encoder_reset()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Has the left encoder reached zero?
        //
        if (a_left_arm_encoder_count() == 0)
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
            v_motor_left_arm.setMode
                    ( DcMotorController.RunMode.RESET_ENCODERS
                    );
        }

    } // reset_left_drive_encoder
    public void run_using_left_arm_encoder ()

    {
        if (v_motor_left_arm != null)
        {
            v_motor_left_arm.setMode
                    ( DcMotorController.RunMode.RUN_USING_ENCODERS
                    );
        }

    } // run_using_left_drive_encoder
    public void run_without_left_arm_encoder ()

    {
        if (v_motor_left_arm != null)
        {
            if (v_motor_left_arm.getMode () ==
                    DcMotorController.RunMode.RESET_ENCODERS)
            {
                v_motor_left_arm.setMode
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
    public int magnitude_turned_degrees(int start_heading) {
        if (!a_gyro_ready()) {
            return 0;   // don't know amount turned
        }
        final int current_heading = a_current_heading();

        final int result = Math.abs(signed_turn_in_degrees(start_heading));
        return result;
    }

    /**
     * You can save the initial heading using start_heading = a_current_heading();
     *
     * Use this function if you want to check if you are turning clockwise or counter-clockwise.
     * Otherwise, use magnitude_turned_degrees if you are simply trying to turn a certain angle
     * and are confident which way you are turning.
     *
     * @param start_heading
     * @return for up to 179 degrees of turn, reports positive (clockwise) or negative (counter-clockwise) turn
     */
    public int signed_turn_in_degrees(int start_heading) {
        if (!a_gyro_ready()) {
            return 0;   // don't know amount turned
        }
        final int current_heading = a_current_heading();

        final int clockwise_no_wrap = current_heading - start_heading;
        if (clockwise_no_wrap >= 0 && clockwise_no_wrap <= 180) {
            return clockwise_no_wrap;
        }
        final int clockwise_with_wrap = (current_heading+360)-start_heading;
        if (clockwise_with_wrap <= 180) {
            return clockwise_with_wrap;
        }
        final int counterclockwise_no_wrap = current_heading - start_heading;
        if (counterclockwise_no_wrap <= 0 &&
                counterclockwise_no_wrap >= -180) {
            return counterclockwise_no_wrap;
        }
        final int counterclockwise_with_wrap = (current_heading-360) - start_heading;
        return counterclockwise_with_wrap;
    }

    // returns true if gyro reports a turn AT LEAST AS FAR as end_heading
    // from start_heading in the given direction
    public boolean turned_from_to_heading(int start_heading, int end_heading, boolean turning_clockwise) {
        final int MAX_TURN = 180;
        final int clip_start_heading = clip_at_360(start_heading);
        final int clip_end_heading = clip_at_360(end_heading);

        if (!a_gyro_ready()) {
            return false;   // don't know if reached heading
        }
        final int current_heading = a_current_heading();

        if (turning_clockwise) {
            if (clip_end_heading > clip_start_heading) {
                return current_heading >= clip_end_heading;
            }
            else {
                // turn must wrap past zero and then reach end_heading (but not continue on to a 360!)
                return current_heading >= clip_end_heading &&
                        current_heading <= (clip_start_heading+clip_end_heading)/2;
            }
        }
        else {  // counter-clockwise
            if (clip_end_heading < clip_start_heading) {
                return current_heading <= clip_end_heading;
            }
            else {
                // turn must wrap past zero and then reach end_heading (but not continue on to a 360!)
                return current_heading <= clip_end_heading &&
                        current_heading >= (clip_start_heading+clip_end_heading)/2;
            }
        }
    }

    /**
     *
     * @return true if gyro heading can be read
     */
    public boolean a_gyro_ready() {
        boolean result = false;
        if (sensorGyro != null && !sensorGyro.isCalibrating()) {
            result = true;
        }
        return result;
    }

    /**
     *
     * @return zero if calibrating or no gyro, else current heading
     */
    public int a_current_heading() {
        int result = 0;
        if (sensorGyro == null) {
            // no action
        }
        else if (sensorGyro.isCalibrating()) {
            // no action
        }
        else {
            result = sensorGyro.getHeading();
        }
        return result;
    }

    /**
     *
     * @param heading
     * @return heading circled into [0,359]
     */
    public int clip_at_360(int heading) {
        int result = heading;
        while (result >= 360) {
            result -= 360;
        }
        while (result <= 0) {
            result += 360;
        }
        return result;
    }
}
