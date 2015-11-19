package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotManual
//

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Provide a basic manual operational mode that uses the left and right
 * drive motors, left arm motor, servo motors and gamepad input from only one
 * gamepad for the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-09-05-20-12
 */
public class PushBotManual1 extends PushBotTelemetry

{
    final double HAND_MOVEMENT_SIZE = 0.015;
    final double MY_MIN_HAND_POSITION = 0.23;
    final float ARM_MOVE_SPEED = 0.5f; // 0.175f;
    private DcMotor v_motor_right_arm;
    private DcMotor my_v_motor_left_arm;

    final int LEFT_ARM_BEGINNING = 10000;
    private int left_arm_encoder_old=LEFT_ARM_BEGINNING;
    final double HOLD_UP_ARM = 0.08;
    final double HOLD_HOLD_ARM = 0.03;
    final int HOLD_RANGE_ARM = -10;
    final double STICK_DEAD = 0.1;

    //Arm safety plan
    double save_leftarm_encoder=0;
    double save_armtime=0;
    boolean arm_stalled_up = false;
    boolean arm_stalled_down = false;
    final double WAIT_TIME_ARM=0.5;
    final double LEFT_ARM_THRESHHOLD=30;

    GyroSensor sensorGyro = null;
    int start_heading = 10000;
    TouchSensor v_sensor_touch = null;
    ColorSensor sensorRGB = null;

    //--------------------------------------------------------------------------
    //
    // PushBotManual1
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public PushBotManual1 ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotManual1
    void m_right_arm_power (double p_level)
    {
        if (v_motor_right_arm != null)
        {
            v_motor_right_arm.setPower (p_level);
        }

    } // m_right_arm_power
    double a_right_arm_power ()
    {
        double l_return = 0.0;

        if (v_motor_right_arm != null)
        {
            l_return = v_motor_right_arm.getPower ();
        }

        return l_return;

    } // a_right_arm_power
    @Override
    public void init() {
        super.init();
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
            my_v_motor_left_arm = hardwareMap.dcMotor.get ("left_arm");
    }
        catch (Exception p_exeception)
        {
            m_warning_message ("left_arm");
            DbgLog.msg(p_exeception.getLocalizedMessage());

            my_v_motor_left_arm = null;
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
        try
        {
            sensorRGB = hardwareMap.colorSensor.get("mr");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("mr (color sensor)");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            sensorRGB = null;
        }

    }

    public void start() {
        my_reset_left_arm_encoder();
        run_without_drive_encoders();
    }
//--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during
     * manual-operation.  The state machine uses gamepad input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {
        double currentTime = getRuntime();

        //----------------------------------------------------------------------
        //
        // DC Motors
        //
        // Obtain the current values of the joystick controllers.
        //
        // Note that x and y equal -1 when the joystick is pushed all of the way
        // forward (i.e. away from the human holder's body).
        //
        // The clip method guarantees the value never exceeds the range +-1.
        //
        // The DC motors are scaled to make it easier to control them at slower
        // speeds.
        //
        // The setPower methods write the motor power values to the DcMotor
        // class, but the power levels aren't applied until this method ends.
        //

        //
        // Manage the drive wheel motors.
        //
        float l_gp1_left_stick_y = -gamepad1.left_stick_y;
        float l_left_drive_power
                = (float) scale_motor_power(l_gp1_left_stick_y);

        float l_gp1_right_stick_y = -gamepad1.right_stick_y;
        float l_right_drive_power
                = (float) scale_motor_power(l_gp1_right_stick_y);

        set_drive_power(l_left_drive_power, l_right_drive_power);

        //
        // Manage the arm motor.  The right trigger makes the arm move from the
        // front of the robot to the back (i.e. up).  The left trigger makes the
        // arm move from the back to the front (i.e. down).
        //
        run_using_left_arm_encoder();
        run_without_right_arm_encoder();

        if (left_arm_encoder_old == LEFT_ARM_BEGINNING) {
            left_arm_encoder_old = my_a_left_arm_encoder_count();
        }
        final boolean deadstick = Math.abs(gamepad2.left_stick_y) <= STICK_DEAD;
        if (!deadstick) {
            left_arm_encoder_old = my_a_left_arm_encoder_count();

            float l_left_arm_power
                    = Range.clip(
                    (float) scale_motor_power(gamepad2.left_stick_y),
                    -ARM_MOVE_SPEED,
                    ARM_MOVE_SPEED);
            if ((l_left_arm_power > 0 && !arm_stalled_up) ||
                (l_left_arm_power < 0.0 && !arm_stalled_down)) {
                m_left_arm_power(l_left_arm_power);
                arm_stalled_down = false;
                arm_stalled_up = false;
            }
        }
        else {
            final int currentposition = my_a_left_arm_encoder_count();
            int arm_change = currentposition - left_arm_encoder_old;
            if (arm_change <= HOLD_RANGE_ARM) {
                if (!arm_stalled_up) {
                    m_left_arm_power(HOLD_UP_ARM);
                }
            }
            else if (arm_change >= 0) {
                m_left_arm_power(0.0);
            }
            else {
                // keeping power on might burn out motor
                m_left_arm_power(0.0);  // was HOLD_HOLD_ARM
            }
        }

        if (deadstick || a_left_arm_power() == 0.0) {
            save_leftarm_encoder = my_a_left_arm_encoder_count();
            save_armtime = currentTime;
        }
        else {
            if (Math.abs(currentTime-save_armtime) >= WAIT_TIME_ARM) {
                if (Math.abs(my_a_left_arm_encoder_count()-save_leftarm_encoder)<LEFT_ARM_THRESHHOLD){
                    if (a_left_arm_power() > 0.0) {
                        arm_stalled_up = true;      // don't power arm up anymore until stop first
                    }
                    else {
                        arm_stalled_down = true;    // don't power arm down anymore until stop first
                    }
                    m_left_arm_power(0.0);
                }
                else {
                    save_leftarm_encoder=my_a_left_arm_encoder_count();
                    save_armtime=currentTime;
                }
            }
        }
        telemetry.addData("19", "Left arm stall up " + arm_stalled_up + " / down " + arm_stalled_down
            );



        float l_right_arm_power
                = Range.clip(
                (float) scale_motor_power(gamepad2.right_stick_y),
                -ARM_MOVE_SPEED,
                ARM_MOVE_SPEED);
        m_right_arm_power(l_right_arm_power);

        //----------------------------------------------------------------------
        //
        // Servo Motors
        //
        // Obtain the current values of the gamepad 'x' and 'b' buttons.
        //
        // Note that x and b buttons have boolean values of true and false.
        //
        // The clip method guarantees the value never exceeds the allowable
        // range of [0,1].
        //
        // The setPosition methods write the motor power values to the Servo
        // class, but the positions aren't applied until this method ends.
        //
        if (gamepad2.x) {

           final double target_position = a_hand_position()-HAND_MOVEMENT_SIZE;
           final double l_position = Range.clip
                    (target_position
                            , MY_MIN_HAND_POSITION
                            , Servo.MAX_POSITION

                    );

             m_hand_position(l_position);

        } else if (gamepad2.b) {
           final double target_position = a_hand_position()+HAND_MOVEMENT_SIZE;
          final double l_position = Range.clip
                    (target_position
                            , MY_MIN_HAND_POSITION
                            , Servo.MAX_POSITION);
            m_hand_position(l_position);
        }
        // if the right arm is touching the top then stop the motor
        if(v_sensor_touch.isPressed() ) {
            if (a_right_arm_power() > 0) {


                m_right_arm_power(0.0);
            }
        }

        // check gyro
        if (start_heading > 360 && a_gyro_ready()) {
            start_heading = a_current_heading();
        }
        // if the A and B buttons are pressed, reset Z heading.
        if(gamepad1.a && gamepad1.b)  {
            // reset heading.
            sensorGyro.resetZAxisIntegrator();
        }
        if (sensorGyro == null) {
            telemetry.addData
                    ("20", "Gyro not found"
                    );
        }
        else if (sensorGyro.isCalibrating()) {
            telemetry.addData
                    ("20", "Gyro calibrating"
                    );

        }
        else {
            // get the heading info.
            // the Modern Robotics' gyro sensor keeps
            // track of the current heading for the Z axis only.
            int heading = sensorGyro.getHeading();

            telemetry.addData("20", "Gyro head " + String.format("%03d", heading));
        }
        telemetry.addData("21", "Gyro turn (signed): " + signed_turn_in_degrees(start_heading) );
        telemetry.addData("22", "Gyro turn amount: " + magnitude_turned_degrees(start_heading) );

        telemetry.addData("27", "Touch sensor " + (v_sensor_touch.isPressed() ? "pressed" : "not pressed") );
        // read color sensor
        final int clear = sensorRGB.alpha();
        final int red = sensorRGB.red();
        final int green = sensorRGB.green();
        final int blue = sensorRGB.blue();
        final int COLOR_SHIFT = 4;
        final boolean mostly_red = (red > green+COLOR_SHIFT && red > blue+COLOR_SHIFT);
        final boolean mostly_green = (green > red+COLOR_SHIFT && green > blue+COLOR_SHIFT);
        final boolean mostly_blue = (blue > red+COLOR_SHIFT && blue > green+COLOR_SHIFT);
        telemetry.addData("28", "Clear / R / G / B" +
                        String.format(" %02d", clear) +
                        String.format(" %02d", red) +
                        String.format(" %02d", green) +
                        String.format(" %02d", blue)
        );
        telemetry.addData("29", "Color " + (mostly_red ? "red" : (mostly_green ? "green" : (mostly_blue ? "blue" : "??")))
            );

            //
        // Send telemetry data to the driver station.
        //
            update_telemetry(); // Update common telemetry
            update_gamepad_telemetry();
        telemetry.addData
                ("12"
                        , "Left Arm encoder: " + (my_has_left_arm_encoder_reset() ? "R" : "") + my_a_left_arm_encoder_count()
                );
        telemetry.addData("16", "Right Arm Power" + l_right_arm_power );
        telemetry.addData("25", "Time " + currentTime
        );


    }  //loop
    int my_a_left_arm_encoder_count()
    {
        int l_return = 0;

        if (my_v_motor_left_arm != null)
        {
            l_return = my_v_motor_left_arm.getCurrentPosition ();
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
    boolean my_has_left_arm_encoder_reset()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Has the left encoder reached zero?
        //
        if (my_a_left_arm_encoder_count() == 0)
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
    public void my_reset_left_arm_encoder()

    {
        if (my_v_motor_left_arm != null)
        {
            my_v_motor_left_arm.setMode
                    (DcMotorController.RunMode.RESET_ENCODERS
                    );
        }

    } // reset_left_drive_encoder

    public void my_run_without_left_arm_encoder() {
        if (my_v_motor_left_arm != null)
        {
            if (my_v_motor_left_arm.getMode() ==
                    DcMotorController.RunMode.RESET_ENCODERS)
            {
                my_v_motor_left_arm.setMode
                        (DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }
    }

    public void run_using_left_arm_encoder ()

    {
        if (my_v_motor_left_arm != null)
        {
            my_v_motor_left_arm.setMode
                    (DcMotorController.RunMode.RUN_USING_ENCODERS
                    );
        }

    } // run_using_right_drive_encoder

    public void run_without_right_arm_encoder() {
        if (v_motor_right_arm != null)
        {
            if (v_motor_right_arm.getMode() ==
                    DcMotorController.RunMode.RESET_ENCODERS)
            {
                v_motor_right_arm.setMode
                        (DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }
    }

    /**
     * You can save the initial heading using start_heading = a_current_heading();
     *
     * Use this function if you want to check if you are simply trying to turn a certain angle
     * and are confident which way you are turning.
     *
     * For example: if (magnitude_turned_degrees(start_heading) >= 45) {...}
     *
     * If you need to know whether you have turned clockwise or counterclockwise,
     * use signed_turn_in_degrees which reports positive for clockwise.
     *
     * @param start_heading
     * @return for up to 179 degrees of turn, reports (always positive) magnitude of degrees turned
     */
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
} // PushBotManual 1
