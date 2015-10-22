package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotManual
//

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Provide a basic manual operational mode that uses the left and right
 * drive motors, left arm motor, servo motors and gamepad input from only one
 * gamepad for the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-09-05-20-12
 */
public class PushBotManual2 extends PushBotTelemetry

{
    final double HAND_START_POSITION = 0.725;  // open symmetric
    final double HAND_INIT_LEFT_POSITION = 0.0;  // full twist to stay inside 18 inches
    final double HAND_INIT_RIGHT_POSITION = 0.5; // avoid hitting left side
    final double HAND_STEP_SIZE = 0.015;
    final double HAND_MANUAL_LIMIT = 0.725; // avoid breaking off-shover

    final float ARM_DOWN_SLOWDOWN = 0.5f; // scale down arm using lower power
    final float ARM_UP_SLOWDOWN = 0.5f; // scale down arm using lower power

    Servo v_servo_myleft_hand;
    Servo v_servo_myright_hand;

    //--------------------------------------------------------------------------
    //
    // PushBotManual1
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public PushBotManual2 ()

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

    @Override public void init() {
        super.init();   // we must call the super class to initialize its variables

        try
        {
            v_servo_myleft_hand = hardwareMap.servo.get ("left_hand");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("left_hand");
            DbgLog.msg(p_exeception.getLocalizedMessage());

            v_servo_myleft_hand = null;
        }

        try
        {
            v_servo_myright_hand = hardwareMap.servo.get ("right_hand");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("right_hand");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_myright_hand = null;
        }

        m_myhand_position(HAND_INIT_LEFT_POSITION, HAND_INIT_RIGHT_POSITION);
        update_telemetry(); // Update common telemetry

    }

    @Override public void start() {
        super.start();
        m_hand_position (HAND_START_POSITION);

    }

    @Override public void stop() {
        super.stop();
        m_hand_position (HAND_START_POSITION);

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
            = (float)scale_motor_power (l_gp1_left_stick_y);

        float l_gp1_right_stick_y = -gamepad1.right_stick_y;
        float l_right_drive_power
            = (float)scale_motor_power (l_gp1_right_stick_y);

        set_drive_power (l_left_drive_power, l_right_drive_power);

        //
        // Manage the arm motor.  The right trigger makes the arm move from the
        // front of the robot to the back (i.e. up).  The left trigger makes the
        // arm move from the back to the front (i.e. down).
        //
        // scaling down motor power to slow the arm
        float l_left_arm_power = 0.0f;
        final float lefty = -gamepad2.left_stick_y; // gamepad direction is reverse of motor direction
        if (lefty >= 0) {
            l_left_arm_power = (float)scale_motor_power (ARM_UP_SLOWDOWN * lefty);  // medium up power
        }
        else {
            l_left_arm_power = (float)scale_motor_power (ARM_DOWN_SLOWDOWN * lefty); // slow down power
        }
        m_left_arm_power (l_left_arm_power);

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
        if (gamepad2.x)
        {
            final double new_position = a_hand_position () + HAND_STEP_SIZE;

            m_hand_position(new_position);
        }
        else if (gamepad2.b)
        {
            final double new_position = a_hand_position () - HAND_STEP_SIZE;

            m_hand_position(new_position);
        }

        //
        // Send telemetry data to the driver station.
        //
        update_telemetry (); // Update common telemetry
        update_gamepad_telemetry();
        /*
        telemetry.addData
                ("12"
                        , "Left Arm1: " + l_left_arm_power
            );
        telemetry.addData
                ( "13"
                        , "Hand: " + a_hand_position()
                );
        */
    } // loop

    //--------------------------------------------------------------------------
    //
    // m_hand_position
    //
    /**
     * Mutate the hand position to be not symmetric.
     */
    void m_myhand_position (double left_position, double right_position)
    {
        //
        // Ensure the specific value is legal.
        //
        double l_position = Range.clip
                ( left_position
                        , Servo.MIN_POSITION
                        , Servo.MAX_POSITION
                );
        double r_position = Range.clip
                (right_position
                        , Servo.MIN_POSITION
                        , Servo.MAX_POSITION
                );

        //
        // Set the value.  The right hand value must be opposite of the left
        // value.
        //
        if (v_servo_myleft_hand != null)
        {
            v_servo_myleft_hand.setPosition (l_position);
        }
        if (v_servo_myright_hand != null)
        {
            v_servo_myright_hand.setPosition (r_position);
        }

        telemetry.addData
                ("12"
                        , "Hand twist: " + l_position + ", " + r_position
                );
    } // m_myhand_position

} // PushBotManual1
