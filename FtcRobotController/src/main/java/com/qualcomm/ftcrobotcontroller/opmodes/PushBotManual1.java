package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotManual
//

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
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
public class PushBotManual1 extends PushBotTelemetry

{
    final double HAND_MOVEMENT_SIZE = 0.015;
    final double MY_MIN_HAND_POSITION = 0.23;
    final float ARM_MOVE_SPEED = 0.175f;
    private DcMotor v_motor_right_arm;
    private DcMotor my_v_motor_left_arm;

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

    } // m_left_arm_power
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

        float l_left_arm_power
                = Range.clip(
                (float) scale_motor_power(gamepad2.left_stick_y),
                -ARM_MOVE_SPEED,
                ARM_MOVE_SPEED);
        m_left_arm_power(l_left_arm_power);



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
            my_v_motor_left_arm.setChannelMode
                    ( DcMotorController.RunMode.RESET_ENCODERS
                    );
        }

    } // reset_left_drive_encoder

    public void my_run_without_left_arm_encoder() {
        if (my_v_motor_left_arm != null)
        {
            if (my_v_motor_left_arm.getChannelMode () ==
                    DcMotorController.RunMode.RESET_ENCODERS)
            {
                my_v_motor_left_arm.setChannelMode
                        ( DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }
    }

    public void run_using_left_arm_encoder ()

    {
        if (my_v_motor_left_arm != null)
        {
            my_v_motor_left_arm.setChannelMode
                    ( DcMotorController.RunMode.RUN_USING_ENCODERS
                    );
        }

    } // run_using_right_drive_encoder

    public void run_without_right_arm_encoder() {
        if (v_motor_right_arm != null)
        {
            if (v_motor_right_arm.getChannelMode () ==
                    DcMotorController.RunMode.RESET_ENCODERS)
            {
                v_motor_right_arm.setChannelMode
                        ( DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }
    }
} // PushBotManual 1
