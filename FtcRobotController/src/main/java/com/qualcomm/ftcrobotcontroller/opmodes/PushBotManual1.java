package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotManual
//

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
        float l_left_arm_power
                = Range.clip(
                (float) scale_motor_power(gamepad2.left_stick_y),
                -ARM_MOVE_SPEED,
                ARM_MOVE_SPEED);
        m_left_arm_power(l_left_arm_power);

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
                        , "Left Arm1b: " + l_left_arm_power
                );



    }  //loop

} // PushBotManual 1
