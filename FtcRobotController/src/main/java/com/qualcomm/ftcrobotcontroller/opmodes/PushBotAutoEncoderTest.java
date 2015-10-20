package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Lego on 10/20/2015.
 */
public class PushBotAutoEncoderTest extends PushBotTelemetrySensors {
    //--------------------------------------------------------------------------
    //
    // PushBotAutoEncoderTest
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public PushBotAutoEncoderTest ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotAutoSensors

    //--------------------------------------------------------------------------
    //
    // start
    //
    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */
    @Override public void start ()

    {
        //
        // Call the PushBotHardware (super/base class) start method.
        //
        super.start ();

        //
        // Reset the motor encoders on the drive wheels.
        //
        reset_drive_encoders ();

        resetStartTime();

    } // start

    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during auto-operation.
     * The state machine uses a class member and sensor input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {
        final double currentTime = getRuntime();
        //
        // Update the state machines
        //
        switch (v_state)
        {
            //
            // State 0.
            //
            case 0:
                //
                // Wait for the encoders to reset.  This might take multiple cycles.
                //
                if (have_drive_encoders_reset ())
                {
                    //
                    // Begin the next state.  Drive forward.
                    //
                    drive_using_encoders (-1.0f, 1.0f, -2880, 2880);

                    //
                    // Transition to the next state.
                    //
                    v_state++;
                }

                break;
            //
            // State 1.
            //
            case 1:
                //
                // Drive forward at full power until the encoders trip.
                //
                // Forward motion is achieved when the first and second parameters
                // are the same and positive.
                //
                // The magnitude of the first and second parameters determine the
                // speed (0.0 is stopped and 1.0 is full).
                //
                // The third and fourth parameters determine how long the wheels
                // turn.
                //
                // When the encoder values have been reached the call resets the
                // encoders, halts the motors, and returns true.
                //
                if (drive_using_encoders (1.0f, 1.0f, 2880, 2880))
                {
                    //
                    // The drive wheels have reached the specified encoder values,
                    // so transition to the next state when this method is called
                    // again.
                    //
                    v_state++;
                }
                break;
            //
            // State 2.
            //
            case 2:
                //
                // Wait for the encoders to reset.  This might take multiple cycles.
                //
                if (have_drive_encoders_reset ())
                {
                    //
                    // Begin the next state.  Turn left.
                    //
                    // There is no conditional (if statement) here, because the
                    // encoders can't be read until the next cycle
                    // (drive_using_encoders makes the run_using_encoders call,
                    // which won't be processed until this method exits).
                    //
                    drive_using_encoders (-1.0f, 1.0f, -2880, 2880);

                    //
                    // Start the arm state machine.
                    //
                    v_arm_state = 1;

                    //
                    // The drive wheels have reached the specified encoder values,
                    // so transition to the next state when this method is called
                    // again.
                    //
                    v_state++;
                }
                break;
            //
            // State 3.
            //
            case 3:
                //
                // Continue turning left, if necessary.
                //
                if (drive_using_encoders (-1.0f, 1.0f, -2880, 2880))
                {
                    v_state++;
                }
                break;
            //
            // State 4.
            //
            case 4:
                //
                // As soon as the drive encoders reset, begin the next state.
                //
                if (have_drive_encoders_reset ())
                {
                    //
                    // Begin the next state.
                    //
                    // There is no conditional (if statement) here, because the
                    // motor power won't be applied until this method exits.
                    //
                    run_without_drive_encoders ();

                    //
                    // Transition to the next state.
                    //
                    v_state++;
                }
                break;
            //
            // State 5.
            //
            case 5:
                hand_motion_time_start = currentTime;
                hand_motion_time_stop = hand_motion_time_start + 3.0;   // amount time hand is open
                m_hand_position(Servo.MAX_POSITION);

                v_state++;
                break;
            //
            // State 6.
            //
            case 6:
                if (currentTime >= hand_motion_time_stop) {
                    m_hand_position(Servo.MIN_POSITION);

                    v_state++;
                }

                break;
            //
            // Perform no action - stay in this case until the OpMode is stopped.
            // This method will still be called regardless of the state machine.
            //
            default:
                //
                // The autonomous actions have been accomplished (i.e. the state has
                // transitioned into its final state.
                //
                break;
        }

        //
        // Update the arm state machine.
        //
        update_arm_state ();

        //
        // Send telemetry data to the driver station.
        //
        update_telemetry (); // Update common telemetry
        telemetry.addData("11", "Drive State: " + v_state);
        telemetry.addData("12", "Arm State: " + v_arm_state);

    } // loop

    //--------------------------------------------------------------------------
    //
    // update_arm_state
    //
    /**
     * Implement a state machine that controls the arm during auto-operation.
     */
    public void update_arm_state ()

    {
        final double currentTime = getRuntime();
        //
        // Update the arm state machine.
        //
        switch (v_arm_state)
        {
            //
            // State 0.
            //
            case 0:
                //
                // Wait until a command is given (i.e. v_state is set to 1).
                //
                break;
            //
            // State 1.
            //
            case 1:
                //
                if (arm_motion_time_start == 0.0) {
                    arm_motion_time_start = currentTime;
                    arm_motion_time_stop = arm_motion_time_start + 0.5;
                    m_left_arm_power(-0.15);  // start arm moving down
                }
                else if (currentTime >= arm_motion_time_stop) { // first time window elapsed, reverse arm
                    arm_motion_time_start = currentTime;
                    arm_motion_time_stop = arm_motion_time_start + 0.5;
                    m_left_arm_power(0.15); // reverse arm to up

                    v_arm_state++;
                }
                break;
            case 2:
                if (currentTime >= arm_motion_time_stop) {  // second time window elapsed, stop arm
                    m_left_arm_power(0.0);  // stop arm
                    //
                    // Transition to the stop state.
                    //
                    v_arm_state++;
                }
                break;
            //
            // Perform no action - stay in this case until the OpMode is
            // stopped.  This method will still be called regardless of the
            // state machine.
            //
            default:
                //
                // The autonomous actions have been accomplished (i.e. the state
                // has transitioned into its final state.
                //
                break;
        }

    } // update_arm_state

    //--------------------------------------------------------------------------
    //
    // v_state
    //
    /**
     * This class member remembers which state is currently active.  When the
     * start method is called, the state will be initialize (0).  During the
     * first iteration of the loop method, the state will change from initialize
     * to state_1.  When state_1 actions are complete, the state will change to
     * state_2.  This implements a state machine for the loop method.
     */
    private int v_state = 0;
    private double hand_motion_time_start = 0.0;
    private double hand_motion_time_stop = 0.0;

    //--------------------------------------------------------------------------
    //
    // v_arm_state
    //
    /**
     * This class member remembers which state is currently active.  When the
     * start method is called, the state will be initialize (0).  During the
     * first iteration of the loop method, the state will change from initialize
     * to state_1.  When state_1 actions are complete, the state will change to
     * state_2.  This implements a state machine for the loop method.
     */
    private int v_arm_state = 0;
    private double arm_motion_time_start = 0.0;
    private double arm_motion_time_stop = 0.0;
}
