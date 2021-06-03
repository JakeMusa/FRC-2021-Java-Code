/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    
    //motor ports
    public static final int RIGHT_ALPHA_DRIVE = 1;
    public static final int LEFT_ALPHA_DRIVE = 2;
    public static final int RIGHT_BETA_DRIVE = 3; 
    public static final int LEFT_BETA_DRIVE = 4; 
    public static final int RIGHT_SHOOT = 5;
    public static final int LEFT_SHOOT = 6;
    public static final int INTAKE_MOTOR = 1;
    public static final int INDEX_MOTOR =2;
    public static final int PLATE_MOTOR = 0;
    //ramp constants 
    public static final double CLOSED_LOOP_RAMP_RATE = 0.75; 
    public static final double OPEN_LOOP_RAMP_RATE = 0.75; 
    public static final double SHOOT_RAMP = 1.75;
    //controller constants
    public static final int XBOX_LEFT_Y_AXIS = 1;
    public static final int XBOX_LEFT_X_AXIS = 0;
    public static final int XBOX_RIGHT_Y_AXIS = 5;
    public static final int XBOX_RIGHT_TRIGGER = 3;
    public static final int XBOX_LEFT_TRIGGER = 2;
    public static final int JOYSTICK_NUMBER = 1;
    //drive constants
    public static final double DRIVESPEED = 0.9;
    public static final double INDEX_SPEED = 0.25;
    public static final double PLATE_SPEED = 0.65;
    //auto constants
    public static final double AUTO_SPEED = 0.4;
    public static final double SHOOT_TIME = 5.0;
    public static final double TIMED_DRIVE = 2.0;
    //shooter constants 
    public static final double SHOOT_SPEED = -0.70;
    //intake/index constants
    public static final double INTAKE_SPEED = -1;
    //PID constants
    public static final double kA = 0.491;
    public static final double kV = 2.1;
    public static final double kS = 0.509;
    public static final double kP = 2.71;

    
	
	



}

