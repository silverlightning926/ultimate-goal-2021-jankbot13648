package org.firstinspires.ftc.teamcode.Vision;

//----------------------------------------------------------------------------
//
//  $Workfile: RingLocation.java$
//
//  $Revision: X$
//
//  Project:    FTC 2021
//
//                            Copyright (c) 2021
//                              Stealth Robotics
//                            All Rights Reserved
//
//  Modification History:
//  $Log:
//  $
//
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
//Class Declarations
//----------------------------------------------------------------------------
//
//Class Name: RingLocation
//
//Purpose:
//  This is the class that tells you how many rings and where they are.
//
//----------------------------------------------------------------------------
public class RingLocation {
    //----------------------------------------------------------------------------
    //  Class Constants
    //----------------------------------------------------------------------------
    public static final int TARGET_ZERO_RINGS = 0;
    public static final int TARGET_ONE_RING   = 1;
    public static final int TARGET_FOUR_RINGS = 2;

    //----------------------------------------------------------------------------
    //  Class Atributes
    //----------------------------------------------------------------------------
    public double mTop = 0.0;
    public double mBottom = 0.0;
    public double mLeft = 0.0;
    public double mRight = 0.0;
    public int    mDetected = TARGET_ZERO_RINGS;
}
