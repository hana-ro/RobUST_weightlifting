using UnityEngine;
using Unity.Mathematics;
using System;
using System.Collections.Generic;

/// <summary>
/// Complete robot description for the RobUST cable-driven parallel robot.
/// Contains all constant parameters needed for kinematics, dynamics, and control.
/// Allocated once at startup - zero runtime allocations.
/// </summary>
public sealed class RobUSTDescription
{
    public readonly int NumCables;
    
    /// <summary>Pulley positions in robot frame [m] (double3 for SIMD)</summary>
    public readonly double3[] FramePulleyPositions;
    
    /// <summary>Cable attachment points on barbell, in end-effector frame [m]</summary>
    public readonly double3[] LocalAttachmentPoints;
    
    /// <summary>Barbell center in end-effector frame [m]</summary>
    public readonly double3 BarbellCenter_EE_Frame;
    
    
    // ============ User Parameters ============
    /// <summary>User body mass [kg]</summary>
    public readonly double UserMass;
    
    /// <summary>Chest anterior-posterior distance [m]</summary>
    public readonly double ChestAPDistance;
    
    /// <summary>Chest medial-lateral distance [m]</summary>
    public readonly double ChestMLDistance;

    // ============ Force Plate Geometry ============
    public readonly double3 FP_BackLeft;
    public readonly double3 FP_BackRight;
    public readonly double3 FP_FrontLeft;
    public readonly double3 FP_FrontRight;

    // Full 8-cable hardware definition (Static Database from vive tracker measurement)
    private static readonly double3[] AllPulleyPositions = new double3[]
    {
        new double3(-0.8000, 1.650, 0.9875),   // 0: Front-Right Top (Motor 10)
        new double3(-0.7900, 0.0022, 0.9540),   // 1: Front-Left Top (Motor 5)
        new double3(0.9580, 0.0335, 0.9828),    // 2: Back-Left Top (Motor 4)
        new double3(0.9650, 1.6420, 1.0000),    // 3: Back-Right Top (Motor 11)
        new double3(-0.7850, 1.6750, -0.2920),  // 4: Front-Right Bottom (Motor 8)
        new double3(-0.7800, 0.0330, -0.3340),  // 5: Front-Left Bottom (Motor 7)
        new double3(0.9850, 0.0520, -0.5165),   // 6: Back-Left Bottom (Motor 2)
        new double3(0.9800, 1.6890, -0.4994)    // 7: Back-Right Bottom (Motor 13)
    };

    // Mapping from solver index to motor driver index for the full set
    // Corresponds to the order in AllPulleyPositions
    public static readonly int[] FullMotorMapping = new int[] { 9, 4, 3, 10, 7, 6, 1, 12 };

    // Source-index grouping by end-effector side (x in local EE frame)
    private static readonly int[] LeftSideSourceIndices = new int[] { 0, 3, 4, 7 };
    private static readonly int[] RightSideSourceIndices = new int[] { 1, 2, 5, 6 };

    public readonly int[] SolverToMotorMap;
    public readonly int[] LeftCableIndices;
    public readonly int[] RightCableIndices;

    private RobUSTDescription(int numCables, double chestAP, double chestML, 
                              double userMass)
    {
        NumCables = numCables;
        ChestAPDistance = chestAP;
        ChestMLDistance = chestML;
        UserMass = userMass;

        // Force Plate Configuration ----
        double3 back_left_correction   = new double3( 0.03, -0.03, -0.01);
        double3 back_right_correction  = new double3( 0.03,  0.03, -0.01);
        double3 front_left_correction  = new double3(-0.03, -0.03, -0.01);
        double3 front_right_correction = new double3(-0.03,  0.03, -0.01);
        // vive tracker jig readings:
        FP_BackLeft   = new double3( 0.5385, 0.5793, -0.9520) + back_left_correction;
        FP_BackRight  = new double3( 0.5234, 1.1350, -0.9463) + back_right_correction;
        FP_FrontLeft  = new double3(-0.3202, 0.5510, -0.9525) + front_left_correction;
        FP_FrontRight = new double3(-0.3341, 1.1077, -0.9458) + front_right_correction;        
        
        FramePulleyPositions = new double3[numCables];
        LocalAttachmentPoints = new double3[numCables];
        SolverToMotorMap = new int[numCables];

        // Determine active subset based on requested numCables
        int[] activeIndices = BuildActiveCableIndices(numCables);

        // Temporary simplification: one estimated attachment point per tracker side.
        const double leftAttachmentOffsetY = 0.085;
        const double rightAttachmentOffsetY = -0.085;
        
        // Populate arrays based on active indices
        for (int i = 0; i < numCables; i++)
        {
            int srcIdx = activeIndices[i];
            
            FramePulleyPositions[i] = AllPulleyPositions[srcIdx];
            SolverToMotorMap[i] = FullMotorMapping[srcIdx];

            if (IsLeftSideSourceIndex(srcIdx))
            {
                LocalAttachmentPoints[i] = new double3(0.0, leftAttachmentOffsetY, 0.0);
            }
            else if (IsRightSideSourceIndex(srcIdx))
            {
                LocalAttachmentPoints[i] = new double3(0.0, rightAttachmentOffsetY, 0.0);
            }
            else
            {
                throw new ArgumentException($"Unknown source cable index: {srcIdx}");
            }
        }

        (LeftCableIndices, RightCableIndices) = BuildTrackerSideCableIndices(activeIndices);

        //BarbellCenter_EE_Frame = new double3(0, -chestAP / 2.0, 0);
    }

    private static int[] BuildActiveCableIndices(int numCables)
    {
        const int maxCables = 8;

        if (numCables < 2 || numCables > maxCables || (numCables % 2) != 0)
            throw new ArgumentException($"Unsupported cable count: {numCables}. valid options are even counts from 2 to {maxCables}.");

        int cablesPerSide = numCables / 2;
        if (cablesPerSide > LeftSideSourceIndices.Length || cablesPerSide > RightSideSourceIndices.Length)
            throw new ArgumentException($"Cable count {numCables} exceeds available side definitions.");

        bool[] selected = new bool[maxCables];
        for (int i = 0; i < cablesPerSide; i++)
        {
            selected[LeftSideSourceIndices[i]] = true;
            selected[RightSideSourceIndices[i]] = true;
        }

        int[] active = new int[numCables];
        int write = 0;
        for (int src = 0; src < maxCables; src++)
        {
            if (selected[src])
                active[write++] = src;
        }

        return active;
    }

    private static (int[] left, int[] right) BuildTrackerSideCableIndices(int[] activeIndices)
    {
        List<int> left = new List<int>(activeIndices.Length / 2 + 1);
        List<int> right = new List<int>(activeIndices.Length / 2 + 1);

        for (int i = 0; i < activeIndices.Length; i++)
        {
            int sourceIndex = activeIndices[i];
            if (IsLeftSideSourceIndex(sourceIndex))
                left.Add(i);
            else if (IsRightSideSourceIndex(sourceIndex))
                right.Add(i);
            else
                throw new ArgumentException($"Unknown source cable index: {sourceIndex}");
        }

        return (left.ToArray(), right.ToArray());
    }

    private static bool IsLeftSideSourceIndex(int sourceIndex)
    {
        for (int i = 0; i < LeftSideSourceIndices.Length; i++)
        {
            if (LeftSideSourceIndices[i] == sourceIndex)
                return true;
        }

        return false;
    }

    private static bool IsRightSideSourceIndex(int sourceIndex)
    {
        for (int i = 0; i < RightSideSourceIndices.Length; i++)
        {
            if (RightSideSourceIndices[i] == sourceIndex)
                return true;
        }

        return false;
    }

    /// <summary>
    /// Factory method to create RobUST description from belt/chest configuration.
    /// All allocations happen here at init - nothing at runtime.
    /// </summary>
    /// <param name="numCables">Number of cables in the system</param>
    /// <param name="chestAPDistance">Chest anterior-posterior distance [m]</param>
    /// <param name="chestMLDistance">Chest medial-lateral distance [m]</param>
    /// <param name="userMass">User body mass [kg]</param>
    /// <param name="shoulderWidth">User shoulder width [m]</param>
    /// <param name="userHeight">User trunk height hip-to-shoulder [m]</param>
    public static RobUSTDescription Create(int numCables, double chestAPDistance, double chestMLDistance,
                                           double userMass)
    {
        return new RobUSTDescription(numCables, chestAPDistance, chestMLDistance, userMass);
    }
}
