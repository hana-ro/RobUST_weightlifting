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
        //for old robust before expansion
        // new double3(-0.8000, 1.650, 0.9875),   // 0: Front-Right Top (Motor 10)
        // new double3(-0.7900, 0.0022, 0.9540),   // 1: Front-Left Top (Motor 5)
        // new double3(0.9580, 0.0335, 0.9828),    // 2: Back-Left Top (Motor 4)
        // new double3(0.9650, 1.6420, 1.0000),    // 3: Back-Right Top (Motor 11)
        // new double3(-0.7850, 1.6750, -0.2920),  // 4: Front-Right Bottom (Motor 8)
        // new double3(-0.7800, 0.0330, -0.3340),  // 5: Front-Left Bottom (Motor 7)
        // new double3(0.9850, 0.0520, -0.5165),   // 6: Back-Left Bottom (Motor 2)
        // new double3(0.9800, 1.6890, -0.4994)    // 7: Back-Right Bottom (Motor 13)

        //new robust after expansion; left side should have stayed the same  
        //new positive x: towards gait mat; positive y: right towards bathroom; positive z: up
        //new double3(-0.8000, 2.6416, 0.9875),   // 0: Front-Right Top (Motor 10)
        //new double3(-0.7900, 0.0022, 0.9540),   // 1: Front-Left Top (Motor 5)
        //new double3(0.9580, 0.0335, 0.9828),    // 2: Back-Left Top (Motor 4)
        //new double3(0.9650, 2.6416, 1.0000),    // 3: Back-Right Top (Motor 11)
        //new double3(-0.7850, 2.5019, -0.2920),  // 4: Front-Right Bottom (Motor 8)
        //new double3(-0.7800, 0.0330, -0.3340),  // 5: Front-Left Bottom (Motor 7)
        //new double3(0.9850, 0.0520, -0.5165),   // 6: Back-Left Bottom (Motor 2)
        //new double3(0.9800, 2.5019, -0.4994)    // 7: Back-Right Bottom (Motor 13)

        //position of vive frame tracker has changed
        //new positive x: towards gait mat; positive y: right towards bathroom; positive z: up
        // new double3(-0.583222337971983, 2.5508900250331, 0.227511387085326),   // 0: Front-Right Top (Motor 10)
        // new double3(-0.573533381858206, 0.0631838136479876, 0.173349140849092),   // 1: Front-Left Top (Motor 5)
        // new double3(1.18569761686698, 0.0590750002399072, 0.245374625420247),    // 2: Back-Left Top (Motor 4)
        // new double3(1.19861556094742, 2.55186185695798, 0.295413183904312),    // 3: Back-Right Top (Motor 11)
        // new double3(-0.554510179595532, 2.57998755330367, -1.257),  // 4: Front-Right Bottom (Motor 8)
        // new double3(-0.532261584267422, 0.0650720717224675, -1.257),  // 5: Front-Left Bottom (Motor 7)
        // new double3(1.2466629448881, 0.07627937095702, -1.25845576355938),   // 6: Back-Left Bottom (Motor 2)
        // new double3(1.25866757865328, 2.59656509679819, -1.21149695323785)    // 7: Back-Right Bottom (Motor 13)


        //new positive x: towards gait mat; positive y: right towards bathroom; positive z: up
        new double3(-0.576835048372789, 2.57648944314966, 0.252733189583757),   // 0: Front-Right Top (Motor 10)
        new double3(-0.588436088357761, 0.0273606629681548, 0.182261108739911),   // 1: Front-Left Top (Motor 5)
        new double3(1.18955071881626, 0.0229493275829191, 0.264120148165004),    // 2: Back-Left Top (Motor 4)
        new double3(1.19164684688485, 2.58195169592441, 0.33072052196806),   // 3: Back-Right Top (Motor 11)
        new double3(-0.527316212320867, 2.60276181483673, -1.10992095940637),  // 4: Front-Right Bottom (Motor 8)
        new double3(-0.535379256296376, 0.0557650522283454, -1.17363481318352),  // 5: Front-Left Bottom (Motor 7)
        new double3(1.25455282289238, 0.0542982128558426, -1.23426325420294),   // 6: Back-Left Bottom (Motor 2)
        new double3(1.26826084676152, 2.61087432020563, -1.18361414421777)   // 7: Back-Right Bottom (Motor 13)
    };

    // Mapping from solver index to motor driver index for the full set
    // Corresponds to the order in AllPulleyPositions
    // Pulley index 0 -> Motor 10
    public static readonly int[] FullMotorMapping = new int[] { 9, 4, 3, 10, 7, 6, 1, 12 };

    // Source-index grouping by end-effector side (x in local EE frame)
    private static readonly int[] LeftSideSourceIndices = new int[] { 0, 3, 4, 7 };
    // Motors 5, 4, 7, 2 are on the left side of the robot
    
    private static readonly int[] RightSideSourceIndices = new int[] { 1, 2, 5, 6 };
    // Motors 10, 11, 8, 13 are on the right side of the robot 

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
        FP_BackLeft   = new double3( 0.5385, 1.0793, -1.4520) + back_left_correction;
        FP_BackRight  = new double3( 0.5234, 1.6350, -1.4463) + back_right_correction;
        FP_FrontLeft  = new double3(-0.3202, 1.0510, -1.4525) + front_left_correction;
        FP_FrontRight = new double3(-0.3341, 1.6077, -1.4458) + front_right_correction;        
        
        FramePulleyPositions = new double3[numCables];
        LocalAttachmentPoints = new double3[numCables];
        SolverToMotorMap = new int[numCables];

        // Determine active subset based on requested numCables
        int[] activeIndices = BuildActiveCableIndices(numCables);

        // Temporary simplification: one estimated attachment point per tracker side.
        const double leftAttachmentOffsetY = 0.06;
        const double rightAttachmentOffsetY = -0.06;
        
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
