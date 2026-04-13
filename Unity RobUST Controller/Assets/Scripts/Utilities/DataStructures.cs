using UnityEngine;
using Unity.Mathematics;
using Unity.Profiling;

public static class RobotProfiler
{
    public static readonly ProfilerCategory Workloads = new("Robot Thread Workloads");
    public static readonly ProfilerCategory Intervals = new("Robot Thread Intervals");
}
/// <summary>
/// A simple data structure to hold the position and rotation of a tracker.
/// NOTE: Data is stored in a RIGHT-HANDED coordinate system (OpenVR standard).
/// </summary>
[System.Serializable]
public struct TrackerData
{
    /// <summary>
    /// The full 4x4 homogeneous transformation matrix representing the tracker's pose.
    /// NOTE: This matrix is in the RIGHT-HANDED coordinate system (OpenVR standard).
    /// </summary>
    public Matrix4x4 PoseMatrix;

    public TrackerData(Matrix4x4 poseMatrix)
    {
        PoseMatrix = poseMatrix;
    }
}

/// <summary>
/// A simple data structure to hold force and moment data from a force plate.
/// </summary>
[System.Serializable]
public struct ForcePlateData
{
    public double3 Force;
    public double3 CoP;

    public ForcePlateData(double3 force, double3 cop)
    {
        Force = force;
        CoP = cop;
    }
}

/// <summary>
/// Rigid body state for MPC prediction.
/// </summary>
[System.Serializable]
public struct RBState
{
    public double3 p;   // position [m]
    public double3 th;  // ZYX Euler angles [rad]
    public double3 v;   // linear velocity [m/s]
    public double3 w;   // angular velocity [rad/s]

    public RBState(double3 position, double3 eulerAngles, double3 linearVelocity, double3 angularVelocity)
    {
        p = position;
        th = eulerAngles;
        v = linearVelocity;
        w = angularVelocity;
    }
}


[System.Serializable]
public struct Wrench
{
    public double3 Force;
    public double3 Torque;

    public Wrench(double3 force, double3 torque)
    {
        Force = force;
        Torque = torque;
    }
}

/// <summary>
/// Abstract base class for any high-level controller that outputs cable tensions.
/// Both MPCController and StabilityController derive from this.
/// </summary>
public abstract class BaseController<T>
{
    public abstract T computeNextControl();
}

/// <summary>
/// Builds a robust midpoint/barbell pose from left and right end-effector poses.
/// Uses the left-right position baseline as the primary lateral axis and a pose-derived
/// up hint to avoid sudden quaternion averaging flips.
/// </summary>
public static class PoseFusion
{
    private const float MinBaseline = 1e-5f;

    public static double4x4 BuildBarbellPose(double4x4 poseL, double4x4 poseR)
    {
        double3 center = 0.5 * (poseL.c3.xyz + poseR.c3.xyz);

        float3 pL = (float3)poseL.c3.xyz;
        float3 pR = (float3)poseR.c3.xyz;
        float3 baseline = pR - pL;

        float3 axisY;
        if (math.lengthsq(baseline) > MinBaseline)
        {
            axisY = math.normalize(baseline);
        }
        else
        {
            axisY = new float3(0f, 1f, 0f);
        }

        // Robot frame convention: +Z is vertical.
        float3 worldUp = new float3(0f, 0f, 1f);
        float3 axisZ = worldUp - math.dot(worldUp, axisY) * axisY;
        if (math.lengthsq(axisZ) < MinBaseline)
        {
            // If baseline becomes near-vertical, choose a stable fallback axis.
            float3 fallback = math.abs(axisY.x) < 0.9f ? new float3(1f, 0f, 0f) : new float3(0f, 1f, 0f);
            axisZ = fallback - math.dot(fallback, axisY) * axisY;
        }
        axisZ = math.normalize(axisZ);

        float3 axisX = math.normalize(math.cross(axisY, axisZ));
        axisZ = math.normalize(math.cross(axisX, axisY));

        return new double4x4(
            new double4(axisX.x, axisX.y, axisX.z, 0.0),
            new double4(axisY.x, axisY.y, axisY.z, 0.0),
            new double4(axisZ.x, axisZ.y, axisZ.z, 0.0),
            new double4(center, 1.0)
        );
    }
}
