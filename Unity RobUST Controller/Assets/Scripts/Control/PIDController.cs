using Unity.Mathematics;

/// <summary>
/// Simple Cartesian PID controller that outputs a desired wrench (force + torque).
/// Force is computed from position/velocity PID terms and torque from orientation/angular-rate PID terms.
/// </summary>
public class PIDController : BaseController<Wrench>
{
    // Position gains
    public double3 Kp_pos;
    public double3 Ki_pos;
    public double3 Kd_pos;

    // Orientation gains
    public double3 Kp_ori;
    public double3 Ki_ori;
    public double3 Kd_ori;

    // Saturation limits
    public double MaxForce = 300.0;
    public double MaxTorque = 30.0;

    // Anti-windup limits for integral state (per-axis)
    public double3 PosIntegralLimit = new double3(0.25, 0.25, 0.25);
    public double3 OriIntegralLimit = new double3(0.4, 0.4, 0.4);

    private double4x4 currentPose;
    private double3 currentLinVel;
    private double3 currentAngVel;
    private RBState targetState;
    private double dt = 0.01;
    private bool isStateValid = false;

    private double3 posIntegral;
    private double3 oriIntegral;

    public PIDController(float userMass)
    {
        // Conservative defaults to start tuning safely.
        double kpPosXY = 300.0 / 68.0 * userMass;
        double kpPosZ = 500.0 / 68.0 * userMass;

        Kp_pos = new double3(kpPosXY, kpPosXY, kpPosZ);
        Ki_pos = new double3(5.0, 5.0, 7.0);
        Kd_pos = new double3(20.0, 20.0, 25.0);

        Kp_ori = new double3(4.0, 4.0, 4.0);
        Ki_ori = new double3(0.1, 0.1, 0.1);
        Kd_ori = new double3(0.3, 0.3, 0.3);
    }

    /// <summary>
    /// Updates measured state and target before computeNextControl().
    /// </summary>
    public void UpdateState(double4x4 pose_RF, double3 linVel_RF, double3 angVel_RF, RBState target, double sampleTimeSec)
    {
        currentPose = pose_RF;
        currentLinVel = linVel_RF;
        currentAngVel = angVel_RF;
        targetState = target;

        if (sampleTimeSec > 1e-5)
            dt = sampleTimeSec;

        isStateValid = true;
    }

    public void Reset()
    {
        posIntegral = double3.zero;
        oriIntegral = double3.zero;
        isStateValid = false;
    }

    public override Wrench computeNextControl()
    {
        if (!isStateValid)
            return new Wrench(double3.zero, double3.zero);

        // Position PID
        double3 pCurr = currentPose.c3.xyz;
        double3 ePos = targetState.p - pCurr;
        double3 eVel = targetState.v - currentLinVel;

        posIntegral = ClampPerAxis(posIntegral + (ePos * dt), -PosIntegralLimit, PosIntegralLimit);
        double3 forceCmd = (Kp_pos * ePos) + (Ki_pos * posIntegral) + (Kd_pos * eVel);

        // Orientation PID (rotation-vector error from quaternion delta)
        quaternion qz = quaternion.AxisAngle(new float3(0f, 0f, 1f), (float)targetState.th.z);
        quaternion qy = quaternion.AxisAngle(new float3(0f, 1f, 0f), (float)targetState.th.y);
        quaternion qx = quaternion.AxisAngle(new float3(1f, 0f, 0f), (float)targetState.th.x);
        quaternion qDes = math.mul(qz, math.mul(qy, qx));

        quaternion qCurr = new quaternion(new float3x3((float3)currentPose.c0.xyz, (float3)currentPose.c1.xyz, (float3)currentPose.c2.xyz));
        quaternion qDiff = math.mul(qDes, math.inverse(qCurr));

        double3 eOri = double3.zero;
        double wClamped = math.clamp(qDiff.value.w, -1.0f, 1.0f);
        if (math.abs(wClamped) < 0.999999)
        {
            double theta = 2.0 * math.acos(wClamped);
            double sinHalfTheta = math.sqrt(1.0 - wClamped * wClamped);
            if (sinHalfTheta > 0.001)
                eOri = ((double3)qDiff.value.xyz / sinHalfTheta) * theta;
        }

        double3 eAngVel = targetState.w - currentAngVel;
        oriIntegral = ClampPerAxis(oriIntegral + (eOri * dt), -OriIntegralLimit, OriIntegralLimit);
        double3 torqueCmd = (Kp_ori * eOri) + (Ki_ori * oriIntegral) + (Kd_ori * eAngVel);

        // Saturation
        if (math.length(forceCmd) > MaxForce)
            forceCmd = math.normalize(forceCmd) * MaxForce;

        if (math.length(torqueCmd) > MaxTorque)
            torqueCmd = math.normalize(torqueCmd) * MaxTorque;

        return new Wrench(forceCmd, torqueCmd);
    }

    private static double3 ClampPerAxis(double3 value, double3 minValue, double3 maxValue)
    {
        return new double3(
            math.clamp(value.x, minValue.x, maxValue.x),
            math.clamp(value.y, minValue.y, maxValue.y),
            math.clamp(value.z, minValue.z, maxValue.z)
        );
    }
}
