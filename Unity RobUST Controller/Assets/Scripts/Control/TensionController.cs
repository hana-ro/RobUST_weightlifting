using UnityEngine;
using Unity.Mathematics;
using static Unity.Mathematics.math;

/// <summary>
/// A tension-oriented wrench controller intended for tension/wrench style assistance.
///
/// Outputs a desired wrench that can be passed into CableTensionPlanner.
/// - Open-loop mode: constant feedforward wrench (e.g. fixed downward force)
/// - Closed-loop mode: PID correction on measured wrench tracking error
///
/// This is not a direct per-cable tension controller; it is a task-space wrench
/// controller that relies on CableTensionPlanner for tension distribution.
/// </summary>
public class TensionController : BaseController<Wrench>
{
    [Header("Feedforward Wrench Setpoint (Robot Frame)")]
    public double3 DesiredForce = new double3(0.0);
    public double3 DesiredTorque = new double3(0.0);

    [Header("Closed-Loop Options")]
    public bool EnableForceFeedback = true;
    public bool EnableTorqueFeedback = false;

    [Header("PID Gains")]
    public double3 KpForce = new double3(0.3, 0.3, 0.3);
    public double3 KiForce = new double3(0.2, 0.2, 0.2);
    public double3 KdForce = new double3(0.02, 0.02, 0.02);
    public double3 KpTorque = new double3(0.1, 0.1, 0.1);
    public double3 KiTorque = new double3(0.05, 0.05, 0.05);
    public double3 KdTorque = new double3(0.01, 0.01, 0.01);

    [Header("Integrator Limits")]
    public double3 ForceIntegratorLimit = new double3(200.0, 200.0, 200.0);
    public double3 TorqueIntegratorLimit = new double3(50.0, 50.0, 50.0);

    [Header("Output Saturation")]
    public double MaxForce = 300.0;
    public double MaxTorque = 30.0;

    private double dt = 0.01;

    private double3 measuredForce = new double3(0.0);
    private double3 measuredTorque = new double3(0.0);
    private bool hasMeasurement = false;

    private double3 forceIntegral = new double3(0.0);
    private double3 torqueIntegral = new double3(0.0);
    private double3 previousForceError = new double3(0.0);
    private double3 previousTorqueError = new double3(0.0);
    private bool hasPreviousForceError = false;
    private bool hasPreviousTorqueError = false;

    /// <param name="controlFrequencyHz">Main control-loop frequency in Hz (default 100 Hz)</param>
    public TensionController(double controlFrequencyHz = 100.0)
    {
        SetControlFrequency(controlFrequencyHz);
    }

    public void SetControlFrequency(double controlFrequencyHz)
    {
        dt = controlFrequencyHz > 1e-6 ? 1.0 / controlFrequencyHz : 0.01;
    }

    public void SetSetpoint(Wrench target)
    {
        DesiredForce = target.Force;
        DesiredTorque = target.Torque;
    }

    /// <summary>
    /// Update measured wrench in robot frame.
    /// This can come from force sensors or a wrench estimator.
    /// </summary>
    public void UpdateMeasurement(Wrench measuredWrench)
    {
        measuredForce = measuredWrench.Force;
        measuredTorque = measuredWrench.Torque;
        hasMeasurement = true;
    }

    public void ResetIntegrator()
    {
        forceIntegral = new double3(0.0);
        torqueIntegral = new double3(0.0);
        previousForceError = new double3(0.0);
        previousTorqueError = new double3(0.0);
        hasPreviousForceError = false;
        hasPreviousTorqueError = false;
    }

    public override Wrench computeNextControl()
    {
        double3 forceCmd = DesiredForce;
        double3 torqueCmd = DesiredTorque;

        if (hasMeasurement)
        {
            if (EnableForceFeedback)
            {
                double3 eForce = DesiredForce - measuredForce;
                forceIntegral += eForce * dt;
                forceIntegral = ClampPerAxis(forceIntegral, ForceIntegratorLimit);

                double3 dForce = new double3(0.0);
                if (hasPreviousForceError)
                    dForce = (eForce - previousForceError) / dt;
                previousForceError = eForce;
                hasPreviousForceError = true;

                forceCmd += (KpForce * eForce) + (KiForce * forceIntegral) + (KdForce * dForce);
            }

            if (EnableTorqueFeedback)
            {
                double3 eTorque = DesiredTorque - measuredTorque;
                torqueIntegral += eTorque * dt;
                torqueIntegral = ClampPerAxis(torqueIntegral, TorqueIntegratorLimit);

                double3 dTorque = new double3(0.0);
                if (hasPreviousTorqueError)
                    dTorque = (eTorque - previousTorqueError) / dt;
                previousTorqueError = eTorque;
                hasPreviousTorqueError = true;

                torqueCmd += (KpTorque * eTorque) + (KiTorque * torqueIntegral) + (KdTorque * dTorque);
            }
        }

        if (length(forceCmd) > MaxForce)
            forceCmd = normalize(forceCmd) * MaxForce;

        if (length(torqueCmd) > MaxTorque)
            torqueCmd = normalize(torqueCmd) * MaxTorque;

        return new Wrench(forceCmd, torqueCmd);
    }

    private static double3 ClampPerAxis(double3 value, double3 limits)
    {
        return new double3(
            clamp(value.x, -limits.x, limits.x),
            clamp(value.y, -limits.y, limits.y),
            clamp(value.z, -limits.z, limits.z)
        );
    }
}
