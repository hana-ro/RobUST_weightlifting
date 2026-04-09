using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Mathematics;
using Unity.Profiling;

using System;
using System.Threading;

/// <summary>
/// Main controller for the cable-driven robot.
/// This class centralizes the robot's state management, coordinating trackers,
/// force plates, physics calculations, and communication with LabVIEW.
/// </summary>
public class RobotController : MonoBehaviour
{
    static readonly ProfilerCounterValue<long> s_WorkloadNs = new(RobotProfiler.Workloads, "Controller Workload", ProfilerMarkerDataUnit.TimeNanoseconds);
    static readonly ProfilerCounterValue<long> s_IntervalNs = new(RobotProfiler.Intervals, "Controller Execution Interval", ProfilerMarkerDataUnit.TimeNanoseconds);

    [Header("Module References")]
    [Tooltip("The TrackerManager instance that provides tracker data.")]
    [SerializeField] private TrackerManager trackerManager;
    [Tooltip("The ForcePlateManager instance for reading force plate data.")]
    [SerializeField] private ForcePlateManager forcePlateManager;
    [Tooltip("The LabviewTcpCommunicator instance for sending motor commands.")]
    [SerializeField] private LabviewTcpCommunicator tcpCommunicator;

    [Header("Tracker Requirements")]
    [Tooltip("Require CoM tracker at startup. Disable for EE + Frame testing.")]
    [SerializeField] private bool requireComTracker = false;
    [Tooltip("Require frame tracker at startup. Keep enabled when frame defines robot origin.")]
    [SerializeField] private bool requireFrameTracker = true;

    [Header("Visualization")]
    [Tooltip("Handles tracker/camera visuals. Keeps RobotController logic-only.")]
    [SerializeField] private RobotVisualizer visualizer;

    [Header("Control Settings")]
    [Tooltip("Enable or disable sending commands to LabVIEW.")]
    [SerializeField] private bool isLabviewControlEnabled = true;
    private bool isForcePlateEnabled = true;

    [Header("PID Target")]
    [Tooltip("Relative position offset when entering PID mode in robot frame [m]. X=forward, Y=up, Z=lateral.")]
    [SerializeField] private Vector3 pidPositionOffsetMeters = new Vector3(0.0f, 0.3f, 0.0f);

    [Header("Vertical Trajectory")]
    [Tooltip("Vertical lift amplitude [m] around the captured PID target.")]
    [SerializeField] private float trajectoryLiftHeightMeters = 0.20f;
    [Tooltip("Time [s] to move from bottom to top (and top to bottom).")]
    [SerializeField] private float trajectoryMoveDurationSec = 4.0f;
    [Tooltip("Pause time [s] at the top and bottom of the trajectory.")]
    [SerializeField] private float trajectoryPauseDurationSec = 1.0f;

    [Header("Tension Mode")]
    [Tooltip("Constant Cartesian force target in robot frame [N] used in TENSION mode.")]
    [SerializeField] private Vector3 tensionModeForceN = new Vector3(0.0f, 0.0f, 0.0f);
    [Tooltip("Constant Cartesian torque target in robot frame [N*m] used in TENSION mode.")]
    [SerializeField] private Vector3 tensionModeTorqueNm = new Vector3(0.0f, 0.0f, 0.0f);
    [Tooltip("Enable smooth ramp-in when entering TENSION mode.")]
    [SerializeField] private bool tensionModeRampEnabled = true;
    [Tooltip("Ramp-in duration [s] for TENSION mode wrench target.")]
    [SerializeField] private float tensionModeRampDurationSec = 1.5f;
    [Tooltip("Enable force closed-loop in TENSION mode using force plate net force.")]
    [SerializeField] private bool tensionModeClosedLoopEnabled = true;
    public enum TensionFeedbackSource { OFF, FORCE_PLATE, ESTIMATED_WRENCH }
    [Tooltip("Feedback source for TENSION closed-loop. ESTIMATED_WRENCH uses planner-estimated resultant wrench from previous cycle.")]
    [SerializeField] private TensionFeedbackSource tensionFeedbackSource = TensionFeedbackSource.ESTIMATED_WRENCH;
    // Closed-loop gains are code-only to avoid runtime inspector retuning.
    private readonly Vector3 tensionModeForceKp = new Vector3(0.2f, 0.2f, 0.2f);
    private readonly Vector3 tensionModeForceKi = new Vector3(0.05f, 0.05f, 0.05f);
    private readonly Vector3 tensionModeForceIntegralLimit = new Vector3(120.0f, 120.0f, 120.0f);
    private readonly float tensionModeMaxForceCorrectionN = 120.0f;

    public enum CONTROL_MODE { OFF, TRANSPARENT, IMPEDANCE, PID, TENSION }
    [SerializeField] private volatile CONTROL_MODE currentControlMode = CONTROL_MODE.OFF;
    private CONTROL_MODE previousControlMode = CONTROL_MODE.OFF;

    [Header("Robot Geometry Configuration")]
    [Tooltip("Number of cables in the system. We currently support 8 or 4 cables")]
    [SerializeField] private int numCables = 8;
    [Tooltip("Measured thickness of the chest in the anterior-posterior direction [m].")]
    [SerializeField] private float chestAPDistance = 0.2f;
    [Tooltip("Measured width of the chest in the medial-lateral direction [m].")]
    [SerializeField] private float chestMLDistance = 0.3f;
    [Tooltip("User body mass [kg].")]
    [SerializeField] private float userMass = 70.0f;

    [Header("Data Logging")]
    [Tooltip("Check this to record data to RAM. It will be saved to disk on Stop.")]
    [SerializeField] private volatile bool isLogging = false;
    [Tooltip("Name of the experiment session for the log file.")]
    [SerializeField] private string sessionName = "Experiment";

    private RobUSTDescription robotDescription;
    private DataLogger dataLogger;

    // Controllers
    private ImpedanceController impedanceController;
    private PIDController pidController;
    private TensionController tensionController;
    private CableTensionPlanner tensionPlanner;
    private TrajectoryPlanner trajectoryPlanner;

    // Static frame reference captured only at startup to prevent drift
    private TrackerData robot_frame_tracker;
    private Thread controllerThread;
    private volatile bool isRunning = false;
    private volatile bool isTrajectoryActive = false;
    private volatile bool pidRecaptureRequested = false;
    private volatile bool trajectoryRestartRequested = false;
    private volatile bool tensionModeRampRestartRequested = false;
    private bool pidTargetCaptured = false;
    private RBState pidCapturedTarget;

    private void Start()
    {
        using (System.Diagnostics.Process p = System.Diagnostics.Process.GetCurrentProcess())
        {
            p.PriorityClass = System.Diagnostics.ProcessPriorityClass.High;
        }

        if (!ValidateModules())
        {
            enabled = false; 
            return;
        }

        if (chestAPDistance <= 0 || chestMLDistance <= 0)
        {
            Debug.LogError("Chest dimensions must be positive values.", this);
            enabled = false;
            return;
        }

        // Create RobUST description (single allocation at startup)
        robotDescription = RobUSTDescription.Create(numCables, chestAPDistance, chestMLDistance, userMass);
        Debug.Log($"RobUST description created: {numCables} cables, AP={chestAPDistance}m, ML={chestMLDistance}m");

        // Initialize Modules
        tcpCommunicator.Initialize();
        if (!visualizer.Initialize(robotDescription))
        {
            Debug.LogError("Failed to initialize RobotVisualizer.", this);
            enabled = false;
            return;
        }

        // Enforce runtime tracker requirements from RobotController to avoid stale prefab values on TrackerManager.
        trackerManager.requireComTracker = requireComTracker;
        trackerManager.requireFrameTracker = requireFrameTracker;

        if (!trackerManager.Initialize())
        {
            Debug.LogError("Failed to initialize TrackerManager.", this);
            enabled = false;
            return;
        }
        if (!forcePlateManager.Initialize(robotDescription))
        {
            Debug.LogWarning("Failed to initialize ForcePlateManager.", this);
            isForcePlateEnabled = false;
        }
        
        if (isLabviewControlEnabled)
        {
            tcpCommunicator.ConnectToServer();
            tcpCommunicator.SetClosedLoopControl();
        }
        dataLogger = new DataLogger(60, 100);

        System.Threading.Thread.Sleep(500); // allow tracker thread to go live
        if (trackerManager.HasFrameTracker)
            trackerManager.GetFrameTrackerData(out robot_frame_tracker); // import captured static frame at startup
        else
            robot_frame_tracker = new TrackerData(Matrix4x4.identity);

        // Finally Initialize Controller modules and begin control thread
        impedanceController = new ImpedanceController(userMass);
        pidController = new PIDController(userMass);
        tensionController = new TensionController();
        tensionPlanner = new CableTensionPlanner(robotDescription);
        trajectoryPlanner = new TrajectoryPlanner();
        
        controllerThread = new Thread(controlLoop)
        {
            Name = "Robot Controller Main",
            IsBackground = true,
            Priority = System.Threading.ThreadPriority.AboveNormal
        };

        isRunning = true;
        controllerThread.Start();
    }

    private void Update()
    {
        if (Keyboard.current == null) return;
        if (Keyboard.current.spaceKey.wasPressedThisFrame)
        {
            isTrajectoryActive = !isTrajectoryActive;
            if (isTrajectoryActive)
                trajectoryRestartRequested = true;
        }
        if (Keyboard.current.oKey.wasPressedThisFrame) { currentControlMode = CONTROL_MODE.OFF; isTrajectoryActive = false; }
        if (Keyboard.current.tKey.wasPressedThisFrame)
        { 
            currentControlMode = CONTROL_MODE.TRANSPARENT; 
            isTrajectoryActive = false; 
        }
        if (Keyboard.current.pKey.wasPressedThisFrame)
        {
            currentControlMode = CONTROL_MODE.PID;
            pidRecaptureRequested = true;
        }
        if (Keyboard.current.yKey.wasPressedThisFrame)
        {
            currentControlMode = CONTROL_MODE.TENSION;
            isTrajectoryActive = false;
            tensionModeRampRestartRequested = true;
        }
    }

    /// <summary>
    ///  This control loop is the main "driver" of one or more `Controller` instances. 
    ///  It needs to grab the latest tracker data, update visuals, compute cable tensions, and send commands to LabVIEW.
    ///  a `Controller` functions more like a `Control Policy/Solver` here, with RobotController managing the data flow.
    /// </summary>
    private void controlLoop()
    {
        double ctrl_freq = 100.0;
        
        Span<double> motor_tension_command = stackalloc double[14];
        double[] solver_tensions = new double[robotDescription.NumCables];
        
        
        double4x4 framePose = ToDouble4x4(robot_frame_tracker.PoseMatrix);
        double4x4 frameInv = math.fastinverse(framePose);
        SensorFilter filter_10Hz = new SensorFilter(ctrl_freq, 10.0);
        Span<RBState> Xref_horizon = stackalloc RBState[10];

        double system_frequency = System.Diagnostics.Stopwatch.Frequency;
        double ticksToNs = 1_000_000_000.0 / system_frequency;
        long intervalTicks = (long)(system_frequency / ctrl_freq);
        long nextTargetTime = System.Diagnostics.Stopwatch.GetTimestamp() + intervalTicks;
        long lastLoopTick = System.Diagnostics.Stopwatch.GetTimestamp();

        while (isRunning)
        {
            if (pidRecaptureRequested)
            {
                pidController?.Reset();
                pidTargetCaptured = false;
                pidRecaptureRequested = false;
                trajectoryRestartRequested = true;
            }

            if (currentControlMode != previousControlMode)
            {
                pidController?.Reset();
                tensionController?.Reset();
                // Reset tension planner history to prevent jerks from stale warm-starts
                tensionPlanner?.Reset();
                pidTargetCaptured = false;
                trajectoryRestartRequested = true;
                tensionModeRampRestartRequested = currentControlMode == CONTROL_MODE.TENSION;
                previousControlMode = currentControlMode;
            }

            long loopStartTick = System.Diagnostics.Stopwatch.GetTimestamp();
            s_IntervalNs.Value = (long)((loopStartTick - lastLoopTick) * ticksToNs);
            lastLoopTick = loopStartTick;

            trackerManager.GetEndEffectorTrackerData(out TrackerData rawEndEffectorData);
            TrackerData rawComData;
            if (trackerManager.HasComTracker)
                trackerManager.GetCoMTrackerData(out rawComData);
            else
                rawComData = rawEndEffectorData;
            double4x4 eePose_RF = math.mul(frameInv, ToDouble4x4(rawEndEffectorData.PoseMatrix));
            double4x4 comPose_RF = math.mul(frameInv, ToDouble4x4(rawComData.PoseMatrix));

            // Debug.Log($"comPose_RF: {comPose_RF.c3}");

            // FP already in robot frame (fallback to zero when force plates are unavailable)
            ForcePlateData fp0 = new ForcePlateData(double3.zero, double3.zero);
            ForcePlateData fp1 = new ForcePlateData(double3.zero, double3.zero);
            if (isForcePlateEnabled)
            {
                forcePlateManager.GetForcePlateData(0, out fp0);
                forcePlateManager.GetForcePlateData(1, out fp1);
            }
            double3 netForce = fp0.Force + fp1.Force;
            double3 netCoP = double3.zero;
            if (math.abs(netForce.z) > 1e-3) // Handle division by zero
                netCoP = (fp0.CoP * fp0.Force.z + fp1.CoP * fp1.Force.z) / netForce.z;
            ForcePlateData netFPData = new ForcePlateData(netForce, netCoP);
            
            filter_10Hz.Update(comPose_RF, eePose_RF, netFPData);

            double3 tensionFeedbackForce = double3.zero;
            bool hasTensionFeedbackForce = false;
            if (tensionModeClosedLoopEnabled)
            {
                switch (tensionFeedbackSource)
                {
                    case TensionFeedbackSource.FORCE_PLATE:
                        if (isForcePlateEnabled)
                        {
                            tensionFeedbackForce = netFPData.Force;
                            hasTensionFeedbackForce = true;
                        }
                        break;
                    case TensionFeedbackSource.ESTIMATED_WRENCH:
                        Wrench estimatedWrench = tensionPlanner.CalculateResultantWrench(eePose_RF, solver_tensions);
                        tensionFeedbackForce = estimatedWrench.Force;
                        hasTensionFeedbackForce = true;
                        break;
                }
            }

            switch (currentControlMode)
            {
                case CONTROL_MODE.OFF:
                    motor_tension_command.Clear();
                    Array.Clear(solver_tensions, 0, solver_tensions.Length);
                    break;
                case CONTROL_MODE.TRANSPARENT:
                    switch (robotDescription.NumCables)
                    {
                        case 8:
                            Wrench zeroWrench = new Wrench(double3.zero, double3.zero);
                            solver_tensions = tensionPlanner.CalculateTensions(eePose_RF, zeroWrench);
                            MapTensionsToMotors(solver_tensions, motor_tension_command);
                            break;
                        case 4:
                            solver_tensions[0] = 10.0; solver_tensions[1] = 10.0;
                            solver_tensions[2] = 10.0; solver_tensions[3] = 10.0;
                            MapTensionsToMotors(solver_tensions, motor_tension_command);
                            break;
                    }
                    visualizer.PushGoalTrajectory(Xref_horizon.Slice(0, 1));
                    break;
                case CONTROL_MODE.IMPEDANCE:
                    RBState staticPoint = new RBState(
                        new double3(0.2, 0.825, 0.01), // Default: x=0.2 (forward), y=0.825 (up), z=0.01 (center)
                        new double3(0, 0, -math.PI/2), // Default: Facing forward (Rotate -90 deg around Z if using Z-up, Y-forward convention?)
                        new double3(0, 0, 0), 
                        new double3(0, 0, 0)
                    );
                    
                    Xref_horizon.Fill(staticPoint);

                    impedanceController.UpdateState(eePose_RF, filter_10Hz.EELinearVelocity, filter_10Hz.EEAngularVelocity, staticPoint);
                    Wrench goalWrench = impedanceController.computeNextControl();
                    solver_tensions = tensionPlanner.CalculateTensions(eePose_RF, goalWrench);
                    MapTensionsToMotors(solver_tensions, motor_tension_command);
                    visualizer.PushGoalTrajectory(Xref_horizon.Slice(0, 1));
                    break;
                case CONTROL_MODE.PID:
                    if (!pidTargetCaptured)
                    {
                        double3 currentPos = eePose_RF.c3.xyz;
                        double3 targetPos = currentPos + new double3(
                            pidPositionOffsetMeters.x,
                            pidPositionOffsetMeters.y,
                            pidPositionOffsetMeters.z
                        );

                        double3 currentEulerZYX = RotationToEulerZYX(eePose_RF);
                        pidCapturedTarget = new RBState(targetPos, currentEulerZYX, double3.zero, double3.zero);
                        trajectoryPlanner.Configure(trajectoryLiftHeightMeters, trajectoryMoveDurationSec, trajectoryPauseDurationSec, ctrl_freq);
                        trajectoryPlanner.Reset(pidCapturedTarget);
                        pidTargetCaptured = true;
                        Debug.Log($"PID target captured: current EE + offset ({pidPositionOffsetMeters.x:F3}, {pidPositionOffsetMeters.y:F3}, {pidPositionOffsetMeters.z:F3}) m. Vertical trajectory lift={trajectoryLiftHeightMeters:F3} m.");
                    }

                    if (trajectoryRestartRequested)
                    {
                        trajectoryPlanner.Configure(trajectoryLiftHeightMeters, trajectoryMoveDurationSec, trajectoryPauseDurationSec, ctrl_freq);
                        trajectoryPlanner.Reset(pidCapturedTarget);
                        trajectoryRestartRequested = false;
                    }

                    RBState pidTarget = trajectoryPlanner.GetReference(isTrajectoryActive);
                    Xref_horizon.Fill(pidTarget);

                    pidController.UpdateState(
                        eePose_RF,
                        filter_10Hz.EELinearVelocity,
                        filter_10Hz.EEAngularVelocity,
                        pidTarget,
                        1.0 / ctrl_freq
                    );

                    Wrench pidGoalWrench = pidController.computeNextControl();
                    solver_tensions = tensionPlanner.CalculateTensions(eePose_RF, pidGoalWrench);
                    MapTensionsToMotors(solver_tensions, motor_tension_command);
                    visualizer.PushGoalTrajectory(Xref_horizon.Slice(0, 1));
                    break;
                case CONTROL_MODE.TENSION:
                    Wrench targetTensionWrench = new Wrench(
                        new double3(tensionModeForceN.x, tensionModeForceN.y, tensionModeForceN.z),
                        new double3(tensionModeTorqueNm.x, tensionModeTorqueNm.y, tensionModeTorqueNm.z)
                    );

                    tensionController.Configure(tensionModeRampEnabled, tensionModeRampDurationSec);
                    tensionController.ConfigureClosedLoop(
                        tensionModeClosedLoopEnabled && hasTensionFeedbackForce,
                        new double3(tensionModeForceKp.x, tensionModeForceKp.y, tensionModeForceKp.z),
                        new double3(tensionModeForceKi.x, tensionModeForceKi.y, tensionModeForceKi.z),
                        new double3(tensionModeForceIntegralLimit.x, tensionModeForceIntegralLimit.y, tensionModeForceIntegralLimit.z),
                        tensionModeMaxForceCorrectionN
                    );
                    if (hasTensionFeedbackForce)
                        tensionController.UpdateMeasuredForce(tensionFeedbackForce);

                    if (tensionModeRampRestartRequested)
                    {
                        tensionController.EnterMode(targetTensionWrench);
                        tensionModeRampRestartRequested = false;
                    }
                    else
                    {
                        tensionController.UpdateTarget(targetTensionWrench);
                    }

                    tensionController.Step(1.0 / ctrl_freq);
                    Wrench tensionModeWrench = tensionController.computeNextControl();

                    solver_tensions = tensionPlanner.CalculateTensions(eePose_RF, tensionModeWrench);
                    MapTensionsToMotors(solver_tensions, motor_tension_command);

                    RBState holdState = new RBState(
                        eePose_RF.c3.xyz,
                        RotationToEulerZYX(eePose_RF),
                        double3.zero,
                        double3.zero
                    );
                    Xref_horizon.Fill(holdState);
                    visualizer.PushGoalTrajectory(Xref_horizon.Slice(0, 1));
                    break;
            }
            tcpCommunicator.UpdateTensionSetpoint(motor_tension_command);
            visualizer.PushState(comPose_RF, eePose_RF, fp0, fp1);
            if (isLogging)
            {
                Wrench goalWrench = tensionPlanner.CalculateResultantWrench(eePose_RF, solver_tensions);
                dataLogger.Log(loopStartTick, comPose_RF, eePose_RF, fp0, fp1, goalWrench, Xref_horizon[0]);
            }

            s_WorkloadNs.Value = (long)((System.Diagnostics.Stopwatch.GetTimestamp() - loopStartTick) * ticksToNs);
            while (System.Diagnostics.Stopwatch.GetTimestamp() < nextTargetTime) { } // BURN wait

            nextTargetTime += intervalTicks;
            long now = System.Diagnostics.Stopwatch.GetTimestamp();
            if (now > nextTargetTime) nextTargetTime = now + intervalTicks; // drift correction
        }
    }


    // Validates that all required module references are assigned in the Inspector.
    // returns True if all modules are assigned, false otherwise.
    private bool ValidateModules()
    {
        bool allValid = true;
        if (trackerManager == null) { Debug.LogError("Module not assigned in Inspector: trackerManager", this); allValid = false; }
        if (forcePlateManager == null) { Debug.LogError("Module not assigned in Inspector: forcePlateManager", this); allValid = false; }
        if (tcpCommunicator == null) { Debug.LogError("Module not assigned in Inspector: tcpCommunicator", this); allValid = false; }
        if (visualizer == null) { Debug.LogError("Visualizer not assigned in Inspector: visualizer", this); allValid = false; }

        return allValid;
    }

    private void OnDestroy()
    {
        // Clean shutdown of threaded components.
        // TrackerManager handles its own shutdown via its OnDestroy method.
        isRunning = false;
        tcpCommunicator?.Disconnect();
        if (dataLogger != null && dataLogger.FrameCount > 0)
            dataLogger.WriteToDisk(sessionName);
    }


    /// <summary>
    /// Maps solver tensions to the 14-motor driver array using the active configuration.
    /// </summary>
    private void MapTensionsToMotors(double[] solverResult, Span<double> output)
    {
        output.Clear();
        int count = robotDescription.SolverToMotorMap.Length;
        for (int i = 0; i < count; i++)
        {
            int motorIndex = robotDescription.SolverToMotorMap[i];
            output[motorIndex] = solverResult[i];
        }
    }

    // Local helper
    private static double4x4 ToDouble4x4(in Matrix4x4 m)
    {
        return new double4x4(
            m.m00, m.m01, m.m02, m.m03,
            m.m10, m.m11, m.m12, m.m13,
            m.m20, m.m21, m.m22, m.m23,
            m.m30, m.m31, m.m32, m.m33
        );
    }

    /// <summary>
    /// Converts a rotation matrix to ZYX Euler angles (roll=x, pitch=y, yaw=z)
    /// such that R = Rz(yaw) * Ry(pitch) * Rx(roll).
    /// </summary>
    private static double3 RotationToEulerZYX(double4x4 pose)
    {
        double r00 = pose.c0.x;
        double r01 = pose.c1.x;
        double r10 = pose.c0.y;
        double r11 = pose.c1.y;
        double r20 = pose.c0.z;
        double r21 = pose.c1.z;
        double r22 = pose.c2.z;

        double pitch = math.asin(math.clamp(-r20, -1.0, 1.0));
        double cp = math.cos(pitch);

        double roll;
        double yaw;
        if (math.abs(cp) > 1e-6)
        {
            roll = math.atan2(r21, r22);
            yaw = math.atan2(r10, r00);
        }
        else
        {
            roll = 0.0;
            yaw = math.atan2(-r01, r11);
        }

        return new double3(roll, pitch, yaw);
    }

}
