using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Mathematics;
using Unity.Profiling;

using System;
using System.Threading;

public class RobotController : MonoBehaviour
{
    static readonly ProfilerCounterValue<long> s_WorkloadNs =
        new(RobotProfiler.Workloads, "Controller Workload", ProfilerMarkerDataUnit.TimeNanoseconds);

    static readonly ProfilerCounterValue<long> s_IntervalNs =
        new(RobotProfiler.Intervals, "Controller Execution Interval", ProfilerMarkerDataUnit.TimeNanoseconds);

    [Header("Module References")]
    [SerializeField] private TrackerManager trackerManager;
    [SerializeField] private ForcePlateManager forcePlateManager;
    [SerializeField] private LabviewTcpCommunicator tcpCommunicator;
    [SerializeField] private RobotVisualizer visualizer;

    [Header("Control Settings")]
    [SerializeField] private bool isLabviewControlEnabled = true;
    private bool isForcePlateEnabled = true;

    public enum CONTROL_MODE { OFF, TRANSPARENT, IMPEDANCE, CONSTANT }
    [SerializeField] private volatile CONTROL_MODE currentControlMode = CONTROL_MODE.OFF;



    [Header("Robot Geometry Configuration")]
    [SerializeField] private int numCables = 8;
    [SerializeField] private float chestAPDistance = 0.2f;
    [SerializeField] private float chestMLDistance = 0.3f;
    [SerializeField] private float userMass = 70.0f;
    private double3 barbellWeight = new double3(0 , 0, -22.2486876); 


    [Header("Constant Load Mode")]
    [SerializeField, Min(0f)] private float constantLoadKG = 5.0f;

    
    [Header("Data Logging")]
    [SerializeField] private volatile bool isLogging = false;
    [SerializeField] private string sessionName = "Experiment";

    [Header("Runtime Overlay")]
    [SerializeField] private bool showOverlay = true;

    private RobUSTDescription robotDescription;
    private DataLogger dataLogger;

    private ImpedanceController impedanceController;
    private CableTensionPlanner tensionPlanner;

    private TrackerData robot_frame_tracker;
    private Thread controllerThread;
    private volatile bool isRunning = false;
    private volatile bool tcpAvailable = false; 
    private volatile bool isTrajectoryActive = false;
    private bool showEndEffectorFrames = false;
    private bool showForceVisuals = true;

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

        robotDescription =
            RobUSTDescription.Create(numCables, chestAPDistance, chestMLDistance, userMass);

        tcpCommunicator.Initialize();
        visualizer.Initialize(robotDescription);
        trackerManager.Initialize();
        forcePlateManager.Initialize(robotDescription);

        if (isLabviewControlEnabled)
        {
            tcpCommunicator.ConnectToServer();
            tcpCommunicator.SetClosedLoopControl();
        }

        dataLogger = new DataLogger(60, 100);
        tcpAvailable = true;

        System.Threading.Thread.Sleep(500);
        trackerManager.GetFrameTrackerData(out robot_frame_tracker);

        // =========================
        // OTHER CONTROLLERS
        // =========================
        impedanceController = new ImpedanceController(userMass);
        tensionPlanner = new CableTensionPlanner(robotDescription);

        controllerThread = new Thread(controlLoop)
        {
            Name = "Robot Controller Main",
            IsBackground = true,
            Priority = System.Threading.ThreadPriority.Highest
        };

        isRunning = true;
        controllerThread.Start();
    }

    private void Update()
    {
        if (Keyboard.current == null) return;

        if (Keyboard.current.spaceKey.wasPressedThisFrame)
            isTrajectoryActive = !isTrajectoryActive;

        if (Keyboard.current.oKey.wasPressedThisFrame)
            currentControlMode = CONTROL_MODE.OFF;
        if (Keyboard.current.tKey.wasPressedThisFrame)
            currentControlMode = CONTROL_MODE.TRANSPARENT;

        if (Keyboard.current.iKey.wasPressedThisFrame)
            currentControlMode = CONTROL_MODE.IMPEDANCE;

        if (Keyboard.current.cKey.wasPressedThisFrame)
            currentControlMode = CONTROL_MODE.CONSTANT;
    }

    private void OnGUI()
    {
        if (!showOverlay) return;

        GUIStyle labelStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 18,
            fontStyle = FontStyle.Bold,
            normal = { textColor = Color.white }
        };

        GUIStyle buttonStyle = new GUIStyle(GUI.skin.button)
        {
            fontSize = 16,
            fontStyle = FontStyle.Bold
        };

        const float panelWidth = 0.5f;
        const float labelPad = 12f;

        GUI.Label(new Rect(labelPad, Screen.height - 34f, 140f, 24f), "Side", labelStyle);
        GUI.Label(new Rect(Screen.width * panelWidth + labelPad, Screen.height - 34f, 100f, 24f), "Back Perspective", labelStyle);

        if (GUI.Button(new Rect(12f, 12f, 250f, 32f), showEndEffectorFrames ? "Hide EE Frames" : "Show EE Frames", buttonStyle))
        {
            showEndEffectorFrames = !showEndEffectorFrames;
            if (visualizer != null)
                visualizer.SetEndEffectorFramesVisible(showEndEffectorFrames);
        }

        if (GUI.Button(new Rect(12f, 50f, 250f, 32f), showForceVisuals ? "Hide GRFs" : "Display GRFs", buttonStyle))
        {
            showForceVisuals = !showForceVisuals;
            if (visualizer != null)
                visualizer.SetForceVisualsVisible(showForceVisuals);
        }
    }

    private void controlLoop()
    {
        double ctrl_freq = 100.0;
        double framePeriodMs = 1000.0 / ctrl_freq;  // 10ms @ 100Hz

        Span<double> motor_tension_command = stackalloc double[14];
        Span<double> measured_tensions = stackalloc double[14]; 
        double[] actuated_tensions = new double[robotDescription.NumCables];
        double[] solver_tensions = new double[robotDescription.NumCables];

        double4x4 framePose = ToDouble4x4(robot_frame_tracker.PoseMatrix);
        double4x4 frameInv = math.fastinverse(framePose);

        SensorFilter filter_10Hz = new SensorFilter(ctrl_freq, 10.0);
        
        double system_frequency = System.Diagnostics.Stopwatch.Frequency;
        double ticksToNs = 1_000_000_000.0 / system_frequency;
        long intervalTicks = (long)(system_frequency / ctrl_freq);
        long nextTargetTime = System.Diagnostics.Stopwatch.GetTimestamp() + intervalTicks;
        long lastLoopTick = System.Diagnostics.Stopwatch.GetTimestamp();

        while (isRunning)
        {
            long loopStartTick = System.Diagnostics.Stopwatch.GetTimestamp();
            s_IntervalNs.Value = (long)((loopStartTick - lastLoopTick) * ticksToNs);
            lastLoopTick = loopStartTick;

            trackerManager.GetEndEffectorLeftTrackerData(out TrackerData rawL);
            trackerManager.GetEndEffectorRightTrackerData(out TrackerData rawR);
            trackerManager.GetCoMTrackerData(out TrackerData rawCom);

            double4x4 eePoseL_RF = math.mul(frameInv, ToDouble4x4(rawL.PoseMatrix));
            double4x4 eePoseR_RF = math.mul(frameInv, ToDouble4x4(rawR.PoseMatrix));
            double4x4 comPose_RF = math.mul(frameInv, ToDouble4x4(rawCom.PoseMatrix));

            Debug.Log($"comPose_RF: {comPose_RF.c3}");

            // Initialize default values for logging
            Wrench goalWrench = default;
            Wrench solverWrench = default;
            Wrench measuredWrench = default;
            RBState goalState = default;

            switch (currentControlMode)
            {
                case CONTROL_MODE.OFF:
                    motor_tension_command.Clear();
                    if (CanSendTcpCommands())
                    {
                        tcpCommunicator.SetOpenLoopControl();
                    }
                    Array.Clear(solver_tensions, 0, solver_tensions.Length);
                    break;

                case CONTROL_MODE.TRANSPARENT:
                    goalWrench = new Wrench(-barbellWeight, double3.zero);
                    solver_tensions = tensionPlanner.CalculateTensions(eePoseL_RF, eePoseR_RF, goalWrench); 
                    if (CanSendTcpCommands())
                    {
                        tcpCommunicator.SetClosedLoopControl();
                    }
                    MapTensionsToMotors(solver_tensions, motor_tension_command);
                    break;

                case CONTROL_MODE.IMPEDANCE:
                    goalWrench = impedanceController.computeNextControl();
                    solver_tensions = tensionPlanner.CalculateTensions(eePoseL_RF, eePoseR_RF, goalWrench);
                    if (CanSendTcpCommands())
                    {
                        tcpCommunicator.SetClosedLoopControl();
                    }
                    MapTensionsToMotors(solver_tensions, motor_tension_command);
                    break;

                case CONTROL_MODE.CONSTANT:
                    goalWrench = new Wrench(new double3(0, 0, -(constantLoadKG * 9.81f)) - barbellWeight, double3.zero);
                    solver_tensions = tensionPlanner.CalculateTensions(eePoseL_RF, eePoseR_RF, goalWrench);
                    if (CanSendTcpCommands())
                    {
                        tcpCommunicator.SetClosedLoopControl();
                    }
                    MapTensionsToMotors(solver_tensions, motor_tension_command);
                    break;

            }
            if (CanSendTcpCommands())
            {
                tcpCommunicator.UpdateTensionSetpoint(motor_tension_command);
            }

            //tcpCommunicator.UpdateTensionSetpoint(motor_tension_command);

            forcePlateManager.GetForcePlateData(0, out ForcePlateData fp1);
            forcePlateManager.GetForcePlateData(1, out ForcePlateData fp2);
            visualizer.PushState(comPose_RF, eePoseL_RF, eePoseR_RF, fp1, fp2);
            
            // Log data if logging is enabled
            if (isLogging)
            {
                tcpCommunicator.GetMeasuredTensions(measured_tensions); 
                MapMotorsToTensions(measured_tensions, actuated_tensions);
                solverWrench = tensionPlanner.CalculateResultantWrench(eePoseL_RF, eePoseR_RF, solver_tensions);
                measuredWrench = tensionPlanner.CalculateResultantWrench(eePoseL_RF, eePoseR_RF, actuated_tensions);
                dataLogger.Log(loopStartTick, comPose_RF, eePoseL_RF, eePoseR_RF, fp1, fp2, goalWrench, solverWrench, measuredWrench, goalState);
            }
            
            s_WorkloadNs.Value = (long)((System.Diagnostics.Stopwatch.GetTimestamp() - loopStartTick) * ticksToNs);
            while (System.Diagnostics.Stopwatch.GetTimestamp() < nextTargetTime) { } // BURN wait

            nextTargetTime += intervalTicks;
            long now = System.Diagnostics.Stopwatch.GetTimestamp();
            if (now > nextTargetTime) nextTargetTime = now + intervalTicks; // drift correction

        }
    }

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

    private void MapMotorsToTensions(ReadOnlySpan<double> motorTensions, Span<double> actuated_tensions)
    {
        int[] map = robotDescription.SolverToMotorMap;

        for (int actuatedCableIndex = 0; actuatedCableIndex < map.Length; actuatedCableIndex++)
        {
            actuated_tensions[actuatedCableIndex] = motorTensions[map[actuatedCableIndex]];
        }
    }



    private static double4x4 ToDouble4x4(in Matrix4x4 m)
    {
        return new double4x4(
            m.m00, m.m01, m.m02, m.m03,
            m.m10, m.m11, m.m12, m.m13,
            m.m20, m.m21, m.m22, m.m23,
            m.m30, m.m31, m.m32, m.m33
        );
    }

    // private bool InitializeTcpForActuation()
    // {
    //     if (!isLabviewControlEnabled)
    //     {
    //         return false;
    //     }
    //     if (tcpCommunicator == null)
    //     {
    //         Debug.LogWarning("RobotController: TCP communicator is unavailable. Robot commands will be skipped; tracker logging will continue.", this);
    //         return false;
    //     }
    //     try
    //     {
    //         if (!tcpCommunicator.Initialize())
    //         {
    //             Debug.LogWarning("RobotController: TCP communicator initialization failed. Robot commands will be skipped; tracker logging will continue.", this);
    //             return false;
    //         }

    //         tcpCommunicator.ConnectToServer();
    //         return true;
    //     }
    //     catch (Exception ex)
    //     {
    //         Debug.LogWarning($"RobotController: TCP setup failed ({ex.Message}). Robot commands will be skipped; tracker logging will continue.", this);
    //         return false;
    //     }
    // }
    
    private bool CanSendTcpCommands()
    {
        return tcpAvailable && tcpCommunicator != null && tcpCommunicator.IsConnected; 
    }

    private bool ValidateModules()
    {
        return trackerManager &&
               forcePlateManager &&
               tcpCommunicator &&
               visualizer;
    }

    private void OnDestroy()
    {
        isRunning = false;
        tcpCommunicator?.Disconnect();

        if (dataLogger != null && dataLogger.FrameCount > 0)
            dataLogger.WriteToDisk(sessionName);
    }
}