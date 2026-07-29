using UnityEngine;
using Unity.Mathematics;
using System;

public class RobotVisualizer : MonoBehaviour
{
    [Header("General Settings")]
    [Tooltip("Toggle to enable/disable Robot visualization.")]
    public bool isActive = true;
    [Tooltip("When enabled, show two additional coordinate frames for left/right end-effectors while keeping the barbell frame.")]
    public bool showIndividualEndEffectorFrames = false;

    [Header("Tracker Prefabs")]
    public Transform comTrackerVisual;
    public Transform endEffectorLVisual;
    public Transform endEffectorRVisual;
    public Transform frameTrackerVisual;
    
    [Header("Multi-Camera Setup")]
    [Tooltip("Top-Down view (Top Left). Manually placed.")]
    public Camera topViewCamera;
    [Tooltip("Side view (Left Half). Manually placed.")]
    public Camera sideViewCamera;
    [Tooltip("Back view view (Right Half). This one will track the robot center.")]
    public Camera backViewCamera;

    [Header("Barbell Visual")]
    [Tooltip("Cylinder radius in meters for the yellow barbell visual.")]
    public float barbellRadius = 0.02f;
    [Tooltip("Cylinder length in meters for the yellow barbell visual.")]
    public float barbellLength = 1.25f;
    [Tooltip("Local rotation offset applied to the cylinder after the barbell pose is tracked.")]
    public Vector3 barbellLocalEulerOffset = new Vector3(0f, 0f, 90f);
    private readonly Color BarbellColor = new Color(1.0f, 0.92f, 0.16f, 0.35f); // Transparent Yellow

    [Header("Force Plate Visual")]
    [Tooltip("Toggle display of the ground reaction forces.")]
    public bool showForceVisuals = true;

    [Header("Pulley Visual")]
    [Tooltip("Toggle display of the pulley positions.")]
    public bool showPulleyDebug = true;
    private GameObject pulleyDebugRoot;

    private float metersPerNewton = 0.002f;
    private float forceCapsuleRadius = 0.01f;

    // -- Goal Trajectory Constants --
    private const int GoalTrajSteps = 10;
    private const float GoalSphereDiameter = 0.1f;
    private readonly Color GoalTrajColor = new Color(0.0f, 1.0f, 0.0f, 0.4f); // Transparent Green

    // -- Internal State --
    private RobUSTDescription robot;
    private bool isInitialized = false;

    private Transform[] pulleySpheres;
    private Transform grf0Capsule;
    private Transform grf1Capsule;
    private Transform[] goalTrajectorySpheres;
    private Transform barbellRootVisual;
    private Transform barbellRodVisual;
    private Transform endEffectorLIndividualVisual;
    private Transform endEffectorRIndividualVisual;

    // -- Thread Safety --
    private readonly object dataLock = new object();

    // -- CACHED DATA --
    private double4x4 _comPose_Robot;
    private double4x4 _eeLPose_Robot;
    private double4x4 _eeRPose_Robot;
    private double3 _cop0_Robot;
    private double3 _grf0_Robot;
    private double3 _cop1_Robot;
    private double3 _grf1_Robot;

    // Trajectory Caches
    private double3[] _goalTrajectoryCache = new double3[GoalTrajSteps];

    public bool Initialize(RobUSTDescription robotDescription)
    {
        // Validation
        if (comTrackerVisual == null ||
            endEffectorLVisual == null ||
            endEffectorRVisual == null ||
            frameTrackerVisual == null || 
            backViewCamera == null ||
            topViewCamera == null ||
            sideViewCamera == null)
        {
            Debug.LogError("RobotVisualizer: Please assign all modules.");
            return false;
        }
        robot = robotDescription;
        StripPhysics(comTrackerVisual);
        StripPhysics(endEffectorLVisual);
        StripPhysics(endEffectorRVisual);
        StripPhysics(frameTrackerVisual);
        StripPhysics(backViewCamera.transform);
        StripPhysics(topViewCamera.transform);
        StripPhysics(sideViewCamera.transform);
        Destroy(topViewCamera.GetComponent<AudioListener>());
        Destroy(sideViewCamera.GetComponent<AudioListener>());
        Destroy(backViewCamera.GetComponent<AudioListener>());

        var root = new GameObject("Generated_Visuals").transform;
        root.SetParent(this.transform);

        grf0Capsule = CreateCapsule("Vis_GRF0", Color.cyan, root);
        grf1Capsule = CreateCapsule("Vis_GRF1", Color.cyan, root);
        GeneratePulleyVisuals(root);
        GenerateForcePlateVisual(root);
        GenerateGoalTrajectoryVisuals(root);
        GenerateBarbellVisual(root);
        GenerateIndividualEndEffectorVisuals(root);

        ConfigureSplitScreen();

        isInitialized = true;
        return true;
    }

    private void ConfigureSplitScreen()
    {
        // Screen Layout:
        // ----------------------+----------------------
        // |                      |                     |
        // | Side View from Left  |    Back View        |
        // |       (Left)         |     (Right)         |
        // |                      |                     |
        // |                      |                     |
        // ----------------------+----------------------
        float3 robust_center = new float3(0f, 0.7f, 0.2f);

        if (sideViewCamera != null)
            {
                // sideViewCamera.orthographic = true;
                // sideViewCamera.orthographicSize = 2.25f;
                // Left half of screen
                sideViewCamera.rect = new Rect(0.0f, 0.0f, 0.5f, 1.0f);
                float3 sidePos = robust_center + new float3(0f, -3.3f, 0f);

                sideViewCamera.transform.position = (Vector3)RobotToUnityPos(sidePos);
                sideViewCamera.transform.LookAt((Vector3)RobotToUnityPos(robust_center));
            }


            //
            // RIGHT PANEL: BACK VIEW
            //
            if (backViewCamera != null)
            {
                backViewCamera.orthographic = true;
                backViewCamera.orthographicSize = 2.25f;
                backViewCamera.rect = new Rect(0.5f, 0.0f, 0.5f, 1.0f);

                // Back view along robot X axis
                float3 backPos = robust_center + new float3(4.0f, 0f, 0f);
                float3 backViewCenter = robust_center + new float3(0f, 0.5f, 0f);


                backViewCamera.transform.position = (Vector3)RobotToUnityPos(backPos);
                backViewCamera.transform.LookAt((Vector3)RobotToUnityPos(backViewCenter), Vector3.up);
            }
}

    /// <summary>
    /// Updated API: Accepts double4x4 directly.
    /// </summary>
    public void PushState(in double4x4 comPose, in double4x4 eeLPose, in double4x4 eeRPose, in ForcePlateData fp0, in ForcePlateData fp1)
    {
        lock (dataLock)
        {
            _comPose_Robot = comPose;
            _eeLPose_Robot = eeLPose;
            _eeRPose_Robot = eeRPose;
            _cop0_Robot = fp0.CoP;
            _grf0_Robot = fp0.Force;
            _cop1_Robot = fp1.CoP;
            _grf1_Robot = fp1.Force;
        }
    }



    /// <summary>
    /// Pushes a goal trajectory to the visualizer.
    /// If span < 10, the last valid point is repeated for remaining spheres.
    /// If span >= 10, the first 10 points are used.
    /// </summary>
    public void PushGoalTrajectory(ReadOnlySpan<RBState> trajectory)
    {
        if (trajectory.IsEmpty) return;

        lock (dataLock)
        {
            int len = trajectory.Length;
            int copyCount = (len < GoalTrajSteps) ? len : GoalTrajSteps;

            // 1. Copy available points
            for (int i = 0; i < copyCount; i++)
                _goalTrajectoryCache[i] = trajectory[i].p;

            // 2. Clamp remaining if short trajectory
            if (len < GoalTrajSteps)
            {
                double3 lastP = trajectory[len - 1].p;
                for (int i = len; i < GoalTrajSteps; i++)
                    _goalTrajectoryCache[i] = lastP;
            }
        }
    }

    private void Update()
    {
        if (!isInitialized || !isActive) return;

        // Local cache vars
        double4x4 comMat, eeLMat, eeRMat;
        double3 cop0, grf0, cop1, grf1;
        Span<double3> goalSnapshot = stackalloc double3[GoalTrajSteps];

        lock (dataLock)
        {
            // Simple snapshotting - dump latest state
            comMat = _comPose_Robot;
            eeLMat = _eeLPose_Robot;
            eeRMat = _eeRPose_Robot;
            cop0 = _cop0_Robot;
            grf0 = _grf0_Robot;
            cop1 = _cop1_Robot;
            grf1 = _grf1_Robot;

            for (int i = 0; i < GoalTrajSteps; i++)
                goalSnapshot[i] = _goalTrajectoryCache[i];
        }

        // 1. Cast to float4x4 once (Visuals don't need double precision)
        float4x4 comF = (float4x4)comMat;
        float4x4 eeLF = (float4x4)eeLMat;
        float4x4 eeRF = (float4x4)eeRMat;
        
        comTrackerVisual.position = (Vector3)RobotToUnityPos(comF.c3.xyz);
        comTrackerVisual.rotation = (Quaternion)RobotToUnityRot(new quaternion(comF));

        double4x4 barbellPose = PoseFusion.BuildBarbellPose(eeLMat, eeRMat);
        float4x4 barbellF = (float4x4)barbellPose;
        quaternion barbellRot = new quaternion(new float3x3((float3)barbellF.c0.xyz, (float3)barbellF.c1.xyz, (float3)barbellF.c2.xyz));

        if (barbellRootVisual != null)
        {
            barbellRootVisual.position = (Vector3)RobotToUnityPos(barbellF.c3.xyz);
            barbellRootVisual.rotation = (Quaternion)RobotToUnityRot(barbellRot);
        }

        endEffectorLVisual.position = (Vector3)RobotToUnityPos(barbellF.c3.xyz);
        endEffectorLVisual.rotation = (Quaternion)RobotToUnityRot(barbellRot);

        
        endEffectorRVisual.position = (Vector3)RobotToUnityPos(barbellF.c3.xyz);
        endEffectorRVisual.rotation = (Quaternion)RobotToUnityRot(barbellRot);

        bool showIndividual = showIndividualEndEffectorFrames;
        if (endEffectorLIndividualVisual != null)
        {
            if (endEffectorLIndividualVisual.gameObject.activeSelf != showIndividual)
                endEffectorLIndividualVisual.gameObject.SetActive(showIndividual);
            if (showIndividual)
            {
                endEffectorLIndividualVisual.position = (Vector3)RobotToUnityPos(eeLF.c3.xyz);
                endEffectorLIndividualVisual.rotation = (Quaternion)RobotToUnityRot(new quaternion(eeLF));
            }
        }

        if (endEffectorRIndividualVisual != null)
        {
            if (endEffectorRIndividualVisual.gameObject.activeSelf != showIndividual)
                endEffectorRIndividualVisual.gameObject.SetActive(showIndividual);
            if (showIndividual)
            {
                endEffectorRIndividualVisual.position = (Vector3)RobotToUnityPos(eeRF.c3.xyz);
                endEffectorRIndividualVisual.rotation = (Quaternion)RobotToUnityRot(new quaternion(eeRF));
            }
        }
        
        // Frame (Always Origin)
        frameTrackerVisual.localPosition = Vector3.zero; 
        frameTrackerVisual.localRotation = Quaternion.identity;

        if (showForceVisuals)
        {
            UpdateForceCapsule(grf0Capsule, RobotToUnityPos((float3)cop0), RobotToUnityPos((float3)grf0));
            UpdateForceCapsule(grf1Capsule, RobotToUnityPos((float3)cop1), RobotToUnityPos((float3)grf1));
        }
        else
        {
            grf0Capsule.localScale = Vector3.zero;
            grf1Capsule.localScale = Vector3.zero;
        }

        if (pulleyDebugRoot != null)
        {
            pulleyDebugRoot.SetActive(showPulleyDebug);
        }
        // Update Goal Trajectory
        for (int i = 0; i < GoalTrajSteps; i++)
            goalTrajectorySpheres[i].position = (Vector3)RobotToUnityPos((float3)goalSnapshot[i]);

    }

    // =========================================================
    // Helpers
    // =========================================================

    private static float3 RobotToUnityPos(float3 v)
    {
        return new float3(v.x, v.z, v.y);
    }

    private static quaternion RobotToUnityRot(quaternion q)
    {
        float3 fwd_r = math.rotate(q, new float3(0, 1, 0)); 
        float3 up_r  = math.rotate(q, new float3(0, 0, 1));

        float3 fwd_u = RobotToUnityPos(fwd_r);
        float3 up_u  = RobotToUnityPos(up_r);

        if (math.lengthsq(fwd_u) < 0.001f) return quaternion.identity;
        return quaternion.LookRotation(fwd_u, up_u);
    }

    private void UpdateForceCapsule(Transform capsule, float3 anchor, float3 forceVector)
    {
        float magnitude = math.length(forceVector);
        if (magnitude < 0.1f)
        {
            capsule.localScale = Vector3.zero;
            return;
        }

        float3 dir = forceVector / magnitude;
        float length = magnitude * metersPerNewton;

        capsule.rotation = Quaternion.FromToRotation(Vector3.up, (Vector3)dir);
        capsule.position = (Vector3)(anchor + (dir * (length * 0.5f)));
        capsule.localScale = new Vector3(forceCapsuleRadius * 2, length * 0.5f, forceCapsuleRadius * 2);
    }

    public void SetEndEffectorFramesVisible(bool visible)
    {
        showIndividualEndEffectorFrames = visible;
    }

    public void SetForceVisualsVisible(bool visible)
    {
        showForceVisuals = visible;
        if (!visible)
        {
            if (grf0Capsule != null) grf0Capsule.localScale = Vector3.zero;
            if (grf1Capsule != null) grf1Capsule.localScale = Vector3.zero;
        }
    }

    private Transform CreateCapsule(string name, Color c, Transform parent)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        go.name = name;
        go.transform.SetParent(parent);
        Destroy(go.GetComponent<Collider>());
        go.GetComponent<Renderer>().material.color = c;
        go.transform.localScale = Vector3.zero;
        return go.transform;
    }

    private void GeneratePulleyVisuals(Transform root)
    {
        pulleyDebugRoot = new GameObject("PulleyDebugVisuals");
        pulleyDebugRoot.transform.SetParent(root);
        
        pulleySpheres = new Transform[robot.FramePulleyPositions.Length];
        
        for (int i = 0; i < robot.FramePulleyPositions.Length; i++)
        {
            var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.name = $"Pulley_{i}";

            sphere.transform.SetParent(pulleyDebugRoot.transform);
            sphere.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
            
            Destroy(sphere.GetComponent<Collider>());
            sphere.GetComponent<Renderer>().material.color = Color.gray;

            sphere.transform.position = (Vector3)RobotToUnityPos((float3)robot.FramePulleyPositions[i]);
            pulleySpheres[i] = sphere.transform;
        }
        pulleyDebugRoot.SetActive(showPulleyDebug);
    }
    
    private void GenerateForcePlateVisual(Transform root)
    {
        // Calculate center and orientation from corners in Description
        var pBL = robot.FP_BackLeft;
        var pBR = robot.FP_BackRight;
        var pFL = robot.FP_FrontLeft;
        var pFR = robot.FP_FrontRight;

        double3 center = (pBL + pBR + pFL + pFR) * 0.25;
        
        // Approximate basis vectors for visualization
        double3 forward = math.normalize((pFL - pBL) + (pFR - pBR)); // Approx Y usually
        double3 right = math.normalize((pBR - pBL) + (pFR - pFL));   // Approx X usually
        double3 up = math.cross(forward, right); // Z?
        
        // Dimensions
        float length = (float)math.length((pFL - pBL) + (pFR - pBR)) * 0.5f;
        float width = (float)math.length((pBR - pBL) + (pFR - pFL)) * 0.5f;
        float thickness = 0.05f;

        var cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.name = "ForcePlate_Surface";
        cube.transform.SetParent(root);
        Destroy(cube.GetComponent<Collider>());
        
        cube.transform.position = (Vector3)RobotToUnityPos((float3)center - (float3)up * (thickness * 0.5f)); // Shift down so top surface is at 0
        quaternion rotRobot = quaternion.LookRotation((float3)up, (float3)forward); 
        cube.transform.rotation = (Quaternion)RobotToUnityRot(rotRobot);

        cube.transform.localScale = new Vector3(width, thickness, length);

        var ren = cube.GetComponent<Renderer>();
        ren.material.color = new Color(0.3f, 0.3f, 0.3f, 0.5f); // Transparent Grey
    }

    private void GenerateGoalTrajectoryVisuals(Transform root)
    {
        goalTrajectorySpheres = new Transform[GoalTrajSteps];
        var trajRoot = new GameObject("Goal_Trajectory").transform;
        trajRoot.SetParent(root);

        // Attempt URP Unlit (fastest/cleanest), fallback to Standard
        var shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Standard");
        var mat = new Material(shader);

        // Setup Transparency (Works for both usually)
        mat.SetFloat("_Surface", 1); // URP
        mat.SetFloat("_Mode", 3);    // Standard
        mat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
        mat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        mat.SetInt("_ZWrite", 0);
        mat.EnableKeyword("_ALPHABLEND_ON");
        mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT"); 
        mat.renderQueue = 3000;

        // Set Color (URP uses _BaseColor, Standard uses _Color)
        mat.SetColor("_BaseColor", GoalTrajColor); 
        mat.SetColor("_Color", GoalTrajColor);

        for (int i = 0; i < GoalTrajSteps; i++)
        {
            var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.name = $"Goal_Step_{i}";
            sphere.transform.SetParent(trajRoot);
            
            Destroy(sphere.GetComponent<Collider>());
            sphere.GetComponent<Renderer>().sharedMaterial = mat; 
            sphere.transform.localScale = Vector3.one * GoalSphereDiameter;
            sphere.transform.position = Vector3.zero;
            goalTrajectorySpheres[i] = sphere.transform;
        }
    }

    private void GenerateIndividualEndEffectorVisuals(Transform root)
    {
        endEffectorLIndividualVisual = Instantiate(endEffectorLVisual, root);
        endEffectorLIndividualVisual.name = "EndEffectorL_Individual";
        StripPhysics(endEffectorLIndividualVisual);
        endEffectorLIndividualVisual.gameObject.SetActive(false);

        endEffectorRIndividualVisual = Instantiate(endEffectorRVisual, root);
        endEffectorRIndividualVisual.name = "EndEffectorR_Individual";
        StripPhysics(endEffectorRIndividualVisual);
        endEffectorRIndividualVisual.gameObject.SetActive(false);
    }

    private void GenerateBarbellVisual(Transform root)
    {
        barbellRootVisual = new GameObject("Barbell_Root").transform;
        barbellRootVisual.SetParent(root); 

        var cylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        cylinder.name = "Barbell_Rod"; 
        cylinder.transform.SetParent(barbellRootVisual);
        Destroy(cylinder.GetComponent<Collider>());

        // Attempt URP Unlit (fastest/cleanest), fallback to Standard
        var shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Standard");
        var mat = new Material(shader);

        // Setup Transparency (Works for both usually)
        mat.SetFloat("_Surface", 1); // URP
        mat.SetFloat("_Mode", 3);    // Standard
        mat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
        mat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        mat.SetInt("_ZWrite", 0);
        mat.EnableKeyword("_ALPHABLEND_ON");
        mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT"); 
        mat.renderQueue = 3000;

        var renderer = cylinder.GetComponent<Renderer>();
        renderer.sharedMaterial = mat;

        mat.SetColor("_BaseColor", BarbellColor);
        mat.SetColor("_Color", BarbellColor);
        cylinder.transform.localPosition = Vector3.zero;
        cylinder.transform.localRotation = Quaternion.Euler(barbellLocalEulerOffset); 
        cylinder.transform.localScale = new Vector3(barbellRadius * 2f, barbellLength * 0.5f, barbellRadius * 2f);
    
        barbellRodVisual = cylinder.transform;
    }

    private void StripPhysics(Transform root)
    {
        // Remove Rigidbodies (Start from root, usually only one RB exists)
        foreach (var rb in root.GetComponentsInChildren<Rigidbody>())
        {
            Destroy(rb);
        }

        // Remove Colliders (Recursively)
        foreach (var col in root.GetComponentsInChildren<Collider>())
        {
            Destroy(col);
        }
    }
}