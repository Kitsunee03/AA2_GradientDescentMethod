using UnityEngine;
using System.Collections.Generic;

public class IKGradient3D : MonoBehaviour
{
    [Header("Joint Configuration")]
    public Transform rootJoint;
    public List<Transform> joints = new List<Transform>();
    public Transform endEffector;
    public Transform target;

    [Header("Optimization Parameters")]
    public float alpha = 0.1f;
    public float tolerance = 0.01f;
    
    [Header("Joint Constraints")]
    [Tooltip("Maximum rotation in degrees for each axis (same for all joints)")]
    public float maxRotationDegrees = 45f;

    private float costFunction;
    private float[] angles; // Now stores [theta_x, theta_y, theta_z] for each joint
    private float[] linkLengths;
    private Vector3[] linkDirections; // Initial direction of each link
    private int totalDOF; // Degrees of freedom (3 * number of joints)
    
    // Store initial rotations for local constraints
    private Quaternion[] initialRotations;
    private Vector3[] initialPositions;

    // Adam optimizer parameters
    private float beta1 = 0.9f;
    private float beta2 = 0.999f;
    private float epsilon = 1e-8f;
    private int t = 1;
    private float[] m_t;
    private float[] v_t;

    void Start()
    {
        InitializeIK();
    }

    void InitializeIK()
    {
        if (joints.Count == 0)
        {
            Debug.LogError("No joints assigned!");
            return;
        }

        // Calculate total degrees of freedom (3 angles per joint)
        totalDOF = joints.Count * 3;
        
        // Initialize arrays
        angles = new float[totalDOF];
        m_t = new float[totalDOF];
        v_t = new float[totalDOF];
        linkLengths = new float[joints.Count];
        linkDirections = new Vector3[joints.Count];
        initialRotations = new Quaternion[joints.Count];
        initialPositions = new Vector3[joints.Count];

        // Store initial rotations and positions
        for (int i = 0; i < joints.Count; i++)
        {
            initialRotations[i] = joints[i].rotation;
            initialPositions[i] = joints[i].position;
        }

        // Calculate link lengths and directions based on initial positions
        for (int i = 0; i < joints.Count; i++)
        {
            Vector3 startPos = (i == 0) ? rootJoint.position : initialPositions[i - 1];
            Vector3 endPos = (i < joints.Count - 1) ? initialPositions[i] : endEffector.position;
            
            Vector3 linkVector = endPos - startPos;
            linkLengths[i] = linkVector.magnitude;
            linkDirections[i] = linkVector.normalized;
            
            if (linkLengths[i] < 0.001f)
            {
                Debug.LogWarning($"Joint {i} has very small link length. Check initial positions.");
                linkDirections[i] = Vector3.right;
            }
        }

        // Initialize angles to zero (relative to initial pose)
        for (int i = 0; i < totalDOF; i++)
        {
            angles[i] = 0f;
        }

        // Initialize cost function
        costFunction = CalculateCost(angles);
        
        Debug.Log($"IK Initialized with {joints.Count} joints, total DOF: {totalDOF}");
    }
    
    float NormalizeAngle(float angle)
    {
        // Convert angle to -180 to 180 range
        angle = angle % 360f;
        if (angle > 180f)
            angle -= 360f;
        else if (angle < -180f)
            angle += 360f;
        return angle;
    }

    void Update()
    {
        if (joints.Count == 0 || target == null) return;

        if (costFunction > tolerance)
        {
            float[] gradient = CalculateGradient();
            float[] adaptiveStep = AdaptiveLearningRate(gradient);

            // Update angles
            for (int i = 0; i < totalDOF; i++)
            {
                angles[i] -= adaptiveStep[i];
            }

            // Apply constraints
            ApplyConstraints();

            // Update joint positions
            ForwardKinematics();
        }

        costFunction = CalculateCost(angles);
    }

    void ApplyConstraints()
    {
        float maxRotationRad = maxRotationDegrees * Mathf.Deg2Rad;
        
        for (int i = 0; i < joints.Count; i++)
        {
            int baseIdx = i * 3;
            
            // Constrain all rotations to the same range
            angles[baseIdx] = Mathf.Clamp(angles[baseIdx], -maxRotationRad, maxRotationRad);
            angles[baseIdx + 1] = Mathf.Clamp(angles[baseIdx + 1], -maxRotationRad, maxRotationRad);
            angles[baseIdx + 2] = Mathf.Clamp(angles[baseIdx + 2], -maxRotationRad, maxRotationRad);
        }
    }

    float[] AdaptiveLearningRate(float[] gradient)
    {
        t++;
        float[] adaptiveAlpha = new float[totalDOF];

        for (int i = 0; i < totalDOF; i++)
        {
            m_t[i] = beta1 * m_t[i] + (1 - beta1) * gradient[i];
            v_t[i] = beta2 * v_t[i] + (1 - beta2) * gradient[i] * gradient[i];

            float m_hat = m_t[i] / (1 - Mathf.Pow(beta1, t));
            float v_hat = v_t[i] / (1 - Mathf.Pow(beta2, t));

            adaptiveAlpha[i] = alpha * m_hat / (Mathf.Sqrt(v_hat) + epsilon);
        }

        return adaptiveAlpha;
    }

    float CalculateCost(float[] theta)
    {
        Vector3 endEffectorPos = GetEndEffectorPosition(theta);
        float distance = Vector3.Distance(endEffectorPos, target.position);
        return distance * distance;
    }

    float[] CalculateGradient()
    {
        float[] gradient = new float[totalDOF];
        float step = 0.0001f;
        float baseCost = CalculateCost(angles);

        for (int i = 0; i < totalDOF; i++)
        {
            float[] thetaPlus = (float[])angles.Clone();
            thetaPlus[i] += step;

            float costPlus = CalculateCost(thetaPlus);
            gradient[i] = (costPlus - baseCost) / step;
        }

        return gradient;
    }

    Vector3 GetEndEffectorPosition(float[] theta)
    {
        // Start from root position
        Vector3 currentPos = rootJoint.position;
        
        for (int i = 0; i < joints.Count; i++)
        {
            int baseIdx = i * 3;
            
            // Get local rotation delta from initial pose
            Vector3 localRotationDelta = new Vector3(
                theta[baseIdx] * Mathf.Rad2Deg,
                theta[baseIdx + 1] * Mathf.Rad2Deg,
                theta[baseIdx + 2] * Mathf.Rad2Deg
            );
            
            // Apply rotation delta to the initial link direction
            Quaternion rotationDelta = Quaternion.Euler(localRotationDelta);
            Vector3 rotatedDirection = rotationDelta * linkDirections[i];
            
            // Move along the rotated direction
            currentPos += rotatedDirection * linkLengths[i];
        }

        return currentPos;
    }

    Vector3 GetDirectionFromAngles(Vector3 angles)
    {
        // This method is kept for compatibility but updated
        Quaternion rotation = Quaternion.Euler(
            angles.x * Mathf.Rad2Deg,
            angles.y * Mathf.Rad2Deg,
            angles.z * Mathf.Rad2Deg
        );
        
        return rotation * Vector3.right;
    }

    void ForwardKinematics()
    {
        // Start from root position
        Vector3 currentPos = rootJoint.position;

        for (int i = 0; i < joints.Count; i++)
        {
            int baseIdx = i * 3;
            
            // Get local rotation delta
            Vector3 localRotationDelta = new Vector3(
                angles[baseIdx] * Mathf.Rad2Deg,
                angles[baseIdx + 1] * Mathf.Rad2Deg,
                angles[baseIdx + 2] * Mathf.Rad2Deg
            );
            
            // Apply rotation delta to the initial link direction
            Quaternion rotationDelta = Quaternion.Euler(localRotationDelta);
            Vector3 rotatedDirection = rotationDelta * linkDirections[i];
            
            // Calculate new position
            Vector3 nextPos = currentPos + rotatedDirection * linkLengths[i];
            
            // Update joint position
            joints[i].position = nextPos;
            
            // Update joint rotation (initial rotation + delta)
            joints[i].rotation = initialRotations[i] * rotationDelta;
            
            // Move to next segment
            currentPos = nextPos;
        }

        // Update end effector
        endEffector.position = currentPos;
    }

    // Visualize the chain in the editor
    void OnDrawGizmos()
    {
        if (joints.Count == 0 || rootJoint == null) return;

        Gizmos.color = Color.yellow;
        Vector3 prevPos = rootJoint.position;

        foreach (Transform joint in joints)
        {
            if (joint != null)
            {
                Gizmos.DrawLine(prevPos, joint.position);
                Gizmos.DrawSphere(joint.position, 0.05f);
                prevPos = joint.position;
            }
        }

        if (endEffector != null)
        {
            Gizmos.DrawLine(prevPos, endEffector.position);
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(endEffector.position, 0.08f);
        }

        if (target != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(target.position, 0.1f);
        }
    }
}