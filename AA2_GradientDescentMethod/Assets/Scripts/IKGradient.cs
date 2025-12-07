using UnityEngine;
using System.Collections.Generic;

public class IKGradient : MonoBehaviour
{
    [Header("Joint Configuration")]
    [SerializeField] private Transform rootJoint;
    [SerializeField] private List<Transform> joints = new();
    [SerializeField] private Transform endEffector;
    [SerializeField] private Transform target;

    [Header("Optimization Parameters")]
    [SerializeField] private float alpha = 0.1f;
    [SerializeField] private float tolerance = 0.01f;

    private float costFunction;
    private float[] angles; // [theta_x, theta_y, theta_z] x joint
    private float[] linkLengths;
    private Vector3[] linkDirections;
    private int totalDOF;

    private Vector3[] initialPositions;

    private void Awake() { InitializeIK(); }

    void InitializeIK()
    {
        if (joints.Count == 0)
        {
            Debug.LogError("No joints assigned!");
            return;
        }

        totalDOF = joints.Count * 3;

        angles = new float[totalDOF];
        linkLengths = new float[joints.Count];
        linkDirections = new Vector3[joints.Count];
        initialPositions = new Vector3[joints.Count];

        for (int i = 0; i < joints.Count; i++) { initialPositions[i] = joints[i].position; }
        for (int i = 0; i < joints.Count; i++)
        {
            Vector3 startPos = (i == 0) ? rootJoint.position : initialPositions[i - 1];
            Vector3 endPos = (i < joints.Count - 1) ? initialPositions[i] : endEffector.position;

            Vector3 linkVector = endPos - startPos;
            linkLengths[i] = linkVector.magnitude;
            linkDirections[i] = linkVector.sqrMagnitude > 0.00001f ? linkVector.normalized : Vector3.right;
        }

        for (int i = 0; i < totalDOF; i++) { angles[i] = 0f; }

        costFunction = CalculateCost(angles);

        Debug.Log($"IK Initialized with {joints.Count} joints, total DOF: {totalDOF}");
    }

    void Update()
    {
        if (joints.Count == 0 || target == null) { return; }

        if (costFunction > tolerance)
        {
            float[] gradient = CalculateGradient();

            // Gradient Method simple
            for (int i = 0; i < totalDOF; i++)
                angles[i] -= alpha * gradient[i];

            ForwardKinematics();
        }

        costFunction = CalculateCost(angles);
    }

    // ------------------------
    // COST FUNCTION
    // ------------------------

    float CalculateCost(float[] theta)
    {
        Vector3 endEffectorPos = GetEndEffectorPosition(theta);
        float distance = Vector3.Distance(endEffectorPos, target.position);
        return distance * distance;
    }

    // ------------------------
    // IK GRADIENT CALCULATION
    // ------------------------

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

    // ------------------------
    // FORWARD KINEMATICS
    // ------------------------

    Vector3 GetEndEffectorPosition(float[] p_theta)
    {
        Vector3 currentPos = rootJoint.position;

        for (int i = 0; i < joints.Count; i++)
        {
            int baseIdx = i * 3;

            Vector3 localRotationDelta = new(
                p_theta[baseIdx] * Mathf.Rad2Deg,
                p_theta[baseIdx + 1] * Mathf.Rad2Deg,
                p_theta[baseIdx + 2] * Mathf.Rad2Deg
            );

            Quaternion rotationDelta = Quaternion.Euler(localRotationDelta);
            Vector3 rotatedDirection = rotationDelta * linkDirections[i];

            currentPos += rotatedDirection * linkLengths[i];
        }

        return currentPos;
    }

    void ForwardKinematics()
    {
        Vector3 currentPos = rootJoint.position;

        for (int i = 0; i < joints.Count; i++)
        {
            int baseIdx = i * 3;

            Vector3 localRotationDelta = new(
                angles[baseIdx] * Mathf.Rad2Deg,
                angles[baseIdx + 1] * Mathf.Rad2Deg,
                angles[baseIdx + 2] * Mathf.Rad2Deg
            );

            Quaternion rotationDelta = Quaternion.Euler(localRotationDelta);
            Vector3 rotatedDirection = rotationDelta * linkDirections[i];

            Vector3 nextPos = currentPos + rotatedDirection * linkLengths[i];

            joints[i].position = nextPos;
            //joints[i].rotation = initialRotations[i] * rotationDelta;

            // rotate joint towards previous joint
            Vector3 targetLookPos = (i > 0) ? joints[i - 1].position : rootJoint.position;
            Vector3 directionToPrev = (targetLookPos - joints[i].position).normalized;
            if (directionToPrev.sqrMagnitude > 0.00001f) { joints[i].rotation = Quaternion.LookRotation(directionToPrev, Vector3.right) * Quaternion.Euler(0, -90, 0); }

            currentPos = nextPos;
        }

        endEffector.position = currentPos;
    }

    // ------------------------
    // GIZMOS
    // ------------------------

    void OnDrawGizmos()
    {
        if (joints.Count == 0 || rootJoint == null) { return; }

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