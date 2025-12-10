using UnityEngine;
using System.Collections.Generic;

public class IKGradient : MonoBehaviour
{
    [Header("Joint Configuration")]
    [SerializeField] private Transform rootJoint;
    [SerializeField] private List<Transform> joints = new();
    [SerializeField] private Transform target;

    [Header("Optimization Parameters")]
    [SerializeField] private float alpha = 0.05f;
    [SerializeField] private float tolerance = 0.01f;
    [SerializeField] private float smoothingFactor = 0.15f; // change smoothing
    [SerializeField] private float gradientDamping = 0.8f;
    
    // rotation limits
    private const float collisionPenalty = 2.0f;
    private const float minDistanceBetweenJoints = 0.3f;

    private float costFunction;
    private float[] angles; // [theta_x, theta_y, theta_z] per joint
    private float[] angleVelocities, previousGradient,linkLengths;
    private Vector3[] linkDirections, initialPositions;
    private int totalDOF;

    private void Awake() { InitializeIK(); }

    private void InitializeIK()
    {
        if (joints.Count == 0)
        {
            Debug.LogError("No joints assigned!");
            return;
        }

        totalDOF = joints.Count * 3;

        angles = new float[totalDOF];
        angleVelocities = new float[totalDOF];
        previousGradient = new float[totalDOF];
        linkLengths = new float[joints.Count];
        linkDirections = new Vector3[joints.Count];
        initialPositions = new Vector3[joints.Count];

        for (int i = 0; i < joints.Count; i++) { initialPositions[i] = joints[i].position; }
        for (int i = 0; i < joints.Count; i++)
        {
            Vector3 startPos = (i == 0) ? rootJoint.position : initialPositions[i - 1];
            Vector3 endPos = initialPositions[i];

            Vector3 linkVector = endPos - startPos;
            linkLengths[i] = linkVector.magnitude;
            linkDirections[i] = linkVector.sqrMagnitude > 0.00001f ? linkVector.normalized : Vector3.right;
        }

        for (int i = 0; i < totalDOF; i++) 
        { 
            angles[i] = 0f;
            angleVelocities[i] = 0f;
            previousGradient[i] = 0f;
        }

        costFunction = CalculateCost(angles);
    }

    private void Update()
    {
        if (joints.Count == 0 || target == null) { return; }

        if (costFunction > tolerance)
        {
            float[] gradient = CalculateGradient();

            // apply gradient with smoothing and damping
            for (int i = 0; i < totalDOF; i++)
            {
                float deltaAngle = -alpha * gradient[i];
                angleVelocities[i] = Mathf.Lerp(angleVelocities[i], deltaAngle, smoothingFactor);
                angles[i] += angleVelocities[i] * gradientDamping;
            }

            ForwardKinematics();
        }

        costFunction = CalculateCost(angles);
    }

    // ------------------------
    // COST FUNCTION
    // ------------------------

    private float CalculateCost(float[] p_theta)
    {
        Vector3 endEffectorPos = GetEndEffectorPosition(p_theta);
        float distance = Vector3.Distance(endEffectorPos, target.position);
        float positionCost = distance * distance;
        
        // penalize collisions between joints
        float collisionCost = CalculateCollisionPenalty(p_theta);
        
        return positionCost + collisionPenalty * collisionCost;
    }

    private float CalculateCollisionPenalty(float[] p_theta)
    {
        float penalty = 0f;
        Vector3[] jointPositions = GetJointPositions(p_theta);
        
        for (int i = 0; i < jointPositions.Length; i++)
        {
            for (int j = i + 2; j < jointPositions.Length; j++) // ignore adjacent joints
            {
                float dist = Vector3.Distance(jointPositions[i], jointPositions[j]);
                
                // exponential penalty: stronger the closer they are
                if (dist < minDistanceBetweenJoints * 2f)
                {
                    float violation = minDistanceBetweenJoints - dist;
                    penalty += violation * violation * violation; // cubic for more aggressiveness
                }
            }
        }
        
        return penalty;
    }

    // ------------------------
    // IK GRADIENT CALCULATION
    // ------------------------

    private Vector3[] GetJointPositions(float[] p_theta)
    {
        Vector3[] positions = new Vector3[joints.Count];
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
            positions[i] = currentPos;
        }

        return positions;
    }

    private float[] CalculateGradient()
    {
        float[] gradient = new float[totalDOF];
        float step = 0.001f;

        float baseCost = CalculateCost(angles);

        for (int i = 0; i < totalDOF; i++)
        {
            float[] thetaPlus = (float[])angles.Clone();
            thetaPlus[i] += step;

            float costPlus = CalculateCost(thetaPlus);
            gradient[i] = (costPlus - baseCost) / step;
            
            // clamp gradients to avoid abrupt changes
            gradient[i] = Mathf.Clamp(gradient[i], -1f, 1f);
            
            // smooth with history for stability
            gradient[i] = Mathf.Lerp(previousGradient[i], gradient[i], 0.6f);
            previousGradient[i] = gradient[i];
        }

        return gradient;
    }

    // ------------------------
    // FORWARD KINEMATICS
    // ------------------------

    private Vector3 GetEndEffectorPosition(float[] p_theta)
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

    private void ForwardKinematics()
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

            // smooth position and rotation
            joints[i].position = Vector3.Lerp(joints[i].position, nextPos, 0.2f);
            
            // rotation to look at previous joint
            Vector3 targetLookPos = (i > 0) ? joints[i - 1].position : rootJoint.position;
            Vector3 directionToPrev = (targetLookPos - joints[i].position).normalized;
            
            if (directionToPrev.sqrMagnitude > 0.00001f) 
            { 
                Quaternion targetRotation = Quaternion.LookRotation(directionToPrev, Vector3.right) * Quaternion.Euler(0, -90, 0);
                joints[i].rotation = Quaternion.Lerp(joints[i].rotation, targetRotation, 0.15f);
            }

            currentPos = nextPos;
        }
    }

    // ------------------------
    // GIZMOS
    // ------------------------

    private void OnDrawGizmos()
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

        // Draw last joint as end effector in green
        if (joints.Count > 0 && joints[joints.Count - 1] != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(joints[joints.Count - 1].position, 0.08f);
        }

        if (target != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(target.position, 0.1f);
        }
    }
}