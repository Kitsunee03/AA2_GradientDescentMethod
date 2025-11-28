using UnityEngine;
using System.Collections.Generic;

public class IKGradientDynamic : MonoBehaviour
{
    [Header("Arm Joints")]
    [SerializeField] private List<Transform> joints;
    [SerializeField] private Transform target;

    [Header("IK Parameters")]
    [SerializeField] private float alpha = 0.08f;
    [SerializeField] private float smoothing = 0.15f;
    [SerializeField] private int iterationsPerFrame = 3;
    [SerializeField] private float endEffectorBias = 2.5f;
    private const float tolerance = 0.001f;

    [Header("Joint Limits (degrees, relative to initial rotation)")]
    [Tooltip("Límites de movimiento relativo desde la rotación inicial")]
    [SerializeField] private Vector2 relativePitchLimits = new Vector2(-60f, 60f);
    [SerializeField] private Vector2 relativeYawLimits = new Vector2(-60f, 60f);

    private float[] theta;
    private float[] thetaSmooth;
    private float[] thetaInitial; // Ángulos iniciales como referencia
    private float[] lengths;
    private float costFunction;

    private void Awake()
    {
        int n = joints.Count - 1;
        theta = new float[n * 2];
        thetaSmooth = new float[n * 2];
        thetaInitial = new float[n * 2]; // Guardar rotaciones iniciales
        lengths = new float[n];

        // Calcular longitudes originales
        for (int i = 0; i < n; i++)
        {
            lengths[i] = Vector3.Distance(joints[i].position, joints[i + 1].position);
        }

        // Guardar rotaciones iniciales y usar como punto de partida
        for (int i = 0; i < n; i++)
        {
            float pitchDeg = joints[i].localEulerAngles.x;
            float yawDeg = joints[i].localEulerAngles.y;

            // Convertir a radianes con signo
            float pitch = Mathf.Deg2Rad * Mathf.DeltaAngle(0f, pitchDeg);
            float yaw = Mathf.Deg2Rad * Mathf.DeltaAngle(0f, yawDeg);

            // Guardar como inicial
            thetaInitial[i * 2] = pitch;
            thetaInitial[i * 2 + 1] = yaw;

            // Empezar desde la pose inicial
            theta[i * 2] = pitch;
            theta[i * 2 + 1] = yaw;
            thetaSmooth[i * 2] = pitch;
            thetaSmooth[i * 2 + 1] = yaw;
        }

        costFunction = Cost(theta);
    }

    private void Update()
    {
        for (int i = 0; i < iterationsPerFrame; i++) { HandleJointLocations(); }

        SmoothTheta();
        HandleJointRotations();
    }

    private void HandleJointLocations()
    {
        if (costFunction > tolerance)
        {
            float[] grad = CalculateGradient(theta);

            // Ponderar gradientes - más peso a joints cerca del end-effector
            int numJoints = joints.Count - 1;
            for (int i = 0; i < numJoints; i++)
            {
                float weight = 1f + (endEffectorBias - 1f) * (i / (float)(numJoints - 1));
                grad[i * 2] *= weight;
                grad[i * 2 + 1] *= weight;
            }

            // Normalizar el gradiente
            float gradMagnitude = 0f;
            for (int i = 0; i < grad.Length; i++) { gradMagnitude += grad[i] * grad[i]; }
            gradMagnitude = Mathf.Sqrt(gradMagnitude);

            if (gradMagnitude > 1e-8f)
            {
                float normFactor = 1f / gradMagnitude;
                for (int i = 0; i < grad.Length; i++) { grad[i] *= normFactor; }
            }

            // Gradient descent
            float maxStep = 0.015f;
            for (int i = 0; i < theta.Length; i++)
            {
                float step = -alpha * grad[i];
                step = Mathf.Clamp(step, -maxStep, maxStep);
                theta[i] += step;
            }

            // Aplicar límites RELATIVOS a la rotación inicial
            for (int i = 0; i < numJoints; i++)
            {
                int pitchIdx = i * 2;
                int yawIdx = i * 2 + 1;

                // Obtener límites para este joint
                Vector2 pitchLim = relativePitchLimits;
                Vector2 yawLim = relativeYawLimits;

                // Calcular ángulo relativo a la inicial
                float pitchRad = theta[pitchIdx];
                float yawRad = theta[yawIdx];

                float pitchInitial = thetaInitial[pitchIdx];
                float yawInitial = thetaInitial[yawIdx];

                // Convertir a offset en grados desde inicial
                float pitchOffset = Mathf.DeltaAngle(pitchInitial * Mathf.Rad2Deg, pitchRad * Mathf.Rad2Deg);
                float yawOffset = Mathf.DeltaAngle(yawInitial * Mathf.Rad2Deg, yawRad * Mathf.Rad2Deg);

                // Aplicar límites al offset
                pitchOffset = Mathf.Clamp(pitchOffset, pitchLim.x, pitchLim.y);
                yawOffset = Mathf.Clamp(yawOffset, yawLim.x, yawLim.y);

                // Convertir de vuelta a ángulo absoluto
                theta[pitchIdx] = (pitchInitial * Mathf.Rad2Deg + pitchOffset) * Mathf.Deg2Rad;
                theta[yawIdx] = (yawInitial * Mathf.Rad2Deg + yawOffset) * Mathf.Deg2Rad;
            }

            // Normalizar ángulos a [-pi, pi]
            for (int i = 0; i < theta.Length; i++) { theta[i] = Mathf.Atan2(Mathf.Sin(theta[i]), Mathf.Cos(theta[i])); }
        }

        costFunction = Cost(theta);
    }

    private void SmoothTheta()
    {
        for (int i = 0; i < theta.Length; i++)
        {
            float diff = Mathf.DeltaAngle(thetaSmooth[i] * Mathf.Rad2Deg, theta[i] * Mathf.Rad2Deg);
            thetaSmooth[i] += diff * Mathf.Deg2Rad * smoothing;
            thetaSmooth[i] = Mathf.Atan2(Mathf.Sin(thetaSmooth[i]), Mathf.Cos(thetaSmooth[i]));
        }
    }

    private void HandleJointRotations()
    {
        Quaternion rotation = Quaternion.identity;
        Vector3 pos = joints[0].position;

        for (int i = 0; i < joints.Count - 1; i++)
        {
            float pitch = thetaSmooth[i * 2];
            float yaw = thetaSmooth[i * 2 + 1];

            Quaternion jointRotation = Quaternion.Euler(pitch * Mathf.Rad2Deg, yaw * Mathf.Rad2Deg, 0f);
            joints[i].rotation = jointRotation;
            rotation *= jointRotation;

            Vector3 targetPos = pos + rotation * Vector3.forward * lengths[i];
            joints[i + 1].position = targetPos;
            pos = targetPos;
        }
    }

    #region IK Core
    private float Cost(float[] p_theta)
    {
        Vector3 endEffector = ComputeEndEffector(p_theta);
        float distanceCost = Vector3.SqrMagnitude(endEffector - target.position);

        float angleCost = 0f;
        for (int i = 0; i < p_theta.Length; i++) { angleCost += p_theta[i] * p_theta[i]; }

        // Penalty por acercarse a los límites (soft constraint)
        float limitPenalty = 0f;
        int numJoints = joints.Count - 1;
        for (int i = 0; i < numJoints; i++)
        {
            Vector2 pitchLim = relativePitchLimits;
            Vector2 yawLim = relativeYawLimits;

            float pitchOffset = Mathf.DeltaAngle(thetaInitial[i * 2] * Mathf.Rad2Deg, p_theta[i * 2] * Mathf.Rad2Deg);
            float yawOffset = Mathf.DeltaAngle(thetaInitial[i * 2 + 1] * Mathf.Rad2Deg, p_theta[i * 2 + 1] * Mathf.Rad2Deg);

            // Penalizar si está cerca de los límites (10 grados de margen)
            if (pitchOffset < pitchLim.x + 10f || pitchOffset > pitchLim.y - 10f) { limitPenalty += 0.1f; }
            if (yawOffset < yawLim.x + 10f || yawOffset > yawLim.y - 10f) { limitPenalty += 0.1f; }
        }

        return distanceCost + 0.01f * angleCost + limitPenalty;
    }

    private float[] CalculateGradient(float[] p_theta)
    {
        float h = 0.001f;
        float baseCost = Cost(p_theta);
        float[] grad = new float[p_theta.Length];
        float[] temp = new float[p_theta.Length];

        for (int i = 0; i < p_theta.Length; i++)
        {
            p_theta.CopyTo(temp, 0);
            temp[i] += h;
            grad[i] = (Cost(temp) - baseCost) / h;
        }

        return grad;
    }

    private Vector3 ComputeEndEffector(float[] p)
    {
        Vector3 pos = joints[0].position;
        Quaternion rotation = Quaternion.identity;

        for (int i = 0; i < joints.Count - 1; i++)
        {
            float pitch = p[i * 2];
            float yaw = p[i * 2 + 1];
            Quaternion jointRotation = Quaternion.Euler(pitch * Mathf.Rad2Deg, yaw * Mathf.Rad2Deg, 0f);
            rotation *= jointRotation;

            pos += rotation * Vector3.forward * lengths[i];
        }

        return pos;
    }
    #endregion

    // Visualización de límites en Scene view
    private void OnDrawGizmos()
    {
        if (joints == null || joints.Count < 2) { return; }

        int n = joints.Count - 1;
        float[] debugThetaInitial = new float[n*2];

        for (int i = 0; i < n; i++)
        {
            float pitchDeg = joints[i].localEulerAngles.x;
            float yawDeg = joints[i].localEulerAngles.y;

            // Convertir a radianes con signo
            float pitch = Mathf.Deg2Rad * Mathf.DeltaAngle(0f, pitchDeg);
            float yaw = Mathf.Deg2Rad * Mathf.DeltaAngle(0f, yawDeg);

            // Guardar como inicial
            debugThetaInitial[i * 2] = pitch;
        }

        // Dibujar joints
        Gizmos.color = Color.yellow;
        for (int i = 0; i < n; i++)
        {
            if (joints[i] != null) { Gizmos.DrawWireSphere(joints[i].position, 0.05f); }
        }

        // Dibujar zona de límites del primer joint (solo en play mode)
        for (int i = 0; i < n; i++)
        {
            Transform joint0 = joints[i];
            Vector2 yawLim = relativeYawLimits;

            float yawInitial = debugThetaInitial[i*2] * Mathf.Rad2Deg;
            float yawMin = yawInitial + yawLim.x;
            float yawMax = yawInitial + yawLim.y;

            // Dibujar arco de límite yaw
            Gizmos.color = new(0, 1, 0, 0.3f);
            Vector3 dirMin = Quaternion.Euler(0, yawMin, 0) * Vector3.forward;
            Vector3 dirMax = Quaternion.Euler(0, yawMax, 0) * Vector3.forward;
            Gizmos.DrawLine(joint0.position, joint0.position + dirMin * 0.5f);
            Gizmos.DrawLine(joint0.position, joint0.position + dirMax * 0.5f);
        }
    }
}