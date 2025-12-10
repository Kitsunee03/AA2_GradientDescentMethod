using UnityEngine;
using UnityEngine.InputSystem;

public class CameraController : MonoBehaviour
{
    [SerializeField] private GameObject m_target;

    [Header("Movement")]
    [SerializeField, Min(0)] private Vector2 distanceLimits = new(5f, 10f);
    [SerializeField, Min(0)] private float m_cameraLerp = 20f;
    private float m_targetDistance;
    private float rotationX, rotationY;

    private void Awake()
    {
        m_targetDistance = distanceLimits.y / 2;
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
    }

    private void LateUpdate()
    {
        HandleRotation();
        HandleMovement();

        HandleZoom();
    }

    private void HandleRotation()
    {
        // mouse movement
        Vector2 mouseDelta = Mouse.current.delta.ReadValue();
        rotationX -= mouseDelta.y * 0.1f;
        rotationY += mouseDelta.x * 0.1f;

        rotationX = Mathf.Clamp(rotationX, -40, 50f);
        transform.eulerAngles = new(rotationX, rotationY, 0);
    }

    private void HandleMovement()
    {
        transform.position = Vector3.Lerp(
            transform.position,
            m_target.transform.position - transform.forward * m_targetDistance,
            m_cameraLerp * Time.deltaTime
        );
    }

    private void HandleZoom()
    {
        Mouse mouse = Mouse.current;
        if (mouse == null) { return; }

        float scroll = mouse.scroll.ReadValue().y;
        if (Mathf.Approximately(scroll, 0f)) { return; }

        const float zoomSensitivity = 1.5f; // tweak to taste
        m_targetDistance -= scroll * zoomSensitivity;
        m_targetDistance = Mathf.Clamp(m_targetDistance, distanceLimits.x, distanceLimits.y);
    }
}