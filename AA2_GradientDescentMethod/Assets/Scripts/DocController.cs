using UnityEngine;
using UnityEngine.InputSystem;

public class DocController : MonoBehaviour
{
    [SerializeField] private Camera mainCamera;

    [Header("Movement Settings")]
    [SerializeField] private float movementSpeed = 5f;
    private Keyboard kb;

    private void Awake() { kb = Keyboard.current; }

    private void Update()
    {
        HandleMovement();
        HandleRotation();
    }

    private void HandleMovement()
    {
        if (kb == null) { kb = Keyboard.current; }

        // get current position
        MyVector3 newPosition = transform.position;

        // handle movement input with Camera orientation
        if (kb.wKey.isPressed)
        {
            Vector3 forward = mainCamera.transform.forward;
            forward.y = 0; // keep movement horizontal
            forward.Normalize();
            newPosition += new MyVector3(forward.x, forward.y, forward.z) * movementSpeed * Time.deltaTime;
        }
        if (kb.sKey.isPressed)
        {
            Vector3 backward = -mainCamera.transform.forward;
            backward.y = 0; // keep movement horizontal
            backward.Normalize();
            newPosition += new MyVector3(backward.x, backward.y, backward.z) * movementSpeed * Time.deltaTime;
        }
        if (kb.aKey.isPressed)
        {
            Vector3 left = -mainCamera.transform.right;
            left.y = 0; // keep movement horizontal
            left.Normalize();
            newPosition += new MyVector3(left.x, left.y, left.z) * movementSpeed * Time.deltaTime;
        }
        if (kb.dKey.isPressed)
        {
            Vector3 right = mainCamera.transform.right;
            right.y = 0; // keep movement horizontal
            right.Normalize();
            newPosition += new MyVector3(right.x, right.y, right.z) * movementSpeed * Time.deltaTime;
        }

        // apply the final position after calculations
        transform.position = newPosition;
    }

    private void HandleRotation()
    {
        // rotate character towards movement direction
        Vector3 desiredDirection = -mainCamera.transform.right;
        desiredDirection.y = 0; // keep rotation horizontal
        desiredDirection.Normalize();

        if (desiredDirection.sqrMagnitude > 0.01f)
        {
            Quaternion targetRotation = Quaternion.LookRotation(desiredDirection);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, 10f * Time.deltaTime);
        }
    }
}