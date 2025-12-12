using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class DocController : MonoBehaviour
{
    [SerializeField] private Camera mainCamera;
    [SerializeField] private List<IKGradient> arms;
    private IKGradient activeArm;

    [Header("Movement Settings")]
    [SerializeField] private float movementSpeed = 5f;
    private Keyboard kb;

    private void Awake()
    {
        kb = Keyboard.current;
        HandleActiveArm();
    }

    private void Update()
    {
        HandleMovement();
        HandleRotation();
        
        HandleActiveArm();
    }

    private void HandleMovement()
    {
        if (kb == null) { kb = Keyboard.current; }

        // get current position
        MyVector3 newPosition = transform.position;

        // handle movement input with Camera orientation
        if (kb.wKey.isPressed)
        {
            MyVector3 forward = mainCamera.transform.forward;
            forward.y = 0; // keep movement horizontal
            forward = forward.normalized;
            newPosition = newPosition + forward * movementSpeed * Time.deltaTime;
        }
        if (kb.sKey.isPressed)
        {
            MyVector3 backward = mainCamera.transform.forward;
            backward = new MyVector3(-backward.x, 0, -backward.z);
            backward = backward.normalized;
            newPosition = newPosition + backward * movementSpeed * Time.deltaTime;
        }
        if (kb.aKey.isPressed)
        {
            MyVector3 left = mainCamera.transform.right;
            left = new MyVector3(-left.x, 0, -left.z);
            left = left.normalized;
            newPosition = newPosition + left * movementSpeed * Time.deltaTime;
        }
        if (kb.dKey.isPressed)
        {
            MyVector3 right = mainCamera.transform.right;
            right.y = 0; // keep movement horizontal
            right = right.normalized;
            newPosition = newPosition + right * movementSpeed * Time.deltaTime;
        }

        // apply the final position after calculations
        transform.position = newPosition;
    }

    private void HandleRotation()
    {
        // rotate character towards movement direction
        MyVector3 desiredDirection = mainCamera.transform.right;
        desiredDirection = new MyVector3(-desiredDirection.x, 0, -desiredDirection.z); // negate and keep horizontal
        desiredDirection = desiredDirection.normalized;

        float sqrMag = desiredDirection.x * desiredDirection.x + desiredDirection.y * desiredDirection.y + desiredDirection.z * desiredDirection.z;
        if (sqrMag > 0.01f)
        {
            // convert to Unity for LookRotation and Slerp (these are complex operations)
            Vector3 unityDir = desiredDirection;
            Quaternion targetRotation = Quaternion.LookRotation(unityDir);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, 10f * Time.deltaTime);
        }
    }

    private void HandleActiveArm()
    {
        if (activeArm == null) { activeArm = arms[0]; }
        float activeDistance = activeArm.GetDistanceToTarget();

        foreach (IKGradient arm in arms)
        {
            if(arm.GetDistanceToTarget() < activeDistance)
            {
                activeArm.IsInUse = false;
                activeArm = arm;
                activeArm.IsInUse = true;
                activeDistance = arm.GetDistanceToTarget();
            }
        }
    }
}