using UnityEngine;
using System.Collections.Generic;

public class DocOctopusManager : MonoBehaviour
{
    [Header("Arm Setup")]
    [SerializeField] private OctopusArm[] arms;
    [SerializeField] private Transform spiderMan;
    
    [Header("Coordination")]
    [SerializeField] private bool coordinateArms = true;
    [SerializeField] private float armSeparation = 2f;
    
    [Header("Game Logic")]
    [SerializeField] private int armsNeededToCapture = 2;
    [SerializeField] private float captureDistance = 1.5f;
    
    private List<OctopusArm> grabbingArms = new List<OctopusArm>();
    private bool targetCaptured = false;

    void Start()
    {
        if (arms.Length == 0)
        {
            Debug.LogError("No arms assigned to Doc Octopus!");
            return;
        }
        
        // Assign target to all arms
        foreach (var arm in arms)
        {
            if (coordinateArms)
            {
                // Each arm targets a slightly different position around Spider-Man
                arm.SetTarget(spiderMan);
            }
            else
            {
                arm.SetTarget(spiderMan);
            }
        }
    }

    void Update()
    {
        if (targetCaptured) return;
        
        if (coordinateArms)
        {
            CoordinateArmTargets();
        }
        
        CheckCapture();
    }

    void CoordinateArmTargets()
    {
        // Distribute arms around the target in a circle
        if (spiderMan == null) return;
        
        float angleStep = 360f / arms.Length;
        
        for (int i = 0; i < arms.Length; i++)
        {
            float angle = angleStep * i;
            Vector3 offset = new Vector3(
                Mathf.Cos(angle * Mathf.Deg2Rad) * armSeparation,
                0,
                Mathf.Sin(angle * Mathf.Deg2Rad) * armSeparation
            );
            
            // Create virtual target around Spider-Man
            Vector3 targetPos = spiderMan.position + offset;
            
            // You could create actual transform targets or modify the arm code
            // For now, all arms target Spider-Man directly
        }
    }

    void CheckCapture()
    {
        grabbingArms.Clear();
        
        // Check how many arms are close to Spider-Man
        foreach (var arm in arms)
        {
            // You would need to expose the grabPoint from OctopusArm
            // For now, we check distance from arm base to target
            float distance = Vector3.Distance(transform.position, spiderMan.position);
            
            // This is simplified - in real implementation check individual arm grab points
            if (distance < captureDistance * arms.Length)
            {
                grabbingArms.Add(arm);
            }
        }
        
        // If enough arms are grabbing, capture!
        if (grabbingArms.Count >= armsNeededToCapture && !targetCaptured)
        {
            CaptureTarget();
        }
    }

    void CaptureTarget()
    {
        targetCaptured = true;
        
        SpiderManController spiderController = spiderMan.GetComponent<SpiderManController>();
        if (spiderController != null)
        {
            spiderController.OnCaught();
        }
        
        Debug.Log($"Spider-Man captured by {grabbingArms.Count} arms!");
        
        // Optional: trigger victory animation or UI
        OnTargetCaptured();
    }

    void OnTargetCaptured()
    {
        // Add your game logic here
        // - Show victory UI
        // - Play animation
        // - Stop the game
        // - Award points
    }

    public void ResetGame()
    {
        targetCaptured = false;
        grabbingArms.Clear();
        
        foreach (var arm in arms)
        {
            arm.Release();
        }
        
        SpiderManController spiderController = spiderMan.GetComponent<SpiderManController>();
        if (spiderController != null)
        {
            // Re-enable Spider-Man movement
            spiderController.enabled = false;
            spiderController.enabled = true;
        }
    }

    void OnDrawGizmos()
    {
        if (spiderMan != null)
        {
            // Draw capture radius
            Gizmos.color = targetCaptured ? Color.red : Color.cyan;
            Gizmos.DrawWireSphere(spiderMan.position, captureDistance);
            
            // Draw arm separation visualization
            if (coordinateArms)
            {
                Gizmos.color = Color.magenta;
                float angleStep = 360f / Mathf.Max(arms.Length, 1);
                
                for (int i = 0; i < arms.Length; i++)
                {
                    float angle = angleStep * i;
                    Vector3 offset = new Vector3(
                        Mathf.Cos(angle * Mathf.Deg2Rad) * armSeparation,
                        0,
                        Mathf.Sin(angle * Mathf.Deg2Rad) * armSeparation
                    );
                    
                    Vector3 targetPos = spiderMan.position + offset;
                    Gizmos.DrawWireSphere(targetPos, 0.3f);
                }
            }
        }
    }
}