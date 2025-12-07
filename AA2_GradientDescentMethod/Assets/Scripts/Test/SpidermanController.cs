using UnityEngine;

public class SpiderManController : MonoBehaviour
{
    [Header("Movement Settings")]
    [SerializeField] private float moveSpeed = 5f;
    
    [Header("Random Movement")]
    [SerializeField] private bool randomMovement = true;
    [SerializeField] private float changeDirectionInterval = 2f;
    [SerializeField] private Vector2 movementAreaMin = new Vector2(-10, -10);
    [SerializeField] private Vector2 movementAreaMax = new Vector2(10, 10);
    
    [Header("Predefined Path")]
    [SerializeField] private bool followPath = false;
    [SerializeField] private Transform[] pathPoints;
    [SerializeField] private int currentPathIndex = 0;
    
    private Rigidbody rb;
    private Vector3 randomDirection;
    private float directionTimer;
    private bool isGrounded;
    
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
        }
        
        rb.useGravity = true;
        rb.constraints = RigidbodyConstraints.FreezeRotation;
        
        ChooseRandomDirection();
    }

    void Update()
    {
        if (followPath && pathPoints != null && pathPoints.Length > 0)
        {
            FollowPredefinedPath();
        }
        else if (randomMovement)
        {
            RandomMovement();
        }
        
    }

    void RandomMovement()
    {
        directionTimer -= Time.deltaTime;
        
        if (directionTimer <= 0)
        {
            ChooseRandomDirection();
            directionTimer = changeDirectionInterval;
        }
        
        // Move in random direction
        Vector3 movement = randomDirection * moveSpeed * Time.deltaTime;
        transform.position += movement;
        
        // Keep within bounds
        ClampToBounds();
        
        // Face movement direction
        if (movement.magnitude > 0.01f)
        {
            transform.forward = Vector3.Lerp(transform.forward, movement.normalized, Time.deltaTime * 5f);
        }
    }

    void ChooseRandomDirection()
    {
        float angle = Random.Range(0f, 360f);
        randomDirection = new Vector3(
            Mathf.Cos(angle * Mathf.Deg2Rad),
            0,
            Mathf.Sin(angle * Mathf.Deg2Rad)
        );
        
        // Randomly add vertical movement
        if (Random.value < 0.3f)
        {
            randomDirection.y = Random.Range(-0.5f, 0.5f);
        }
        
        randomDirection.Normalize();
    }

    void FollowPredefinedPath()
    {
        if (pathPoints.Length == 0) return;
        
        Transform targetPoint = pathPoints[currentPathIndex];
        Vector3 direction = (targetPoint.position - transform.position).normalized;
        
        transform.position += direction * moveSpeed * Time.deltaTime;
        transform.forward = Vector3.Lerp(transform.forward, direction, Time.deltaTime * 5f);
        
        // Check if reached point
        if (Vector3.Distance(transform.position, targetPoint.position) < 1f)
        {
            currentPathIndex = (currentPathIndex + 1) % pathPoints.Length;
        }
    }

    void ClampToBounds()
    {
        Vector3 pos = transform.position;
        pos.x = Mathf.Clamp(pos.x, movementAreaMin.x, movementAreaMax.x);
        pos.z = Mathf.Clamp(pos.z, movementAreaMin.y, movementAreaMax.y);
        pos.y = Mathf.Max(pos.y, 0.5f); // Keep above ground
        transform.position = pos;
    }
}