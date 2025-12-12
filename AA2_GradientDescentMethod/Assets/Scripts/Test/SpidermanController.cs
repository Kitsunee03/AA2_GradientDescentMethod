using UnityEngine;

public class SpiderManController : MonoBehaviour
{
    [Header("Movement Settings")]
    [SerializeField] private float moveSpeed = 5f;
    
    [Header("Plane Bounds")]
    [SerializeField] private Collider planeBounds;
    [SerializeField] private float planeYOffset = 0.5f;
    
    [Header("Random Movement")]
    [SerializeField] private float changeDirectionInterval = 2f;
    [SerializeField] private Vector2 movementAreaMin = new Vector2(-10, -10);
    [SerializeField] private Vector2 movementAreaMax = new Vector2(10, 10);
    
    private Rigidbody rb;
    private MyVector3 randomDirection;
    private float directionTimer;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
        }
        
        rb.useGravity = false;
        rb.constraints = RigidbodyConstraints.FreezeRotation;
        
        ChooseRandomDirection();
        directionTimer = changeDirectionInterval;
    }

    void Update()
    {
        RandomMovement();
    }

    void RandomMovement()
    {
        directionTimer -= Time.deltaTime;
        
        if (directionTimer <= 0)
        {
            ChooseRandomDirection();
            directionTimer = changeDirectionInterval;
        }

        // movimiento deseado en world space (MyVector3 -> Vector3)
        Vector3 desiredMovement = (Vector3)randomDirection * moveSpeed * Time.deltaTime;
        Vector3 desiredPos = transform.position + desiredMovement;

        // obtener posición final limitada por el collider (ClosestPoint)
        if (planeBounds != null)
        {
            Vector3 surfacePoint = planeBounds.ClosestPoint(desiredPos);
            desiredPos = new Vector3(surfacePoint.x, surfacePoint.y, surfacePoint.z);
        }
        else
        {
            desiredPos.x = Mathf.Clamp(desiredPos.x, movementAreaMin.x, movementAreaMax.x);
            desiredPos.z = Mathf.Clamp(desiredPos.z, movementAreaMin.y, movementAreaMax.y);
            desiredPos.y = Mathf.Max(desiredPos.y, 0.5f);
        }

        // guardar la dirección real del movimiento antes de mover (para rotación)
        Vector3 moveDir = desiredPos - transform.position;

        // mover respetando Rigidbody
        if (rb != null)
            rb.MovePosition(desiredPos);
        else
            transform.position = desiredPos;
        
        // rotar hacia la dirección real del movimiento
        if (moveDir.magnitude > 0.01f)
            transform.forward = Vector3.Lerp(transform.forward, moveDir.normalized, Time.deltaTime * 5f);
    }

    void ChooseRandomDirection()
    {
        int direction = Random.Range(0, 3); // 0..2
        randomDirection = direction switch
        {
            0 => MyVector3.right,
            1 => MyVector3.left,
            2 => MyVector3.forward,
            _ => MyVector3.right
        };
    }
}