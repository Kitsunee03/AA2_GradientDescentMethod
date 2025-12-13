using UnityEngine;

public class SpiderManController : MonoBehaviour
{
    [Header("Movement Settings")]
    [SerializeField] private float moveSpeed = 5f;
    [SerializeField] private float jumpForce = 5f;

    [Header("Plane Bounds")]
    [SerializeField] private Collider planeBounds;

    [Header("Random Movement")]
    [SerializeField] private float changeDirectionInterval = 2f;
    [SerializeField] private float jumpCheckInterval = 5f;
    [SerializeField] private Vector2 movementAreaMin = new(-10, -10);
    [SerializeField] private Vector2 movementAreaMax = new(10, 10);

    private Rigidbody rb;
    private MyVector3 randomDirection;
    private float directionTimer;
    private float jumpTimer;
    private int previousDirection = -1;

    private void Start()
    {
        TryGetComponent(out rb);
        if (rb == null) { rb = gameObject.AddComponent<Rigidbody>(); }

        rb.useGravity = false;
        rb.constraints = RigidbodyConstraints.FreezeRotation;

        ChooseRandomDirection();
        directionTimer = changeDirectionInterval;
    }

    private void Update() { RandomMovement(); }

    private void RandomMovement()
    {
        directionTimer -= Time.deltaTime;

        if (directionTimer <= 0)
        {
            ChooseRandomDirection();
            directionTimer = changeDirectionInterval;
        }

        // calcular nueva posición
        MyVector3 currentPos = transform.position;
        MyVector3 movement = randomDirection * moveSpeed * Time.deltaTime;
        MyVector3 newPos = currentPos + movement;

        // aplicar límites del plano
        if (planeBounds != null)
        {
            Vector3 clampedPos = planeBounds.ClosestPoint((Vector3)newPos);
            newPos = new MyVector3(clampedPos.x, clampedPos.y, clampedPos.z);
        }
        else
        {
            newPos.x = Mathf.Clamp(newPos.x, movementAreaMin.x, movementAreaMax.x);
            newPos.z = Mathf.Clamp(newPos.z, movementAreaMin.y, movementAreaMax.y);
            newPos.y = Mathf.Max(newPos.y, 0.5f);
        }

        // mover
        if (rb != null) { rb.MovePosition((Vector3)newPos); }
        else { transform.position = (Vector3)newPos; }

        // rotar hacia la dirección de movimiento (solo invertir x para corregir orientación)
        MyVector3 lookDirection = new MyVector3(-randomDirection.x, randomDirection.y, randomDirection.z);
        MyQuaternion targetRot = MyQuaternion.LookRotation(lookDirection, MyVector3.up);
        MyQuaternion smoothRot = MyQuaternion.Slerp(transform.rotation, targetRot, Time.deltaTime * 10f);
        
        if (rb != null) { rb.MoveRotation(smoothRot); }
        else { transform.rotation = smoothRot; }
    }

    private void ChooseRandomDirection()
    {
        int direction;
        do
        {
            direction = Random.Range(0, 4);
        } while (direction == previousDirection);
        
        previousDirection = direction;
        
        randomDirection = direction switch
        {
            0 => MyVector3.right,     // derecha
            1 => MyVector3.left,      // izquierda
            2 => MyVector3.forward,   // adelante
            3 => MyVector3.back,      // atrás
            _ => MyVector3.right
        };
        
        /*string directionName = direction switch
        {
            0 => "Derecha",
            1 => "Izquierda",
            2 => "Adelante",
            3 => "Atrás",
            _ => "Derecha"
        };
        
        Debug.Log($"Spiderman va hacia: {directionName} - Vector: ({randomDirection.x}, {randomDirection.y}, {randomDirection.z})");
        */
    }
}