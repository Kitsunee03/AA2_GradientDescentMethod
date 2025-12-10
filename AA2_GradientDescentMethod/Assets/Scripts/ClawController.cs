using UnityEngine;

public class ClawController : MonoBehaviour
{
    private Animator clawAnimator;

    private void Awake() { clawAnimator = GetComponent<Animator>(); }


    private void OnTriggerEnter(Collider p_other)
    {
        if (p_other.CompareTag("Player")) { clawAnimator.SetBool("isOpen", false); }
    }

    private void OnTriggerExit(Collider p_other)
    {
        if (p_other.CompareTag("Player")) { clawAnimator.SetBool("isOpen", true); }
    }

}