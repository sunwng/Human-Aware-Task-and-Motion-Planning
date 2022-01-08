using UnityEngine;
[RequireComponent(typeof(Animator))]
public class OpenClose : MonoBehaviour {
    public KeyCode key;
    private Animator _animator;
    void Start() {
        _animator = GetComponent<Animator>();
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(key)) {
            bool isOpen = _animator.GetBool("isOpen");
            _animator.SetBool("isOpen", !isOpen);
        }

    }
}
