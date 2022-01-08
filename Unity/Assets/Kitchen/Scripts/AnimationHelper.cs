using System.Collections.Generic;
using System.ComponentModel;
using UnityEngine;
using UnityEngine.UI;


//This script is for making a control for animations.
// First Array is used to specify keys that will corresponds to animations from second array
[RequireComponent(typeof(Animation))]
public class AnimationHelper : MonoBehaviour
{
    
    [Tooltip("You can use the same button for multiple animations, If you do, then the animations will toggle.")]
    
    [Header("KeyCodes")] public KeyCode[] _keyCodes;
    [Header("Animation Clips")] public AnimationClip[] _animationClips;

    [Range(0.1f,12)]
    public float animationSpeed = 1f;
    private Animation _animation;
    private List<Animation> _animations;

    private Dictionary<KeyCode, int> keyCodeIndexDictionary = new Dictionary<KeyCode, int>();


    void Start()
    {
        _animation = GetComponent<Animation>();
        if (_keyCodes.Length == _animationClips.Length)
        {
            for (int i = 0; i < _keyCodes.Length; i++)
            {
                keyCodeIndexDictionary[_keyCodes[i]] = -1;
                _animationClips[i].legacy = true;
                _animation.AddClip(_animationClips[i], _animationClips[i].name);
            }
        }
        else
        {
            Debug.Log("You need to have same amount of Keycodes and Animation Clips");
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (_keyCodes.Length == _animationClips.Length)
        {
            for (int i = 0; i < _keyCodes.Length; i++)
            {
                if (Input.GetKey(_keyCodes[i]))
                {
                    // _animation.Play(_animationClips[i].name);
                    PlayAnimations(_keyCodes[i]);
                }
            }
        }
    }


    private List<int> GetIndexes(int lastIndex, KeyCode keyCode)
    {
        List<int> indexes = new List<int>();
        for (int i = 0; i < _keyCodes.Length; i++)
        {
            if (keyCode == _keyCodes[i])
            {
                if (i > lastIndex)
                {
                    indexes.Add(i);
                }
            }
        }

        return indexes;
    }

    void PlayAnimations(KeyCode keyCode)
    {
        List<int> indexes = GetIndexes(keyCodeIndexDictionary[keyCode], keyCode);

        // All animations has been played, reversing to beginning
        if (indexes.Count == 0)
        {
            indexes = GetIndexes(-1, keyCode);
        }

        for (int i = 0; i < _animationClips.Length; i++)
        {
            // Last time animation was played by the same key, so lets play next animation
            if (!_animation.isPlaying && i == indexes[0])
            {
                keyCodeIndexDictionary[keyCode] = i;
                _animation[_animationClips[i].name].speed = animationSpeed;
                _animation.Play(_animationClips[i].name);
                break;
            }
        }
    }
}