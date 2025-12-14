---
sidebar_position: 2
title: High-fidelity Rendering and Human-Robot Interaction
---

# High-fidelity Rendering and Human-Robot Interaction

## Introduction

While Gazebo excels at physics simulation, **Unity** offers photorealistic rendering and advanced interaction capabilities essential for developing humanoid robots that work alongside humans. This chapter explores how to leverage Unity's powerful graphics engine and interaction systems to create compelling digital twins that bridge simulation and reality.

**Why Unity for Humanoid Robotics?**

- **Photorealistic Graphics**: Ray tracing, global illumination, and material systems that mirror real-world lighting
- **Human Animation**: Mecanim animation system for natural human motion
- **Cross-Platform**: Deploy simulations to VR/AR for immersive testing
- **Asset Ecosystem**: Massive library of 3D models, environments, and animations
- **ML-Agents**: Native reinforcement learning framework for training AI

## The Rendering Pipeline for Robotics

### Understanding the Graphics Pipeline

Unity's rendering pipeline consists of several stages that affect how realistic your simulation appears:

```
Scene → Culling → Rendering → Post-Processing → Display
```

**Key Rendering Pipelines:**

| Pipeline | Use Case | Performance | Visual Quality |
|----------|----------|-------------|----------------|
| **Built-in** | Legacy, simple scenes | High | Low-Medium |
| **Universal (URP)** | Mobile, VR, general robotics | Medium-High | Medium-High |
| **High Definition (HDRP)** | Photorealistic training data | Low-Medium | Very High |

### Choosing HDRP for Humanoid Simulation

For humanoid robotics, **HDRP (High Definition Render Pipeline)** is recommended because:

- **Physically-Based Materials**: Accurate surface properties (metal, skin, fabric)
- **Volumetric Lighting**: Realistic fog, light shafts for depth perception
- **Ray-Traced Reflections**: Critical for mirror-like surfaces and metallic robots
- **Advanced Shadows**: Soft, contact-hardening shadows for visual realism

```csharp
// Example: Setting up HDRP in a ROS-Unity bridge scene
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

public class HDRPSetup : MonoBehaviour
{
    void Start()
    {
        // Configure HDRP volume for robotics simulation
        Volume volume = gameObject.AddComponent<Volume>();
        volume.isGlobal = true;

        VolumeProfile profile = volume.sharedProfile;

        // Enable ambient occlusion for depth cues
        if (profile.TryGet(out AmbientOcclusion ao))
        {
            ao.intensity.value = 0.5f;
            ao.directLightingStrength.value = 0.25f;
        }

        // Configure exposure for consistent lighting
        if (profile.TryGet(out Exposure exposure))
        {
            exposure.mode.value = ExposureMode.Fixed;
            exposure.fixedExposure.value = 12f;
        }
    }
}
```

## Materials and Lighting for Realistic Robots

### Physically-Based Materials (PBR)

Humanoid robots contain diverse materials: aluminum frames, rubber grips, plastic casings, LED displays. Unity's PBR system models these accurately.

**Key Material Properties:**

1. **Albedo** (Base Color): The diffuse color of the surface
2. **Metallic**: Whether the surface is metallic (0 = dielectric, 1 = metal)
3. **Smoothness**: Surface roughness (0 = rough, 1 = mirror-smooth)
4. **Normal Map**: Surface detail without adding geometry
5. **Emission**: Self-illuminating surfaces (LEDs, screens)

```csharp
// Creating a robot metal material programmatically
Material CreateRobotMetal()
{
    Material metal = new Material(Shader.Find("HDRP/Lit"));

    // Brushed aluminum properties
    metal.SetColor("_BaseColor", new Color(0.9f, 0.9f, 0.9f));
    metal.SetFloat("_Metallic", 1.0f);        // Fully metallic
    metal.SetFloat("_Smoothness", 0.7f);      // Slightly rough
    metal.SetFloat("_NormalScale", 0.5f);     // Subtle surface detail

    return metal;
}

// Soft rubber grip material
Material CreateRubberGrip()
{
    Material rubber = new Material(Shader.Find("HDRP/Lit"));

    rubber.SetColor("_BaseColor", new Color(0.1f, 0.1f, 0.1f));
    rubber.SetFloat("_Metallic", 0.0f);       // Non-metallic
    rubber.SetFloat("_Smoothness", 0.2f);     // Very rough

    return rubber;
}
```

### Advanced Lighting Techniques

Proper lighting is essential for computer vision systems to work in simulation.

**Three-Point Lighting Setup:**

```csharp
public class RoboticsLightingRig : MonoBehaviour
{
    public Transform robot;

    void CreateThreePointLighting()
    {
        // Key Light (main illumination)
        GameObject keyLight = new GameObject("Key Light");
        HDAdditionalLightData keyLightData = keyLight.AddComponent<Light>().gameObject.AddComponent<HDAdditionalLightData>();
        keyLight.transform.position = robot.position + new Vector3(2, 3, 2);
        keyLight.transform.LookAt(robot);
        keyLightData.intensity = 50000;
        keyLightData.SetColor(Color.white);

        // Fill Light (soften shadows)
        GameObject fillLight = new GameObject("Fill Light");
        HDAdditionalLightData fillLightData = fillLight.AddComponent<Light>().gameObject.AddComponent<HDAdditionalLightData>();
        fillLight.transform.position = robot.position + new Vector3(-2, 1, 1);
        fillLight.transform.LookAt(robot);
        fillLightData.intensity = 20000;

        // Back Light (separation from background)
        GameObject backLight = new GameObject("Back Light");
        HDAdditionalLightData backLightData = backLight.AddComponent<Light>().gameObject.AddComponent<HDAdditionalLightData>();
        backLight.transform.position = robot.position + new Vector3(0, 2, -2);
        backLight.transform.LookAt(robot);
        backLightData.intensity = 30000;
    }
}
```

## Human-Robot Interaction (HRI) Fundamentals

### The HRI Design Space

Human-Robot Interaction encompasses:

1. **Physical Interaction**: Handshakes, object handoffs, collaborative manipulation
2. **Social Interaction**: Gaze, gestures, proxemics (personal space)
3. **Cognitive Interaction**: Intent recognition, task understanding, shared goals

**Key HRI Principles for Humanoids:**

- **Predictability**: Humans should understand robot intentions
- **Legibility**: Robot motions should clearly communicate goals
- **Comfort**: Respect personal space and move naturally
- **Safety**: Never surprise or threaten users

### Implementing Gaze Behavior

Eye contact is fundamental to human communication. Humanoid robots must simulate natural gaze patterns.

```csharp
using UnityEngine;

public class HumanoidGaze : MonoBehaviour
{
    public Transform headBone;
    public Transform leftEye;
    public Transform rightEye;
    public Transform currentTarget;

    [Range(0f, 1f)]
    public float blinkFrequency = 0.1f;

    private float nextBlinkTime;
    private float gazeShiftTimer;

    void Update()
    {
        // Natural head tracking with damping
        if (currentTarget != null)
        {
            Vector3 targetDirection = currentTarget.position - headBone.position;
            Quaternion targetRotation = Quaternion.LookRotation(targetDirection);

            // Smooth head movement (humans don't snap their heads)
            headBone.rotation = Quaternion.Slerp(
                headBone.rotation,
                targetRotation,
                Time.deltaTime * 2f
            );

            // Eyes lead the head slightly
            RotateEyes(targetDirection);
        }

        // Periodic blinking for realism
        if (Time.time > nextBlinkTime)
        {
            Blink();
            nextBlinkTime = Time.time + Random.Range(2f, 6f);
        }

        // Gaze shifting (humans don't stare continuously)
        gazeShiftTimer += Time.deltaTime;
        if (gazeShiftTimer > Random.Range(3f, 8f))
        {
            ShiftGaze();
            gazeShiftTimer = 0f;
        }
    }

    void RotateEyes(Vector3 direction)
    {
        // Eyes can rotate independently of head (up to ~35 degrees)
        Quaternion eyeRotation = Quaternion.LookRotation(direction);
        leftEye.rotation = Quaternion.Slerp(leftEye.rotation, eyeRotation, Time.deltaTime * 5f);
        rightEye.rotation = Quaternion.Slerp(rightEye.rotation, eyeRotation, Time.deltaTime * 5f);
    }

    void Blink()
    {
        // Animate eyelid closure (requires blend shapes)
        // Typical blink duration: 100-150ms
        StartCoroutine(BlinkAnimation());
    }

    System.Collections.IEnumerator BlinkAnimation()
    {
        float blinkDuration = 0.15f;
        float elapsed = 0f;

        while (elapsed < blinkDuration)
        {
            float blinkAmount = Mathf.Sin((elapsed / blinkDuration) * Mathf.PI);
            // Apply to eyelid blend shape
            elapsed += Time.deltaTime;
            yield return null;
        }
    }

    void ShiftGaze()
    {
        // Humans periodically look away during conversation
        // Shift gaze to nearby point, then return
        Vector3 randomOffset = Random.insideUnitSphere * 0.5f;
        currentTarget.position += randomOffset;
    }
}
```

### Proxemics: Respecting Personal Space

**Edward T. Hall's Proxemic Zones:**

- **Intimate (0-0.5m)**: Close relationships only
- **Personal (0.5-1.2m)**: Friends and family
- **Social (1.2-3.6m)**: Acquaintances and strangers
- **Public (3.6m+)**: Formal interactions

```csharp
public class ProxemicsController : MonoBehaviour
{
    public Transform humanTarget;
    public float intimateDistance = 0.5f;
    public float personalDistance = 1.2f;
    public float socialDistance = 3.6f;

    public enum ProxemicZone { Intimate, Personal, Social, Public }

    public ProxemicZone GetCurrentZone()
    {
        float distance = Vector3.Distance(transform.position, humanTarget.position);

        if (distance < intimateDistance) return ProxemicZone.Intimate;
        if (distance < personalDistance) return ProxemicZone.Personal;
        if (distance < socialDistance) return ProxemicZone.Social;
        return ProxemicZone.Public;
    }

    void Update()
    {
        ProxemicZone zone = GetCurrentZone();

        // Adjust robot behavior based on zone
        switch (zone)
        {
            case ProxemicZone.Intimate:
                // Reduce movement speed, softer voice
                GetComponent<RobotController>().maxSpeed = 0.3f;
                GetComponent<AudioSource>().volume = 0.5f;
                break;

            case ProxemicZone.Personal:
                // Normal interaction
                GetComponent<RobotController>().maxSpeed = 0.8f;
                GetComponent<AudioSource>().volume = 0.7f;
                break;

            case ProxemicZone.Social:
                // Louder voice, more expressive gestures
                GetComponent<RobotController>().maxSpeed = 1.0f;
                GetComponent<AudioSource>().volume = 0.9f;
                break;

            case ProxemicZone.Public:
                // Formal, reduced interaction
                GetComponent<RobotController>().maxSpeed = 1.0f;
                GetComponent<AudioSource>().volume = 1.0f;
                break;
        }
    }
}
```

## Gesture Recognition and Generation

### Recognizing Human Gestures

Unity's input system combined with computer vision can detect human gestures.

```csharp
using UnityEngine;
using System.Collections.Generic;

public class GestureRecognizer : MonoBehaviour
{
    public Transform rightHand;
    public Transform leftHand;

    private List<Vector3> rightHandTrajectory = new List<Vector3>();
    private List<Vector3> leftHandTrajectory = new List<Vector3>();

    public enum Gesture { Wave, Point, Thumbsup, Stop, Come, None }

    void Update()
    {
        // Record hand positions
        rightHandTrajectory.Add(rightHand.position);
        leftHandTrajectory.Add(leftHand.position);

        // Keep only recent history (2 seconds at 60 FPS = 120 frames)
        if (rightHandTrajectory.Count > 120)
        {
            rightHandTrajectory.RemoveAt(0);
            leftHandTrajectory.RemoveAt(0);
        }

        // Analyze trajectory for gestures
        Gesture detected = AnalyzeGesture(rightHandTrajectory);

        if (detected != Gesture.None)
        {
            OnGestureDetected(detected);
        }
    }

    Gesture AnalyzeGesture(List<Vector3> trajectory)
    {
        if (trajectory.Count < 30) return Gesture.None;

        // Wave detection: lateral oscillation
        if (IsWave(trajectory))
            return Gesture.Wave;

        // Point detection: extended arm with stable position
        if (IsPointing(trajectory))
            return Gesture.Point;

        // Stop gesture: palm facing forward
        if (IsStopGesture())
            return Gesture.Stop;

        return Gesture.None;
    }

    bool IsWave(List<Vector3> trajectory)
    {
        // Detect side-to-side motion
        float lateralVariance = CalculateLateralVariance(trajectory);
        return lateralVariance > 0.3f; // Threshold for wave motion
    }

    bool IsPointing(List<Vector3> trajectory)
    {
        // Check if hand is extended and stable
        Vector3 recent = trajectory[trajectory.Count - 1];
        Vector3 shoulderPos = transform.position + Vector3.up * 1.5f;

        float armExtension = Vector3.Distance(recent, shoulderPos);
        float stability = CalculateStability(trajectory);

        return armExtension > 0.6f && stability < 0.1f;
    }

    bool IsStopGesture()
    {
        // Check palm orientation (requires hand tracking data)
        // Simplified: check if hand is at chest height with outward facing
        return rightHand.position.y > transform.position.y + 1.0f;
    }

    float CalculateLateralVariance(List<Vector3> trajectory)
    {
        float sum = 0f;
        for (int i = 1; i < trajectory.Count; i++)
        {
            sum += Mathf.Abs(trajectory[i].x - trajectory[i-1].x);
        }
        return sum / trajectory.Count;
    }

    float CalculateStability(List<Vector3> trajectory)
    {
        Vector3 mean = Vector3.zero;
        foreach (Vector3 pos in trajectory)
            mean += pos;
        mean /= trajectory.Count;

        float variance = 0f;
        foreach (Vector3 pos in trajectory)
            variance += Vector3.Distance(pos, mean);

        return variance / trajectory.Count;
    }

    void OnGestureDetected(Gesture gesture)
    {
        Debug.Log($"Detected gesture: {gesture}");

        // Respond to gesture
        switch (gesture)
        {
            case Gesture.Wave:
                RespondWithWave();
                break;
            case Gesture.Come:
                MoveTowardsPerson();
                break;
            case Gesture.Stop:
                Halt();
                break;
        }
    }

    void RespondWithWave()
    {
        // Trigger wave animation
        GetComponent<Animator>().SetTrigger("Wave");
    }

    void MoveTowardsPerson()
    {
        // Navigate to human position
        GetComponent<UnityEngine.AI.NavMeshAgent>().SetDestination(humanTarget.position);
    }

    void Halt()
    {
        GetComponent<UnityEngine.AI.NavMeshAgent>().isStopped = true;
    }
}
```

### Generating Expressive Robot Gestures

Robots must generate natural, meaningful gestures.

```csharp
public class GestureGenerator : MonoBehaviour
{
    public Animator animator;

    public void ExpressEmotion(string emotion)
    {
        switch (emotion.ToLower())
        {
            case "happy":
                animator.SetTrigger("Celebrate");
                // Open arms, upward gaze
                break;

            case "confused":
                animator.SetTrigger("HeadTilt");
                animator.SetTrigger("ShrugShoulders");
                break;

            case "agreement":
                animator.SetTrigger("Nod");
                break;

            case "disagreement":
                animator.SetTrigger("Shake");
                break;
        }
    }

    public void PointAt(Vector3 worldPosition)
    {
        // Calculate which arm to use based on position
        bool useRightArm = worldPosition.x > transform.position.x;

        // Set IK target for pointing
        if (useRightArm)
        {
            animator.SetIKPositionWeight(AvatarIKGoal.RightHand, 1f);
            animator.SetIKPosition(AvatarIKGoal.RightHand, worldPosition);
        }
        else
        {
            animator.SetIKPositionWeight(AvatarIKGoal.LeftHand, 1f);
            animator.SetIKPosition(AvatarIKGoal.LeftHand, worldPosition);
        }

        // Look at target
        animator.SetLookAtWeight(1f);
        animator.SetLookAtPosition(worldPosition);
    }
}
```

## Natural Language Interaction

### Speech Synthesis Integration

Unity can integrate with text-to-speech engines for natural voice output.

```csharp
using UnityEngine;
using UnityEngine.Networking;
using System.Collections;

public class RobotSpeech : MonoBehaviour
{
    public AudioSource audioSource;
    private string ttsApiUrl = "https://api.openai.com/v1/audio/speech";

    public IEnumerator Speak(string text)
    {
        // Generate speech using OpenAI TTS
        UnityWebRequest request = UnityWebRequest.Post(ttsApiUrl, "");

        string jsonBody = $@"{{
            ""model"": ""tts-1"",
            ""voice"": ""nova"",
            ""input"": ""{text}""
        }}";

        byte[] bodyRaw = System.Text.Encoding.UTF8.GetBytes(jsonBody);
        request.uploadHandler = new UploadHandlerRaw(bodyRaw);
        request.downloadHandler = new DownloadHandlerAudioClip(ttsApiUrl, AudioType.MPEG);
        request.SetRequestHeader("Content-Type", "application/json");
        request.SetRequestHeader("Authorization", "Bearer " + GetApiKey());

        yield return request.SendWebRequest();

        if (request.result == UnityWebRequest.Result.Success)
        {
            AudioClip clip = DownloadHandlerAudioClip.GetContent(request);
            audioSource.clip = clip;
            audioSource.Play();

            // Animate mouth while speaking
            StartCoroutine(AnimateMouth(clip.length));
        }
    }

    IEnumerator AnimateMouth(float duration)
    {
        float elapsed = 0f;
        Animator animator = GetComponent<Animator>();

        while (elapsed < duration)
        {
            // Simulate lip movement
            float mouthOpen = Mathf.PerlinNoise(Time.time * 10f, 0f);
            animator.SetFloat("MouthOpen", mouthOpen);

            elapsed += Time.deltaTime;
            yield return null;
        }

        animator.SetFloat("MouthOpen", 0f);
    }

    string GetApiKey()
    {
        return System.Environment.GetEnvironmentVariable("OPENAI_API_KEY");
    }
}
```

## VR/AR Integration for Human-Robot Testing

### Testing HRI in Virtual Reality

VR allows humans to interact with simulated robots before hardware exists.

```csharp
using UnityEngine;
using UnityEngine.XR;

public class VRHRITester : MonoBehaviour
{
    public Transform vrLeftHand;
    public Transform vrRightHand;
    public Transform humanoidRobot;

    void Update()
    {
        // Detect hand gestures in VR
        if (IsGrabbingGesture(XRNode.RightHand))
        {
            TryGrabObject();
        }

        // Track personal space violations
        float distance = Vector3.Distance(vrLeftHand.position, humanoidRobot.position);
        if (distance < 0.5f)
        {
            humanoidRobot.GetComponent<ProxemicsController>().OnPersonalSpaceViolation();
        }
    }

    bool IsGrabbingGesture(XRNode hand)
    {
        InputDevice device = InputDevices.GetDeviceAtXRNode(hand);
        device.TryGetFeatureValue(CommonUsages.grip, out float gripValue);
        return gripValue > 0.8f;
    }

    void TryGrabObject()
    {
        // Raycast from hand to detect objects
        RaycastHit hit;
        if (Physics.Raycast(vrRightHand.position, vrRightHand.forward, out hit, 0.5f))
        {
            if (hit.collider.CompareTag("Grabbable"))
            {
                // Attach object to hand
                hit.transform.SetParent(vrRightHand);
            }
        }
    }
}
```

## Performance Optimization for Real-Time Interaction

### Rendering Optimizations

```csharp
public class RenderingOptimizer : MonoBehaviour
{
    void Start()
    {
        // Use LOD (Level of Detail) for distant robots
        LODGroup lodGroup = gameObject.AddComponent<LODGroup>();

        LOD[] lods = new LOD[3];

        // High detail (0-10m)
        lods[0] = new LOD(0.1f, GetComponent<Renderer>().materials);

        // Medium detail (10-30m)
        lods[1] = new LOD(0.03f, GetLowerPolyRenderers());

        // Low detail (30m+)
        lods[2] = new LOD(0.01f, GetLowestPolyRenderers());

        lodGroup.SetLODs(lods);
        lodGroup.RecalculateBounds();

        // Enable GPU instancing for repeated objects
        foreach (Renderer r in GetComponentsInChildren<Renderer>())
        {
            r.material.enableInstancing = true;
        }

        // Occlusion culling for indoor environments
        Camera.main.useOcclusionCulling = true;
    }

    Renderer[] GetLowerPolyRenderers()
    {
        // Return simplified mesh renderers
        return new Renderer[0]; // Placeholder
    }

    Renderer[] GetLowestPolyRenderers()
    {
        return new Renderer[0]; // Placeholder
    }
}
```

## Summary

High-fidelity rendering in Unity enables:

- **Photorealistic training environments** for computer vision systems
- **Natural human-robot interaction** through gaze, gestures, and proxemics
- **Immersive VR/AR testing** before deploying physical robots
- **Expressive robot behaviors** that build trust and understanding

**Key Takeaways:**

1. Use HDRP for physically-accurate materials and lighting
2. Implement natural gaze and gesture systems
3. Respect human proxemic zones for comfortable interaction
4. Integrate speech synthesis for natural communication
5. Optimize rendering for real-time performance

By mastering these techniques, you can create digital twins that not only look realistic but behave in ways that feel natural and safe to humans—essential for the future of humanoid robotics.
