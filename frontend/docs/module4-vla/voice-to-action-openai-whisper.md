---
sidebar_position: 2
title: Voice-to-Action (OpenAI Whisper)
---

# Voice-to-Action with OpenAI Whisper

## Introduction: Natural Voice Control for Robots

Voice control represents the most natural interface for human-robot interaction. Instead of typing commands or programming sequences, users simply speak their intentions: "Pick up the red block," "Follow me," or "Clean the kitchen." OpenAI Whisper, a state-of-the-art speech recognition model, enables robust voice-to-action systems that work across accents, languages, and noisy environments.

This chapter explores how to integrate Whisper with ROS 2 to create voice-controlled humanoid robots.

## Why Voice Control Matters for Robotics

### Advantages Over Traditional Interfaces

1. **Hands-Free Operation**: Users can control robots while performing other tasks
2. **Natural Interaction**: Speaking is more intuitive than typing or programming
3. **Accessibility**: Enables robot control for users with limited mobility
4. **Speed**: Voice commands are faster than navigating complex interfaces
5. **Contextual Communication**: Tone and emphasis convey additional meaning

### Real-World Use Cases

- **Manufacturing**: "Robot, hand me the 10mm wrench"
- **Healthcare**: "Bring wheelchair to patient in bed 3"
- **Elderly Care**: "Remind me to take medication at 3 PM"
- **Emergency Response**: "Search for survivors in sector B"
- **Home Assistance**: "Start cleaning while I'm at work"

## OpenAI Whisper: State-of-the-Art Speech Recognition

### What Makes Whisper Special

**Traditional ASR (Automatic Speech Recognition) vs. Whisper:**

| Feature | Traditional ASR | Whisper |
|---------|----------------|---------|
| Training Data | ~1,000 hours | 680,000 hours |
| Languages | Limited | 99 languages |
| Robustness | Poor in noise | Excellent |
| Accuracy | ~80-90% | ~95-98% |
| Accents | Struggles | Handles well |

### Whisper Model Architecture

**Encoder-Decoder Transformer:**
1. **Audio Encoder**: Processes audio waveform into features
2. **Decoder**: Generates text tokens from audio features
3. **Multitask Training**: Transcription, translation, language detection, timestamp prediction

### Model Sizes and Trade-offs

| Model | Parameters | Relative Speed | Relative Accuracy | Use Case |
|-------|-----------|----------------|-------------------|----------|
| tiny | 39M | 32x | Good | Edge devices (Jetson) |
| base | 74M | 16x | Better | Real-time applications |
| small | 244M | 6x | Great | Balanced performance |
| medium | 769M | 2x | Excellent | High accuracy needed |
| large | 1550M | 1x | Best | Offline processing |

**For robotics, we typically use `small` or `base` models for real-time performance.**

## Building a Voice-to-Action Pipeline

### Architecture Overview

```
[Microphone] → [Audio Capture] → [Whisper STT] → [Intent Parser] → [ROS 2 Actions]
     ↓              ↓                  ↓                ↓                  ↓
  Hardware      Audio Buffer      "Pick up cup"    ActionGoal        Robot Moves
```

### Step 1: Audio Capture with ROS 2

**ROS 2 Audio Capture Node:**

```python
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
import pyaudio
import numpy as np

class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_capture')
        self.publisher = self.create_publisher(AudioData, 'audio/input', 10)

        # PyAudio configuration
        self.audio_format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000  # Whisper expects 16kHz
        self.chunk = 1024

        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=self.audio_format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        self.get_logger().info('Audio capture started')
        self.timer = self.create_timer(0.1, self.capture_audio)

    def capture_audio(self):
        try:
            data = self.stream.read(self.chunk, exception_on_overflow=False)
            msg = AudioData()
            msg.data = list(data)
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Audio capture error: {str(e)}')

    def __del__(self):
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
```

### Step 2: Whisper Integration

**Whisper Transcription Node:**

```python
import whisper
import torch
from std_msgs.msg import String

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_transcription')

        # Load Whisper model
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = whisper.load_model("base", device=self.device)
        self.get_logger().info(f'Whisper model loaded on {self.device}')

        # Subscribers and publishers
        self.audio_sub = self.create_subscription(
            AudioData, 'audio/input', self.audio_callback, 10
        )
        self.text_pub = self.create_publisher(String, 'voice/transcription', 10)

        # Audio buffer
        self.audio_buffer = []
        self.buffer_duration = 3.0  # seconds
        self.sample_rate = 16000

    def audio_callback(self, msg):
        # Accumulate audio data
        self.audio_buffer.extend(msg.data)

        # Process when buffer is full
        samples_needed = int(self.buffer_duration * self.sample_rate)
        if len(self.audio_buffer) >= samples_needed:
            self.process_audio()

    def process_audio(self):
        # Convert buffer to numpy array
        audio_data = np.array(self.audio_buffer, dtype=np.int16)
        audio_data = audio_data.astype(np.float32) / 32768.0  # Normalize

        # Transcribe with Whisper
        result = self.model.transcribe(
            audio_data,
            language='en',
            task='transcribe',
            fp16=(self.device == 'cuda')
        )

        transcription = result['text'].strip()

        if transcription:
            self.get_logger().info(f'Transcribed: {transcription}')
            msg = String()
            msg.data = transcription
            self.text_pub.publish(msg)

        # Clear buffer
        self.audio_buffer = []
```

### Step 3: Intent Recognition and Command Parsing

**Command Parser Node:**

```python
import re
from robot_interfaces.msg import RobotCommand

class CommandParserNode(Node):
    def __init__(self):
        super().__init__('command_parser')

        self.text_sub = self.create_subscription(
            String, 'voice/transcription', self.parse_command, 10
        )
        self.command_pub = self.create_publisher(
            RobotCommand, 'robot/command', 10
        )

        # Define command patterns
        self.command_patterns = {
            'navigate': [
                r'go to (.*)',
                r'move to (.*)',
                r'navigate to (.*)'
            ],
            'grasp': [
                r'pick up (.*)',
                r'grab (.*)',
                r'take (.*)'
            ],
            'place': [
                r'put (.*) on (.*)',
                r'place (.*) on (.*)',
            ],
            'follow': [
                r'follow me',
                r'come with me'
            ],
            'stop': [
                r'stop',
                r'halt',
                r'freeze'
            ]
        }

    def parse_command(self, msg):
        text = msg.data.lower()
        self.get_logger().info(f'Parsing: {text}')

        # Match against patterns
        for action, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.match(pattern, text)
                if match:
                    self.execute_action(action, match)
                    return

        self.get_logger().warn(f'Unknown command: {text}')

    def execute_action(self, action, match):
        cmd = RobotCommand()
        cmd.action = action

        if action == 'navigate':
            cmd.target = match.group(1)
        elif action in ['grasp', 'pick']:
            cmd.object = match.group(1)
        elif action == 'place':
            cmd.object = match.group(1)
            cmd.location = match.group(2)

        self.command_pub.publish(cmd)
        self.get_logger().info(f'Command: {action} -> {cmd}')
```

### Step 4: Connecting to Robot Actions

**Action Executor Node:**

```python
from geometry_msgs.msg import PoseStamped
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient

class VoiceActionExecutor(Node):
    def __init__(self):
        super().__init__('voice_action_executor')

        self.command_sub = self.create_subscription(
            RobotCommand, 'robot/command', self.handle_command, 10
        )

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.gripper_client = ActionClient(self, GripperCommand, 'gripper_controller')

    def handle_command(self, cmd):
        if cmd.action == 'navigate':
            self.navigate_to(cmd.target)
        elif cmd.action == 'grasp':
            self.grasp_object(cmd.object)
        elif cmd.action == 'place':
            self.place_object(cmd.location)
        elif cmd.action == 'stop':
            self.emergency_stop()

    def navigate_to(self, target):
        goal_msg = NavigateToPose.Goal()
        # Convert target name to coordinates
        goal_msg.pose = self.lookup_location(target)
        self.nav_client.send_goal_async(goal_msg)
        self.get_logger().info(f'Navigating to {target}')

    def grasp_object(self, object_name):
        # First, detect object location
        object_pose = self.detect_object(object_name)

        # Navigate to object
        self.navigate_to(object_pose)

        # Execute grasp
        grasp_goal = GripperCommand.Goal()
        grasp_goal.command.position = 0.0  # Closed
        self.gripper_client.send_goal_async(grasp_goal)
```

## Advanced Features

### 1. Wake Word Detection

Implement always-listening mode with a wake word (e.g., "Hey Robot"):

```python
import pvporcupine

class WakeWordDetector(Node):
    def __init__(self):
        super().__init__('wake_word_detector')

        # Initialize Porcupine wake word detector
        self.porcupine = pvporcupine.create(
            keywords=['hey-robot'],
            sensitivities=[0.5]
        )

        self.listening = False

    def detect_wake_word(self, audio_frame):
        keyword_index = self.porcupine.process(audio_frame)

        if keyword_index >= 0:
            self.get_logger().info('Wake word detected!')
            self.listening = True
            # Start Whisper transcription
            self.activate_transcription()
```

### 2. Multilingual Support

Whisper supports 99 languages out of the box:

```python
# Automatic language detection
result = model.transcribe(audio, task='transcribe')
detected_language = result['language']
self.get_logger().info(f'Detected language: {detected_language}')

# Or specify language explicitly
result = model.transcribe(audio, language='es')  # Spanish
result = model.transcribe(audio, language='ur')  # Urdu
result = model.transcribe(audio, language='zh')  # Chinese
```

### 3. Confidence-Based Filtering

Reject low-confidence transcriptions:

```python
def process_with_confidence(self, audio):
    result = self.model.transcribe(
        audio,
        word_timestamps=True,
        condition_on_previous_text=False
    )

    # Calculate average confidence
    if 'segments' in result:
        avg_confidence = sum(
            seg.get('confidence', 0) for seg in result['segments']
        ) / len(result['segments'])

        if avg_confidence < 0.7:
            self.get_logger().warn('Low confidence, ignoring')
            return None

    return result['text']
```

### 4. Contextual Understanding with LLMs

Enhance voice commands with LLM reasoning:

```python
import openai

class ContextualVoiceController(Node):
    def __init__(self):
        super().__init__('contextual_voice')
        self.conversation_history = []

    def enhance_command(self, transcription):
        # Add context from previous commands
        prompt = f"""
        Robot conversation history: {self.conversation_history}
        User just said: "{transcription}"

        Extract the robot action as JSON:
        {{"action": "navigate|grasp|place", "target": "...", "parameters": {{...}}}}
        """

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}]
        )

        command = json.loads(response.choices[0].message.content)
        return command
```

## Deployment on Edge Devices

### Running Whisper on NVIDIA Jetson

**Optimized Inference:**

```python
# Use faster-whisper (optimized with CTranslate2)
from faster_whisper import WhisperModel

class OptimizedWhisperNode(Node):
    def __init__(self):
        super().__init__('optimized_whisper')

        # Load model with GPU acceleration
        self.model = WhisperModel(
            "base",
            device="cuda",
            compute_type="float16"  # Use half precision
        )

        self.get_logger().info('Optimized Whisper loaded')

    def transcribe(self, audio):
        segments, info = self.model.transcribe(
            audio,
            beam_size=1,  # Faster decoding
            language="en"
        )

        text = " ".join([segment.text for segment in segments])
        return text
```

**Performance Benchmarks:**

| Device | Model | Real-Time Factor | Latency |
|--------|-------|------------------|---------|
| Jetson Orin Nano | tiny | 0.1x | ~100ms |
| Jetson Orin Nano | base | 0.3x | ~300ms |
| Jetson AGX Orin | small | 0.2x | ~200ms |
| RTX 4090 | large | 0.05x | ~50ms |

*Real-Time Factor: < 1.0 means faster than real-time*

## Handling Edge Cases

### 1. Background Noise

**Noise Filtering:**

```python
import noisereduce as nr

def filter_noise(self, audio, sample_rate):
    # Reduce background noise
    reduced_noise = nr.reduce_noise(
        y=audio,
        sr=sample_rate,
        stationary=True,
        prop_decrease=0.8
    )
    return reduced_noise
```

### 2. Overlapping Speech

**Voice Activity Detection (VAD):**

```python
from webrtcvad import Vad

class VoiceActivityNode(Node):
    def __init__(self):
        super().__init__('vad')
        self.vad = Vad(3)  # Aggressiveness: 0-3

    def is_speech(self, audio_frame, sample_rate):
        return self.vad.is_speech(audio_frame, sample_rate)
```

### 3. Ambiguous Commands

**Clarification Requests:**

```python
def handle_ambiguous_command(self, cmd):
    if self.is_ambiguous(cmd):
        self.speak("I'm not sure what you meant. Could you clarify?")
        # Wait for clarification
        clarification = self.wait_for_response()
        return self.resolve_with_context(cmd, clarification)
```

## Safety Considerations

### 1. Emergency Stop

Always implement voice-activated emergency stop:

```python
EMERGENCY_KEYWORDS = ['stop', 'halt', 'freeze', 'emergency']

def check_emergency_stop(self, transcription):
    if any(keyword in transcription.lower() for keyword in EMERGENCY_KEYWORDS):
        self.emergency_stop()
        self.get_logger().warn('EMERGENCY STOP ACTIVATED')
        return True
    return False
```

### 2. Command Confirmation

For dangerous actions, require confirmation:

```python
def execute_dangerous_action(self, action):
    self.speak(f"Are you sure you want me to {action}?")
    confirmation = self.wait_for_confirmation(timeout=5.0)

    if confirmation and 'yes' in confirmation.lower():
        self.execute(action)
    else:
        self.speak("Action cancelled")
```

## Complete End-to-End Example

**Full Voice-Controlled Navigation System:**

```python
import rclpy
from rclpy.node import Node
import whisper
import pyaudio
import numpy as np

class VoiceNavigationRobot(Node):
    def __init__(self):
        super().__init__('voice_nav_robot')

        # Whisper model
        self.model = whisper.load_model("base")

        # Audio setup
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024
        )

        self.get_logger().info('Voice navigation ready')

    def listen_and_execute(self):
        self.get_logger().info('Listening for command...')

        # Capture 3 seconds of audio
        audio_buffer = []
        for _ in range(int(16000 / 1024 * 3)):
            data = self.stream.read(1024)
            audio_buffer.extend(np.frombuffer(data, dtype=np.int16))

        # Convert and normalize
        audio_array = np.array(audio_buffer, dtype=np.float32) / 32768.0

        # Transcribe
        result = self.model.transcribe(audio_array)
        command = result['text'].strip()
        self.get_logger().info(f'Command: {command}')

        # Execute
        self.execute_command(command)

    def execute_command(self, command):
        cmd_lower = command.lower()

        if 'forward' in cmd_lower:
            self.move_forward()
        elif 'backward' in cmd_lower:
            self.move_backward()
        elif 'left' in cmd_lower:
            self.turn_left()
        elif 'right' in cmd_lower:
            self.turn_right()
        elif 'stop' in cmd_lower:
            self.stop()
        else:
            self.get_logger().warn(f'Unknown command: {command}')

def main():
    rclpy.init()
    robot = VoiceNavigationRobot()

    try:
        while rclpy.ok():
            robot.listen_and_execute()
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()
```

## Performance Optimization Tips

1. **Use Smaller Models**: `tiny` or `base` for real-time applications
2. **GPU Acceleration**: Always use CUDA when available
3. **Batch Processing**: Process multiple audio chunks together
4. **Caching**: Cache common phrases and responses
5. **Voice Activity Detection**: Only process when speech is detected
6. **Quantization**: Use INT8 quantization for edge devices

## Key Takeaways

- **Whisper provides robust speech recognition** across languages and accents
- **Voice control enables natural human-robot interaction** without keyboards or screens
- **Edge deployment is feasible** with model optimization (faster-whisper, quantization)
- **Safety features are critical** (emergency stop, command confirmation)
- **Combining Whisper with LLMs** enables contextual understanding
- **Real-time performance requires** careful model selection and optimization

## Learning Objectives Achieved

By completing this chapter, you should be able to:

- Integrate OpenAI Whisper with ROS 2 for speech recognition
- Build a complete voice-to-action pipeline
- Implement command parsing and intent recognition
- Deploy voice control on edge devices (Jetson)
- Handle edge cases (noise, ambiguity, overlapping speech)
- Design safe voice-controlled robot systems

---

**Next**: We'll explore cognitive planning with LLMs, translating natural language instructions into complex multi-step ROS 2 action sequences.
