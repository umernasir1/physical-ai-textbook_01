---
sidebar_position: 3
title: Cognitive Planning (Translating Natural Language to ROS 2 Actions)
---

# Cognitive Planning: Translating Natural Language to ROS 2 Actions

## Introduction: From Words to Executable Plans

The true power of Language-Vision-Action (VLA) systems emerges when robots can understand high-level human instructions and autonomously decompose them into executable action sequences. Instead of programming every step, we simply tell the robot: "Clean the room," and it figures out the rest.

This chapter explores **cognitive planning**—using Large Language Models (LLMs) to translate natural language commands into structured ROS 2 action sequences that robots can execute.

## The Cognitive Planning Challenge

### From High-Level Goals to Low-Level Actions

**Human Instruction:**
> "Make me a cup of coffee"

**What the Robot Must Figure Out:**
1. Navigate to the kitchen
2. Locate the coffee machine
3. Check if there's a cup nearby
4. If no cup: search for cup, grasp cup, place under spout
5. Verify coffee machine has water and beans
6. Press the brew button
7. Wait for brewing to complete
8. Grasp the filled cup
9. Navigate to the user
10. Hand over the cup safely

**The Challenge:** Bridging the semantic gap between human intent and robot capabilities.

### Why Traditional Planning Falls Short

**Rule-Based Systems:**
```python
# Brittle and inflexible
if command == "make coffee":
    navigate("kitchen")
    find_object("coffee_machine")
    press_button("brew")
    # Breaks if coffee machine is different or cup is missing
```

**Limitations:**
- Cannot handle variations or exceptions
- Requires explicit programming for every scenario
- No common-sense reasoning
- Poor generalization to new situations

## LLM-Based Cognitive Planning

### How LLMs Enable Flexible Planning

LLMs bring several critical capabilities:

1. **World Knowledge**: Understanding of objects, their properties, and typical uses
2. **Common-Sense Reasoning**: Inferring implicit requirements (e.g., "coffee needs a cup")
3. **Task Decomposition**: Breaking complex goals into logical steps
4. **Contextual Adaptation**: Adjusting plans based on environment state
5. **Error Recovery**: Generating alternative plans when failures occur

### Architecture: LLM Planning Layer

```
[Natural Language Command]
         ↓
[LLM Planner] ← [Scene Understanding] ← [Robot Sensors]
         ↓
[Structured Action Plan (JSON)]
         ↓
[ROS 2 Action Bridge]
         ↓
[ROS 2 Action Servers] → [Robot Execution]
```

## Building the Cognitive Planner

### Step 1: Define Action Primitives

First, define the low-level actions your robot can perform:

```python
# action_primitives.py
from dataclasses import dataclass
from typing import Dict, Any, List

@dataclass
class ActionPrimitive:
    """Base class for robot actions"""
    action_type: str
    parameters: Dict[str, Any]
    preconditions: List[str]
    effects: List[str]

# Define available actions
AVAILABLE_ACTIONS = {
    "navigate": ActionPrimitive(
        action_type="navigate",
        parameters={"target": "pose or location name"},
        preconditions=["robot_is_mobile"],
        effects=["robot_at_target"]
    ),
    "grasp": ActionPrimitive(
        action_type="grasp",
        parameters={"object": "object_id", "approach": "top|side"},
        preconditions=["object_in_reach", "gripper_empty"],
        effects=["object_grasped"]
    ),
    "place": ActionPrimitive(
        action_type="place",
        parameters={"location": "target_surface"},
        preconditions=["object_grasped"],
        effects=["object_placed", "gripper_empty"]
    ),
    "detect": ActionPrimitive(
        action_type="detect",
        parameters={"object_class": "object category"},
        preconditions=["camera_active"],
        effects=["object_located"]
    ),
    "manipulate": ActionPrimitive(
        action_type="manipulate",
        parameters={"device": "button|switch|handle", "action": "press|flip|turn"},
        preconditions=["device_in_reach"],
        effects=["device_state_changed"]
    )
}
```

### Step 2: Create the LLM Planning Prompt

Design a prompt that guides the LLM to generate valid plans:

```python
# cognitive_planner.py
import json
import openai
from typing import List, Dict

class CognitivePlanner:
    def __init__(self, model="gpt-4"):
        self.model = model
        self.action_library = AVAILABLE_ACTIONS

    def create_system_prompt(self):
        actions_json = json.dumps(
            {k: v.__dict__ for k, v in self.action_library.items()},
            indent=2
        )

        return f"""You are a robotic task planner. Given a high-level user command and the current environment state, generate a detailed action sequence.

Available Actions:
{actions_json}

Output Format (JSON):
{{
  "plan": [
    {{
      "step": 1,
      "action": "action_type",
      "parameters": {{}},
      "reasoning": "why this step is needed"
    }}
  ],
  "preconditions_check": ["condition1", "condition2"],
  "estimated_duration": "time in seconds",
  "failure_recovery": "what to do if plan fails"
}}

Rules:
1. Break complex tasks into atomic actions
2. Check preconditions before actions
3. Consider object relationships and physics
4. Plan for error cases
5. Minimize unnecessary movements
6. Ensure safety (e.g., check for humans before moving)
"""

    def generate_plan(self, user_command: str, scene_state: Dict) -> Dict:
        """Generate action plan from natural language command"""

        user_prompt = f"""
User Command: "{user_command}"

Current Environment State:
{json.dumps(scene_state, indent=2)}

Generate a detailed action plan to accomplish this task.
"""

        response = openai.ChatCompletion.create(
            model=self.model,
            messages=[
                {"role": "system", "content": self.create_system_prompt()},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.2  # Lower temperature for more consistent plans
        )

        plan = json.loads(response.choices[0].message.content)
        return self.validate_plan(plan)

    def validate_plan(self, plan: Dict) -> Dict:
        """Validate that plan uses only available actions"""
        for step in plan['plan']:
            if step['action'] not in self.action_library:
                raise ValueError(f"Invalid action: {step['action']}")

            # Check parameter requirements
            required_params = self.action_library[step['action']].parameters
            for param in required_params:
                if param not in step['parameters']:
                    raise ValueError(f"Missing parameter {param} for {step['action']}")

        return plan
```

### Step 3: Scene Understanding Integration

Provide the LLM with current environment information:

```python
# scene_understanding.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection3DArray
import cv2
import numpy as np

class SceneUnderstanding(Node):
    def __init__(self):
        super().__init__('scene_understanding')

        # Subscribers for perception data
        self.camera_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self.detection_sub = self.create_subscription(
            Detection3DArray, '/detected_objects', self.detection_callback, 10
        )

        self.detected_objects = []
        self.robot_pose = None

    def get_scene_state(self) -> Dict:
        """Generate structured scene description for LLM"""
        return {
            "robot_location": {
                "position": self.robot_pose['position'] if self.robot_pose else "unknown",
                "orientation": self.robot_pose['orientation'] if self.robot_pose else "unknown"
            },
            "detected_objects": [
                {
                    "id": obj.id,
                    "class": obj.object_class,
                    "position": {
                        "x": obj.pose.position.x,
                        "y": obj.pose.position.y,
                        "z": obj.pose.position.z
                    },
                    "distance": self.calculate_distance(obj.pose),
                    "graspable": self.is_graspable(obj)
                }
                for obj in self.detected_objects
            ],
            "nearby_locations": ["kitchen", "living_room", "table", "counter"],
            "gripper_state": "empty",  # or "holding_object"
            "battery_level": 85,
            "obstacles": self.detect_obstacles()
        }

    def detection_callback(self, msg):
        self.detected_objects = msg.detections

    def is_graspable(self, obj) -> bool:
        """Determine if object can be grasped"""
        # Check size, position, etc.
        size = obj.bbox.size
        return (0.02 < size.x < 0.3 and
                0.02 < size.y < 0.3 and
                0.02 < size.z < 0.3)
```

### Step 4: ROS 2 Action Bridge

Translate LLM plans into ROS 2 actions:

```python
# action_bridge.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from control_msgs.action import GripperCommand
from manipulation_msgs.action import PickupObject, PlaceObject

class ActionBridge(Node):
    def __init__(self):
        super().__init__('action_bridge')

        # Create action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pickup_client = ActionClient(self, PickupObject, 'pickup_object')
        self.place_client = ActionClient(self, PlaceObject, 'place_object')
        self.gripper_client = ActionClient(self, GripperCommand, 'gripper_controller')

    async def execute_plan(self, plan: Dict):
        """Execute LLM-generated plan step by step"""
        for step in plan['plan']:
            self.get_logger().info(f"Executing step {step['step']}: {step['action']}")
            self.get_logger().info(f"Reasoning: {step['reasoning']}")

            success = await self.execute_action(step)

            if not success:
                self.get_logger().error(f"Step {step['step']} failed")
                # Attempt recovery
                recovery_success = await self.attempt_recovery(step, plan)
                if not recovery_success:
                    self.get_logger().error("Recovery failed, aborting plan")
                    return False

        self.get_logger().info("Plan executed successfully")
        return True

    async def execute_action(self, step: Dict) -> bool:
        """Execute single action"""
        action_type = step['action']
        params = step['parameters']

        if action_type == 'navigate':
            return await self.navigate(params['target'])
        elif action_type == 'grasp':
            return await self.grasp(params['object'])
        elif action_type == 'place':
            return await self.place(params['location'])
        elif action_type == 'detect':
            return await self.detect(params['object_class'])
        elif action_type == 'manipulate':
            return await self.manipulate(params['device'], params['action'])
        else:
            self.get_logger().warn(f"Unknown action: {action_type}")
            return False

    async def navigate(self, target) -> bool:
        """Execute navigation action"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.lookup_pose(target)

        self.get_logger().info(f'Navigating to {target}')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False

        result = await goal_handle.get_result_async()
        return result.result.error_code == 0

    async def grasp(self, object_id) -> bool:
        """Execute grasp action"""
        goal_msg = PickupObject.Goal()
        goal_msg.object_id = object_id

        self.get_logger().info(f'Grasping object {object_id}')
        send_goal_future = self.pickup_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            return False

        result = await goal_handle.get_result_async()
        return result.result.success
```

### Step 5: Complete Integration Example

Putting it all together:

```python
# cognitive_robot_controller.py
import rclpy
from rclpy.node import Node
import asyncio

class CognitiveRobotController(Node):
    def __init__(self):
        super().__init__('cognitive_robot_controller')

        # Initialize components
        self.planner = CognitivePlanner()
        self.scene_understanding = SceneUnderstanding()
        self.action_bridge = ActionBridge()

        self.get_logger().info('Cognitive robot controller initialized')

    async def execute_natural_language_command(self, command: str):
        """Main pipeline: Command → Plan → Execute"""

        self.get_logger().info(f'Received command: {command}')

        # 1. Understand current scene
        scene_state = self.scene_understanding.get_scene_state()
        self.get_logger().info(f'Scene state: {scene_state}')

        # 2. Generate plan with LLM
        self.get_logger().info('Generating plan with LLM...')
        plan = self.planner.generate_plan(command, scene_state)
        self.get_logger().info(f'Generated plan: {json.dumps(plan, indent=2)}')

        # 3. Execute plan
        self.get_logger().info('Executing plan...')
        success = await self.action_bridge.execute_plan(plan)

        if success:
            self.get_logger().info('Task completed successfully')
        else:
            self.get_logger().error('Task execution failed')

        return success

async def main():
    rclpy.init()
    controller = CognitiveRobotController()

    # Example commands
    commands = [
        "Pick up the red block and place it on the table",
        "Go to the kitchen and bring me a cup",
        "Find all the books and stack them on the shelf"
    ]

    for cmd in commands:
        await controller.execute_natural_language_command(cmd)
        await asyncio.sleep(2)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
```

## Advanced Planning Techniques

### 1. Hierarchical Task Planning

Break complex tasks into hierarchies:

```python
def generate_hierarchical_plan(self, command: str, scene_state: Dict) -> Dict:
    """Generate multi-level plan"""

    # Level 1: High-level goals
    high_level_prompt = f"""
    Break down this task into 3-5 major subtasks:
    Task: {command}

    Output as JSON list of subtasks.
    """

    subtasks = self.llm_query(high_level_prompt)

    # Level 2: Detailed actions for each subtask
    detailed_plan = []
    for subtask in subtasks:
        detailed_actions = self.generate_plan(subtask, scene_state)
        detailed_plan.append({
            "subtask": subtask,
            "actions": detailed_actions
        })

    return detailed_plan
```

### 2. Dynamic Replanning

Adapt plans when environment changes:

```python
class AdaptivePlanner(CognitivePlanner):
    def __init__(self):
        super().__init__()
        self.current_plan = None
        self.executed_steps = []

    async def execute_with_replanning(self, command: str):
        """Execute plan with dynamic replanning"""

        while not self.task_completed():
            # Get current state
            scene_state = self.scene_understanding.get_scene_state()

            # Generate/update plan
            if self.needs_replanning(scene_state):
                self.get_logger().info('Replanning due to environment change')
                self.current_plan = self.planner.generate_plan(
                    command,
                    scene_state,
                    completed_steps=self.executed_steps
                )

            # Execute next step
            next_step = self.current_plan['plan'][len(self.executed_steps)]
            success = await self.action_bridge.execute_action(next_step)

            if success:
                self.executed_steps.append(next_step)
            else:
                # Trigger replanning
                self.current_plan = None
```

### 3. Multi-Modal Reasoning

Combine vision and language for better understanding:

```python
def generate_multimodal_plan(self, command: str, image: np.ndarray, scene_state: Dict):
    """Use both vision and language"""

    # Encode image
    vision_model = VisionTransformer()
    visual_features = vision_model.encode(image)

    # Create multimodal prompt
    prompt = f"""
    Task: {command}
    Scene state: {scene_state}
    Visual features: {visual_features}

    Based on both the textual description and visual information,
    generate an action plan.
    """

    return self.llm_query(prompt)
```

### 4. Learning from Failures

Improve plans based on execution outcomes:

```python
class LearningPlanner(CognitivePlanner):
    def __init__(self):
        super().__init__()
        self.experience_memory = []

    def record_experience(self, command: str, plan: Dict, outcome: bool):
        """Store execution experience"""
        self.experience_memory.append({
            "command": command,
            "plan": plan,
            "success": outcome,
            "timestamp": time.time()
        })

    def generate_plan_with_experience(self, command: str, scene_state: Dict):
        """Use past experiences to improve planning"""

        # Find similar past tasks
        similar_experiences = self.find_similar_tasks(command)

        prompt = f"""
        Task: {command}
        Scene: {scene_state}

        Past similar tasks and outcomes:
        {json.dumps(similar_experiences, indent=2)}

        Generate a plan that learns from past successes and failures.
        """

        return self.llm_query(prompt)
```

## Safety and Validation

### Plan Validation Before Execution

```python
class SafetyValidator:
    def __init__(self):
        self.safety_rules = self.load_safety_rules()

    def validate_plan(self, plan: Dict, scene_state: Dict) -> tuple[bool, str]:
        """Validate plan for safety"""

        for step in plan['plan']:
            # Check collision risk
            if self.has_collision_risk(step, scene_state):
                return False, f"Collision risk in step {step['step']}"

            # Check force limits
            if step['action'] == 'manipulate':
                if not self.validate_force_limits(step):
                    return False, "Excessive force detected"

            # Check human proximity
            if self.human_too_close(step, scene_state):
                return False, "Human in workspace"

            # Check workspace boundaries
            if not self.in_workspace(step):
                return False, "Action outside safe workspace"

        return True, "Plan validated"

    def human_too_close(self, step: Dict, scene_state: Dict) -> bool:
        """Check if humans are too close to planned action"""
        human_objects = [obj for obj in scene_state['detected_objects']
                        if obj['class'] == 'person']

        action_location = step['parameters'].get('target') or step['parameters'].get('location')

        for human in human_objects:
            distance = self.calculate_distance(action_location, human['position'])
            if distance < 0.5:  # 50cm safety margin
                return True

        return False
```

## Performance Considerations

### Latency Optimization

```python
class FastCognitivePlanner:
    def __init__(self):
        self.planner = CognitivePlanner()
        self.plan_cache = {}

    def generate_plan_fast(self, command: str, scene_state: Dict):
        """Optimize planning latency"""

        # 1. Check cache for similar commands
        cache_key = self.compute_cache_key(command, scene_state)
        if cache_key in self.plan_cache:
            cached_plan = self.plan_cache[cache_key]
            # Adapt cached plan to current state
            return self.adapt_plan(cached_plan, scene_state)

        # 2. Use smaller, faster LLM for simple commands
        if self.is_simple_command(command):
            plan = self.planner.generate_plan(command, scene_state, model="gpt-3.5-turbo")
        else:
            plan = self.planner.generate_plan(command, scene_state, model="gpt-4")

        # 3. Cache the plan
        self.plan_cache[cache_key] = plan

        return plan
```

## Real-World Example: Kitchen Assistant Robot

Complete implementation:

```python
# kitchen_assistant.py
class KitchenAssistant(CognitiveRobotController):
    def __init__(self):
        super().__init__()
        self.kitchen_objects = ['cup', 'plate', 'spoon', 'coffee_machine', 'fridge']

    async def process_cooking_command(self, command: str):
        """Handle cooking-related commands"""

        # Enhanced scene understanding for kitchen
        scene_state = self.scene_understanding.get_scene_state()
        scene_state['kitchen_appliances'] = self.detect_appliances()
        scene_state['ingredients'] = self.detect_ingredients()

        # Generate specialized kitchen plan
        system_prompt = self.get_kitchen_prompt()
        plan = self.planner.generate_plan(command, scene_state, system_prompt)

        # Execute with kitchen-specific safety checks
        return await self.execute_kitchen_plan(plan)

    def get_kitchen_prompt(self):
        return """You are a kitchen assistant robot. Consider:
        - Food safety (proper temperatures, cross-contamination)
        - Appliance operation (preheating, timing)
        - Ingredient preparation order
        - Cleanup procedures
        """

# Example usage
async def demo():
    robot = KitchenAssistant()

    commands = [
        "Make me a sandwich",
        "Heat up the leftovers in the microwave",
        "Clean the counter and put away the dishes"
    ]

    for cmd in commands:
        await robot.process_cooking_command(cmd)
```

## Key Takeaways

- **Cognitive planning bridges high-level intent and low-level actions** using LLM reasoning
- **Structured prompts guide LLMs** to generate valid, executable plans
- **Scene understanding provides critical context** for planning decisions
- **ROS 2 actions serve as the execution layer** for LLM-generated plans
- **Safety validation is mandatory** before executing any plan
- **Dynamic replanning enables adaptation** to changing environments
- **Experience memory improves planning** over time

## Learning Objectives Achieved

By completing this chapter, you should be able to:

- Design cognitive planning systems using LLMs
- Create structured prompts for task decomposition
- Integrate scene understanding with planning
- Bridge LLM outputs to ROS 2 actions
- Implement safety validation for AI-generated plans
- Build adaptive replanning systems
- Optimize planning performance for real-time robotics

---

**Next**: In the capstone project, we'll integrate everything—vision, language, and action—to build a fully autonomous humanoid robot.
