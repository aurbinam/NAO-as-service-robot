"""
Intelligence layer for NAO service robot.

4-layer architecture:
  Layer 1 - GenAIInterpreter : natural language -> structured intent
  Layer 2 - TaskPlanner      : intent -> ordered action plan
  Layer 3 - Executor         : action -> robot motion (wraps existing navigation)
  Layer 4 - ExecutionMonitor : progress tracking + replanning signals

Entry point: RobotBrain.handle_command(user_text)
"""
