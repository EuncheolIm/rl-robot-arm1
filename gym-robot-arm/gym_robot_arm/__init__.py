#from gym.envs.registration import load_env_plugins as _load_env_plugins
from gym.envs.registration import make, register, registry, spec

#_load_env_plugins()

register(
    id="robot-arm-v0",
    entry_point="gym_robot_arm.envs.robot_arm_env:RobotArmEnvV0",
    max_episode_steps=100,
    #reward_threshold=-3.75,
)
