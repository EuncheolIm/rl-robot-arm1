# OpenAI Gym 2D Robot Arm Environment

**Description**

**RobotArm-V0**

By default, increment or decrement rate for both of joints are 0.01 radians. 

Reward Function:

* Robot will get penalty -1 if current distance between tip and target position is smaller than previous distance
* Robot will get reward 1 if cureent distance between tip and target position is > -epsilon and < epsilon, where epsilon = 10 pixels


Observation spaces (Continuous):


Action Spaces (Continuous): 
type Box((-1,),(1,),shape(3,))

* 0: joint 0 value (in range -1 to 1)
* 1: joint 1 value (in range -1 to 1)
* 2: joint 2 value (in range -1 to 1)

value will be scaled into minimum and maximum of joint angle

Reward Function:

* reward = -(distance_error+dist_ctrl) / 100

Terminal Condition:

* if target position is > -epsilon and < epsilon, where epsilon = 5 pixels

**How To Install**

```bash
git clone https://github.com/ekorudiawan/gym-robot-arm.git
cd gym-robot-arm
pip install -e .
```

**Dependencies**
* OpenAI Gym
* PyGame
* Scipy

**Testing Environment**

```python
import gym 

env = gym.make("robot-arm-v0")

for i_episode in range(num_steps):
    observation = env.reset()
    for t in range(1000):
        env.render()
        time.sleep(0.01)
        action = env.action_space.sample()
        observation, reward, done, info = env.step(action)
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            break
env.close()

```
