import numpy as np
import math
from gym import utils
from gym.envs.mujoco import mujoco_env


class RobotArmEnvV0(mujoco_env.MujocoEnv, utils.EzPickle):


    def __init__(self):

        self.current_error = -math.inf
        self.current_score = 0
        self.set_increment_rate(0.01)
        utils.EzPickle.__init__(self)
        mujoco_env.MujocoEnv.__init__(self, "/home/kist-robot2/eim/quarantine-robot/model/rl_tablebeside_copy.xml", 3)
    
    def set_increment_rate(self, rate):
        self.rate = rate
    
    def step(self, action):
        # min_theta = np.radians(0)
        # max_theta = np.radians(90)

        
        # theta = self.sim.data.qpos.ravel().copy() #ravel()은 다차원을 1차원으로 푸는 함수
        # theta[0] = np.interp(action[0], (self.low[0], self.high[0]), (min_theta, max_theta))
        # theta[1] = np.interp(action[1], (self.low[1], self.high[1]), (min_theta, max_theta))
        # theta[2] = np.interp(action[2], (self.low[2], self.high[2]), (min_theta, max_theta))
        

        #Calc reward

        vec = self.get_body_com("fingertip") - self.get_body_com("target")
        reward_dist = - np.linalg.norm(vec)
        reward_ctrl = -np.square(action).sum()
        self.do_simulation(action, self.frame_skip)

        fingertip_x,fingertip_y = self.get_body_com("fingertip")[0:2]
        target_x, target_y = self.get_body_com("target")[0:2]
        reward = (reward_dist + reward_ctrl )/ 100

        observation = self.observation()

        done = False
        info = self.extra_info()

        thershold = 0.01
        if (target_x - thershold) < fingertip_x and fingertip_x < (target_x + thershold):
            if (target_y - thershold) < fingertip_y and fingertip_y < (target_y + thershold):
                reward +=1
                self.current_score +=1
                if self.current_score > 20:
                    done = True
        # print("x : ",target_x - thershold, fingertip_x, target_x + thershold )
        # print("y : ",target_y - thershold, fingertip_y, target_y + thershold )
        # print("self.current_score : ",self.current_score)


        # if info["distance_error"] == 0.00:
        #     done = True

        return observation, reward, done, info
    
    def extra_info(self):
        reward_dist =  np.linalg.norm(self.get_body_com("fingertip") - self.get_body_com("target"))

        info = {
            'distance_error': reward_dist,
            'target_position': self.get_body_com("target"),
            'current_position': self.get_body_com("fingertip")
        }

        return info
    def observation(self):
        theta = self.sim.data.qpos.flat
        return np.concatenate(
            [
                np.cos(theta),
                np.sin(theta),
                self.sim.data.qpos.flat, #joint pos
                self.sim.data.qvel.ravel(), #joint velocity
                self.get_body_com("fingertip") - self.get_body_com("target"), #distance target - ee
            ]
        )

    def viewer_setup(self):
        self.viewer.cam.trackbodyid = 0

    def reset_model(self):
        self.currens_score = 0
        qpos = self.init_qpos + self.np_random.uniform(low=-0.1, high=0.1, size=self.model.nq)
        qvel = self.init_qvel + self.np_random.uniform(low=-0.005, high=0.005, size=self.model.nv)

        self.set_state(qpos, qvel)
        return self.observation()

# class RobotArmEnvV11(RobotArmEnvV0):
#     def __init__(self):
#         super(RobotArmEnvV11, self).__init__()

#         self.min_action = -1.0
#         self.max_action = 1.0
#         self.min_theta = np.radians(0)
#         self.max_theta = np.radians(90)
#         self.action_space = spaces.Box(low=self.min_action, high=self.max_action, shape=(2,), dtype=np.float32)

#     def step(self, action):
#         theta0 = np.interp(action[0], (self.min_action, self.max_action), (self.min_theta, self.max_theta))
#         theta1 = np.interp(action[1], (self.min_action, self.max_action), (self.min_theta, self.max_theta))
#         self.theta[0] = theta0
#         self.theta[1] = theta1
#         # Calc reward
#         vec = self.get_body_com("fingertip") - self.get_body_com("target")
#         reward_dist = -np.linalg.norm(vec)

#         # Sharp reward
#         reward = -reward_dist / 100
#         done = False
#         epsilon = 5
#         if (reward_dist > -epsilon and reward_dist < epsilon):
#             done = True

#         observation = self._get_obs()
#         info = {
#             'distance_error': reward_dist,
#             'target_position':self.get_body_com("target"),
#             'current_position': self.get_body_com("fingertip")
#         }
#         return observation, reward, done, info
    # def __init__(self,
    #     link_length = [0.35, 0.35, 0.2],
    #     goal_y_pos =0.45,
    #     goal_x_pos = -0.42,
    #     exclude_current_positions_from_observation=True,

    #     ):
    #     self.link_length = link_length
    #     self.goal_y_pos = goal_y_pos
    #     self.goal_x_pos = goal_x_pos
    #     self._exclude_current_positions_from_observation = (
    #         exclude_current_positions_from_observation
    #     )

    #     utils.EzPickle.__init__(**locals())
    #     mujoco_env.MujocoEnv.__init__(self, "/home/kist-robot2/eim/quarantine-robot/model/rl_tablebeside_copy.xml", 2)
    # def forward_kinematics(self, _q_pos):
  
    #     x_goal = np.full([3, 1], None)
    #     x_goal[0] = self.link_length[0] * np.cos(_q_pos[1]) + self.link_length[1] * np.cos(_q_pos[1] + _q_pos[2]) + self.link_length[2] * np.cos(_q_pos[1] + _q_pos[2] + _q_pos[3]) # m
    #     x_goal[1] = self.link_length[0] * np.sin(_q_pos[1]) + self.link_length[1] * np.sin(_q_pos[1] + _q_pos[2]) + self.link_length[2] * np.sin(_q_pos[1] + _q_pos[2] + _q_pos[3]) #/ m
    #     x_goal[2] = _q_pos[1] + _q_pos[2] + _q_pos[3]; #degree

    #     return x_goal
    
    # def init_qposs(self):
    #     return np.array([0,0.384274,1.61203,-1.99631])

    # def step(self, action): ##planning
    #     end_xy = self.get_body_com("end")[:2]
    #     target_xy = self.get_body_com("target")[:2]
    
    #     qpos_before = self.sim.data.qpos.copy()
    #     y_before = self.forward_kinematics(qpos_before)[1]

    #     reward_dist = -np.linalg.norm(end_xy - target_xy)   #end-target사이의 유클리드 놈 거리
    #     reward_ctrl = 1e-3 * np.sum(np.square(action)) * 2 #값이 너무 튀면 rewad_ctrl '-'
    #     reward_1 =  reward_dist + reward_ctrl
    #     # simulation #
    #     self.do_simulation(action, self.frame_skip)
    #     # print(reward_dist)
    #     qpos_after = self.sim.data.qpos.copy()
    #     x_after, y_after = self.forward_kinematics(qpos_after)[:2]
        
    #     y_error = y_before - y_after 
    #     if y_error > -0.05 and y_error < 0.05: #y축 움직임 거의 없이 직선으로만 움직이기
    #         reward_y = 10
    #     else:
    #         reward_y = -10
    #         return reward_y
    #     # print("y_error : ",y_error)
    #     rewards = reward_1 + reward_y
    #     observ = self._get_obs()
    #     done = bool(y_after >target_xy[1] or x_after < target_xy[0])
    #     # done = bool(y_after >= self.goal_y_pos + 0.1 and x_after <= self.goal_x_pos + 0.1 ) #target방향 -x axis
    #     info = { 
    #         "x_after_pos" : x_after,
    #     }

    #     return observ, rewards, done, info #reward_ctrl=reward_ctrl)

    # def _get_obs(self):
    #     theta = self.sim.data.qpos #q0,q1,q2,q3
    #     # theta[0] = 0
    #     velocity = self.sim.data.qvel #q0dot, q1dot, q2dot, q3dot
    #     return np.concatenate(
    #         [
    #             np.cos(theta),
    #             np.sin(theta),
    #             theta,
    #             velocity,
    #             self.get_body_com("end") - self.get_body_com("target"),
    #         ]
    #     )
    


    # def reset_model(self):
    #     # qpos = self.init_qpos #set initial_pos
    #     qpos = (
    #          np.array([0,0.384274,1.61203,-1.99631]) + self.np_random.randn(self.init_qpos.shape[0]) * 0.02
    #     )
      
    #     qvel = self.init_qvel +self.np_random.uniform(low=-0.005, high=0.005, size=self.model.nv)
    #     self.set_state(qpos, qvel)

    #     observation = self._get_obs() 
    #     return observation


    # def viewer_setup(self):
    #     self.viewer.cam.trackbodyid = 0 #id fof the body to track
    #     self.viewer.cam.distance = self.model.stat.extent * 0.6 # small = zoom -in large = zoom -ouy
    #     self.viewer.cam.lookat[0] = 0.1 # camera move to x - axis
    #     self.viewer.cam.lookat[2] = 2.2 # camera move to z - axis
    #     self.viewer.cam.elevation = -90 # camera rotate
    #     self.viewer.cam.azimuth = 180 # camera rotate the camera vertical axis
