from deepbots.supervisor.controllers.robot_supervisor_env import RobotSupervisorEnv

from PPO_agent import PPOAgent, Transition

from gym.spaces import Box, Discrete
import numpy as np

X = -5
Y = -2
Z = 0.5

CONTROLS = [(600.0, 600.0, 600.0, 600.0), (40.0, 40.0, 40.0, 40.0), (60.0, 60.0, 60.0, 60.0),
           (100.0, 100.0, 100.0, 100.0), 
        #    (300.0, 0.0, 0.0, 0.0), (0.0, 300.0, 0.0, 0.0),
        #    (0.0, 0.0, 300.0, 0.0), (0.0, 0.0, 0.0, 300.0), 
           (0.0, 0.0, 0.0, 0.0),
        #    (600.0, 600.0, 600.0, 300.0), (600.0, 600.0, 300.0, 600.0), 
        #    (600.0, 300.0, 600.0, 600.0), (300.0, 600.0, 600.0, 600.0)
           ]

def normalize_to_range(value, min_val, max_val, new_min, new_max, clip=False):
    """
    Normalize value to a specified new range by supplying the current range.

    :param value: value to be normalized
    :param min_val: value's min value, value ∈ [min_val, max_val]
    :param max_val: value's max value, value ∈ [min_val, max_val]
    :param new_min: normalized range min value
    :param new_max: normalized range max value
    :param clip: whether to clip normalized value to new range or not
    :return: normalized value ∈ [new_min, new_max]
    """
    value = float(value)
    min_val = float(min_val)
    max_val = float(max_val)
    new_min = float(new_min)
    new_max = float(new_max)

    if clip:
        return np.clip((new_max - new_min) / (max_val - min_val) * (value - max_val) + new_max, new_min, new_max)
    else:
        return (new_max - new_min) / (max_val - min_val) * (value - max_val) + new_max

# N = 2
# QUANTIZATION_N = 2**N


class UAVRobot(RobotSupervisorEnv):
    def __init__(self):
        super().__init__()
        # observation: x,y,z Linear velocities vx,vy,vz Quaternions: w, x,y
        # TODO: add angular velocities
        self.observation_space = Box(low=np.array([-1, -1, -1, -1, -1, -1, 0, 0, 0, -1, -1, -1]), 
                                     high=np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]),
                                     dtype=np.float64)
        # self.action_space = Box(low=np.array([0, 0, 0, 0]),
        #                              high=np.array([600, 600, 600, 600]),
        #                              dtype=np.float64)

        # self.action_space = Discrete(QUANTIZATION_N**4)
        self.action_space = Discrete(len(CONTROLS))

        self.robot = self.getSelf()
        print(f"{self.robot.__dict__}")
        self.timestep = int(self.getBasicTimeStep())
        # Initialize motors
        self.m1_motor = self.getDevice("m1_motor")
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = self.getDevice("m2_motor")
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = self.getDevice("m3_motor")
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = self.getDevice("m4_motor")
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)

        self.imu = self.getDevice("inertial_unit")
        self.imu.enable(self.timestep)
        self.steps_per_episode = 2000

        self.episode_score = 0

        self.episode_score_list = []
        self.floor_reset = False
        self.counter = 0
        # self.action_prev = np.ones((4,4), dtype=np.float64)*600.0


    def get_observations(self):
        # Position
        UAV_positionX = np.tanh(self.robot.getPosition()[0] - X)
        UAV_positionY = np.tanh(self.robot.getPosition()[1] - Y)
        UAV_positionZ = np.tanh(self.robot.getPosition()[2] - Z)
        # Linear velocity
        UAV_velocityX = np.tanh(self.robot.getVelocity()[0])
        UAV_velocityY = np.tanh(self.robot.getVelocity()[1])
        UAV_velocityZ = np.tanh(self.robot.getVelocity()[2])

        # Angular pos
        UAV_angularW = np.tanh(self.imu.getRollPitchYaw()[0])
        UAV_angularX = np.tanh(self.imu.getRollPitchYaw()[1])
        UAV_angularY = np.tanh(self.imu.getRollPitchYaw()[2])

        # Angular pos
        UAV_angular_velW = np.tanh(self.robot.getVelocity()[3])
        UAV_angular_velX = np.tanh(self.robot.getVelocity()[4])
        UAV_angular_velY = np.tanh(self.robot.getVelocity()[5])
        # UAV_angularZ = self.imu.getRollPitchYaw()[3]

        return [UAV_positionX, UAV_positionY, UAV_positionZ, UAV_velocityX, 
                UAV_velocityY, UAV_velocityZ, UAV_angularW, UAV_angularX, UAV_angularY, 
                UAV_angular_velW, UAV_angular_velX, UAV_angular_velY]#, UAV_angularZ
    
    def get_default_observation(self):
        # This method just returns a zero vector as a default observation
        return [0.0 for _ in range(self.observation_space.shape[0])]
    
    def get_reward(self, action=None):
        C = 10
        x = abs(self.robot.getPosition()[0] - X)**2*C
        y = abs(self.robot.getPosition()[1] - Y)**2*C
        z = abs(self.robot.getPosition()[2] - Z)**2*C
        angular_t = np.linalg.norm(np.array([self.robot.getVelocity()[3], self.robot.getVelocity()[4], self.robot.getVelocity()[5]]))
        return 30 - x -y- z - angular_t/300 + (-1*self.counter if self.robot.getPosition()[2]<0.1 else 0)
    
    def is_done(self):
        if((-2<(self.robot.getPosition()[0]-X)<2) and (-2<(self.robot.getPosition()[1]-Y)<2)
            and (-1<(self.robot.getPosition()[2]-Z)<1) ):
            self.counter += 1
            # print(self.counter)
            if (self.robot.getPosition()[2]>0.25):
                self.floor_reset = True
            if (self.robot.getPosition()[2]<0.1 and self.floor_reset):
                self.floor_reset = False
                self.counter = 0
                print(f"vel: {self.robot.getVelocity()[5]}")
                return True
            if ((self.counter>200) and (self.robot.getPosition()[2]<0.1) and (not self.floor_reset)):
                self.counter = 0
                self.floor_reset = False
                print(f"vel: {self.robot.getVelocity()[5]}, counter: {self.counter}")
                return True
            return False
        
        print(f"vel: {self.robot.getVelocity()[5]}")
        self.counter = 0
        self.floor_reset = False
        return True
    
    def solved(self):
        return False
    
    def get_info(self):
        return None

    def render(self, mode='human'):
        pass

    def apply_action(self, action): # []
        action = int(action[0])
        lst = [0,0,0,0]
        # for i in range(4):
        #     lst[i] = (600.0/np.float64(QUANTIZATION_N))*(action%QUANTIZATION_N)
        #     action = action//QUANTIZATION_N
        action = CONTROLS[action]
        # action = lst

        #FIR
        # act = np.mean(self.action_prev,axis=0)
        act = action

        self.m1_motor.setVelocity(-act[0])
        self.m2_motor.setVelocity(-act[1])
        self.m3_motor.setVelocity(-act[2])
        self.m4_motor.setVelocity(-act[3])

        # self.action_prev = np.concat((self.action_prev[1:,:],np.array(action).reshape(1,-1)), axis=0)


if __name__ == '__main__':
    env = UAVRobot()
    agent = PPOAgent(number_of_inputs=env.observation_space.shape[0], number_of_actor_outputs=env.action_space.n)
    solved = False
    episode_count = 0
    episode_limit = 2000000000
    while not solved and episode_count < episode_limit:
        observation = env.reset()  # Reset robot and get starting observation
        env.episode_score = 0
        for step in range(env.steps_per_episode):
            # In training mode the agent samples from the probability distribution, naturally implementing exploration
            selected_action, action_prob = agent.work(observation, type_="selectAction")
            new_observation, reward, done, info = env.step([selected_action])

            # Save the current state transition in agent's memory
            trans = Transition(observation, selected_action, action_prob, reward, new_observation)
            agent.store_transition(trans)

            if done:
                # Save the episode's score
                env.episode_score_list.append(env.episode_score)
                agent.train_step(batch_size=step + 1)
                solved = env.solved()  # Check whether the task is solved
                break

            env.episode_score += reward  # Accumulate episode reward
            observation = new_observation  # observation for next step is current step's new_observation
        else:
            print("end of episode")
        print("Episode #", episode_count, "score:", env.episode_score)
        episode_count += 1  # Increment episode counter
    if not solved:
        print("Task is not solved, deploying agent for testing...")
    elif solved:
        print("Task is solved, deploying agent for testing...")

    observation = env.reset()
    env.episode_score = 0.0
    while True:
        selected_action, action_prob = agent.work(observation, type_="selectActionMax")
        observation, _, done, _ = env.step([selected_action])
        if done:
            observation = env.reset()