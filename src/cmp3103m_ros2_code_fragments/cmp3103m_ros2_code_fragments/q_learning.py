
"""
@author: Ronan Murphy - 15397831 and Geesara Kulathunga ggeesara@gmail.com
"""

import numpy as np
import random
import matplotlib.pyplot as plt

class Environment:
    def __init__(self, ):
        self.rows = 5
        self.cols = 6
        self.bomb_states = [(1,1),(3,0),(3, 3),(1,4)]
        self.start_state = (0,0)
        self.win_state = (4,4)

    def getReward(self, current_state):
        #give the rewards for each state
        for i in self.bomb_states:
            if current_state == i:
                return -100
        if current_state == self.win_state:
            return 100
        else:
            return -1
        
    def isEndFunc(self, current_state):
        #set state to end if win/loss
        if (current_state == self.win_state):
            return True
        for i in self.bomb_states:
            if current_state == i:
                return True 
        return False 
    
    def nxtPosition(self, current_state, action):     
        #set the positions from current action - up, down, left, right
        if action == 0:                
            nxtState = (current_state[0] - 1, current_state[1]) #up             
        elif action == 1:
            nxtState = (current_state[0] + 1, current_state[1]) #down
        elif action == 2:
            nxtState = (current_state[0], current_state[1] - 1) #left
        else:
            nxtState = (current_state[0], current_state[1] + 1) #right

        #check if next state is possible
        if (nxtState[0] >= 0) and (nxtState[0] <= self.rows-1):
            if (nxtState[1] >= 0) and (nxtState[1] <= self.cols-1):    
                    #if possible change to next state                
                    return nxtState 
        #Return current state if outside grid     
        return current_state 
        

class State:
    def __init__(self, state=(0,0)):
        self.current_state = state      

class Agent:
    def __init__(self, env):
        #inialise states and actions 
        self.states = []
        self.actions = [0,1,2,3]    # up, down, left, right
        self.env = env 
        self.State = State(self.env.start_state)
        #set the learning and greedy values
        self.alpha = 0.5
        self.gamma = 0.9
        self.epsilon = 0.1
        self.isEnd = False
        # array to retain reward values for plot
        self.plot_reward = []
        
        #initalise Q values as a dictionary for current and new
        self.Q = {}
        self.new_Q = {}
        #initalise rewards to 0
        self.rewards = 0
        
        #initalise all Q values across the board to 0, print these values
        for i in range(self.env.rows):
            for j in range(self.env.cols):
                for k in range(len(self.actions)):
                    self.Q[(i, j, k)] =0
                    self.new_Q[(i, j, k)] = 0
        
    #method to choose action with Epsilon greedy policy, and move to next state
    def Action(self):
        #random value vs epsilon
        rnd = random.random()
        #set arbitraty low value to compare with Q values to find max
        mx_nxt_reward =-10
        action = None
        
        #9/10 find max Q value over actions 
        if(rnd >self.epsilon) :
            #iterate through actions, find Q  value and choose best 
            for k in self.actions:
                i,j = self.State.current_state
                nxt_reward = self.Q[(i,j, k)]
                if nxt_reward >= mx_nxt_reward:
                    action = k
                    mx_nxt_reward = nxt_reward
                    
        #else choose random action
        else:
            action = np.random.choice(self.actions)
        
        #select the next state based on action chosen
        position = self.env.nxtPosition(self.State.current_state, action)
        return position,action
    
    
    #Q-learning Algorithm
    def Q_Learning(self,episodes):
        x = 0
        #iterate through best path for each episode
        while(x < episodes):
            #check if state is end
            if self.isEnd:
                #get current rewrard and add to array for plot
                reward = self.env.getReward(self.State.current_state)
                self.rewards += reward
                self.plot_reward.append(self.rewards)
                
                #get state, assign reward to each Q_value in state
                i,j = self.State.current_state
                for a in self.actions:
                    self.new_Q[(i,j,a)] = round(reward,3)
                    
                #reset state
                self.State = State()
                self.isEnd = False
                
                #set rewards to zero and iterate to next episode
                self.rewards = 0
                x+=1
            else:
                #set to arbitrary low value to compare net state actions
                mx_nxt_value = -10
                #get current state, next state, action and current reward
                next_state, action = self.Action()
                i,j = self.State.current_state
                reward = self.env.getReward(self.State.current_state)
                #add reward to rewards for plot
                self.rewards +=reward
                
                #iterate through actions to find max Q value for action based on next state action
                for a in self.actions:
                    nxtStateAction = (next_state[0], next_state[1], a)
                    q_value = (1-self.alpha)*self.Q[(i,j,action)] + self.alpha*(reward + self.gamma*self.Q[nxtStateAction])
                
                    #find largest Q value
                    if q_value >= mx_nxt_value:
                        mx_nxt_value = q_value
                
                #next state is now current state, check if end state
                self.State = State(state=next_state)
                self.isEnd = self.env.isEndFunc(self.State.current_state)
                
                #update Q values with max Q value for next state
                self.new_Q[(i,j,action)] = round(mx_nxt_value,3)
            
            #copy new Q values to Q table
            self.Q = self.new_Q.copy()
        
    #plot the reward vs episodes
    def plot(self,episodes):
        plt.plot(self.plot_reward)
        plt.show()
        
        
    #iterate through the board and find largest Q value in each, print output
    def showValues(self):
        for i in range(0, self.env.rows):
            print('-----------------------------------------------')
            out = '| '
            for j in range(0, self.env.cols):
                mx_nxt_value = -10
                for a in self.actions:
                    nxt_value = self.Q[(i,j,a)]
                    if nxt_value >= mx_nxt_value:
                        mx_nxt_value = nxt_value
                out += str(mx_nxt_value).ljust(6) + ' | '
            print(out)
        print('-----------------------------------------------')
        
    
        
if __name__ == "__main__":
    #create agent for 10,000 episdoes implementing a Q-learning algorithm plot and show values.
    env = Environment()
    ag = Agent(env)
    episodes = 10000
    ag.Q_Learning(episodes)
    ag.plot(episodes)
    ag.showValues()