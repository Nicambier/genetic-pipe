import rclpy
import math
import time
import pickle
import sys
import signal
import os
import io
import contextlib
import psutil
import subprocess
import random
from math import hypot, asin, atan2, pi, copysign
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from scipy.optimize import differential_evolution
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance
from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import DeleteEntity
from pipebot_services.srv import Genes
from pipebot_genetic.obstacle_generator import generate_obstacle

SAVEFILE = 'optiParams'
GENERATIONS = 1000
POP = 5


class GA_Client(Node):

    def __init__(self, savefile, gen, runs, seed=0):
        super().__init__('GA_client')
        self.pos = (0,0)
        self.roll = 0
        self.pitch = 0
        self.target = (5,0) #(10,-6)
        self.init_dist = hypot(self.pos[0]-self.target[0], self.pos[1]-self.target[1])
        self.time = 0
        self.maxtime = 20
        self.weights = 25
        self.biases = 7

        if(seed==0):
            self.seed = random.randint(0,65535)
        else:
            self.seed = seed

        self.GENS = gen
        self.POP = runs

        self.savefile = savefile
        self.evo_output = io.StringIO()
        self.gen = 0
        self.instance = 1
        
        self.odom_subscriber = self.create_subscription(Odometry, '/bot1/odom', self.odom_callback, QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT))
        
        self.neural_initialiser = self.create_client(Genes, '/bot1/neural_network')
        while not self.neural_initialiser .wait_for_service(timeout_sec=1.0):
            self.get_logger().info('neural network service not available, waiting again...')
            
        self.reset = self.create_client(Empty, '/reset_simulation')
        while not self.reset.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gazebo reset service not available, waiting again...')
            
        self.pause = self.create_client(Empty, '/pause_physics')
        while not self.pause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pause service not available, waiting again...')
            
        self.unpause = self.create_client(Empty, '/unpause_physics')
        while not self.unpause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('unpause service not available, waiting again...')
            
        self.delete_entity = self.create_client(DeleteEntity, '/delete_entity')
        while not self.delete_entity.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('delete entity service not available, waiting again...')
            
        self.spawn_entity = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_entity.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn entity service not available, waiting again...')
            
        self.training = False

        self.pause.wait_for_service()
        self.pause_sim()

    def save(self, res, evo_output, savefile):
        with open(savefile, 'wb+') as output:
            pickle.dump(res, output, pickle.HIGHEST_PROTOCOL)
        self.get_logger().info("Best setting saved to "+savefile)

    def run_save(self):
        self.get_logger().info('Running saved instance...')
        with open(self.savefile, 'rb') as input:
            save = pickle.load(input)
            self.launch_instance(save.x,True)
    
    def odom_callback(self, msg):
        if(self.training):
            self.time = msg.header.stamp.sec
            self.pos = (msg.pose.pose.position.x,msg.pose.pose.position.y)
            q = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
            self.roll = atan2(2*(q[3]*q[0] + q[1]*q[2]), 1 - 2*(q[0]*q[0] + q[1]*q[1]))
            sinp = 2*(q[3]*q[1] + q[2]*q[0])
            if(abs(sinp) >= 1):
                self.pitch = copysign(pi/2,sinp)
            else:
                self.pitch = asin(sinp)

    def init_NN(self, weight, bias):
        req = Genes.Request()
        req.weights = list(weight)
        req.biases = list(bias)
        
        future = self.neural_initialiser.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def reset_sim(self):
        future = self.reset.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def pause_sim(self):
        future = self.pause.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def unpause_sim(self):
        future = self.unpause.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def update_gen(self):
        self.get_logger().info(self.evo_output.getvalue())
        self.evo_output.seek(0)

        del_req = DeleteEntity.Request()
        del_req.name = 'obstacle_'+str(self.gen)
        future1 = self.delete_entity.call_async(del_req)
        rclpy.spin_until_future_complete(self, future1)
        
        self.gen+=1
        self.instance=1

        #time.sleep(0.5)
        spawn_req = SpawnEntity.Request()
        spawn_req.name = 'obstacle_'+str(self.gen)
        spawn_req.xml = generate_obstacle(0.5,-0.5,3,6,0.4, random.randint(0,65535))
        future2 = self.spawn_entity.call_async(spawn_req)
        rclpy.spin_until_future_complete(self, future2)

        return future1.result(), future2.result()
        

    def launch_instance(self, args, paused=False):
        if(self.evo_output.tell() != 0 or self.gen==0):
            self.update_gen()
        self.pos = (0,0)
        self.pitch = 0
        self.roll = 0
        self.time = 0
        w = args[:self.weights]
        #b = [0.0 for i in range(self.biases)]
        b = args[self.weights:]
        
        self.reset_sim()
        self.init_NN(w,b)
        #time.sleep(0.5)
        if(not paused):
            self.unpause_sim()
        self.training = True
        while(self.time<self.maxtime and self.pos[0]<self.target[0]):
            time.sleep(0.001)
            rclpy.spin_once(self)
            #print(self.time, self.pos)
        #print('achieved')
        self.training = False
        self.pause_sim()
        if(self.pos[0]>=self.target[0]):
            dist = 0
        else:
            dist = math.sqrt(pow(self.pos[0] - self.target[0],2) + pow(self.pos[1] - self.target[1],2))

        fitness = dist/self.init_dist + min(self.time,self.maxtime)/self.maxtime + ((self.pitch/pi)**2 + (self.roll/pi)**2)/4 #arrive as soon and as straight as possible
        self.get_logger().info('Instance #%d of generation %d:' % (self.instance, self.gen) + "Weights: {}\n".format(' '.join(map(str, w))) + "Biases: {}\n".format(' '.join(map(str, b))) + 'Fitness: %f \n' % (fitness))
        self.instance += 1
        return fitness

    def optimize(self, polish):
        self.get_logger().info('Optimising with seed'+str(self.seed)+'...')
        random.seed(self.seed)
        self.gen = 0 #gen needs to be 0 at start to kickstart the counter and generate the first obstacle
        #differential_evolution(self.launch_instance, [(-1,1) for i in range(self.weights+self.biases)])
        with contextlib.redirect_stdout(self.evo_output):
            res = differential_evolution(self.launch_instance, [(-1,1) for i in range(self.weights+self.biases)], maxiter=self.GENS, popsize=self.POP, polish=polish, disp=True)
        self.save(res, self.evo_output, self.savefile)
        return(res)        
        

def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)

    run_save = False
    polish = False
    seed = 0
    for arg in args:
        if(arg=='run_save:=true'):
            run_save = True
        elif('savefile:=' in arg):
            global SAVEFILE
            SAVEFILE = arg.replace('savefile:=','')
        elif('generations:=' in arg):
            global GENERATIONS
            GENERATIONS = int(arg.replace('generations:=',''))
        elif('popsize:=' in arg):
            global POP
            POP = int(arg.replace('popsize:=',''))
        elif('seed:=' in arg):
            seed = int(arg.replace('seed:=',''))
        elif(arg=='polish:=True'):
            polish=True

    genetic_algo = GA_Client(SAVEFILE, GENERATIONS, POP, seed)

    if(run_save):
        genetic_algo.run_save()
    else:
        genetic_algo.optimize(polish).x    

    genetic_algo.destroy_node()
    rclpy.shutdown()
    pid = os.getpid()
    process = psutil.Process(os.getppid())
    for proc in process.children(recursive=True):
        if(proc.pid!=pid):
            proc.kill()
    process.kill()


if __name__ == '__main__':
    main()
