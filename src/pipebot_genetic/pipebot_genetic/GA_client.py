import rclpy
import math
import time
import pickle
import sys
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

SAVEFILE = '~/optiParams'
GENERATIONS = 1
POP = 1


class GA_Client(Node):

    def __init__(self, savefile, gen, runs):
        super().__init__('GA_client')
        self.pos = (0,0)
        self.roll = 0
        self.pitch = 0
        self.target = (5,0) #(10,-6)
        self.init_dist = hypot(self.pos[0]-self.target[0], self.pos[1]-self.target[1])
        self.time = 0
        self.maxtime = 30
        self.weights = 25
        self.biases = 7

        self.obstacle_descr = generate_obstacle(0.5,-0.5,3,6,0.4)

        self.GENS = gen
        self.POP = runs

        self.savefile = savefile
        self.instance = 0
        
        self.odom_subscriber = self.create_subscription(Odometry, '/bot1/odom', self.odom_callback, QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT))
        
        self.neural_initialiser = self.create_client(Genes, '/bot1/neural_network')
        self.reset = self.create_client(Empty, '/reset_simulation')
        self.pause = self.create_client(Empty, '/pause_physics')
        self.unpause = self.create_client(Empty, '/unpause_physics')
        self.delete_entity = self.create_client(DeleteEntity, '/delete_entity')
        self.spawn_entity = self.create_client(SpawnEntity, '/spawn_entity')
        self.training = False

        self.pause.wait_for_service()
        self.pause_sim()

    def save(self, res, savefile):
        with open(savefile, 'w+') as output:
            pickle.dump(res, output, pickle.HIGHEST_PROTOCOL)

    def run_save(self):
        with open(self.savefile, 'r') as input:
            save = pickle.load(input)
            self.launch_instance(save.x)
    
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
        del_req = DeleteEntity.Request()
        del_req.name = 'obstacle'
        future2 = self.delete_entity.call_async(del_req)
        rclpy.spin_until_future_complete(self, future2)
        time.sleep(0.5)
        spawn_req = SpawnEntity.Request()
        spawn_req.name = 'obstacle'
        spawn_req.xml = self.obstacle_descr
        future3 = self.spawn_entity.call_async(spawn_req)
        rclpy.spin_until_future_complete(self, future3)
        return future.result(), future2.result(), future3.result()

    def pause_sim(self):
        future = self.pause.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def unpause_sim(self):
        future = self.unpause.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def launch_instance(self, args):
        self.pos = (0,0)
        self.pitch = 0
        self.roll = 0
        self.time = 0
        w = args[:self.weights]
        #b = [0.0 for i in range(self.biases)]
        b = args[self.weights:]
        
        res1, res2, res3 = self.reset_sim()
        self.get_logger().info(res3.status_message)
        self.init_NN(w,b)
        time.sleep(0.5)
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
        self.instance += 1
        self.get_logger().info('Instance #%d:' % (self.instance) + "Weights: {}\n".format(' '.join(map(str, w))) + "Biases: {}\n".format(' '.join(map(str, b))) + 'Fitness: %f \n' % (fitness))
        return fitness

    def optimize(self):
        self.instance = 0
        #differential_evolution(self.launch_instance, [(-1,1) for i in range(self.weights+self.biases)])
        res = differential_evolution(self.launch_instance, [(-1,1) for i in range(self.weights+self.biases)], maxiter=self.GENS, popsize=self.POP, polish=False)
        self.save(res, self.savefile)
        return(res)
        

def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)
    
    genetic_algo = GA_Client(SAVEFILE, GENERATIONS, POP)

    run_save = False
    print(args)
    if('--run_save' in args):
        run_save = True

    if(run_save):
        genetic_algo.get_logger().info('Running saved instance...')
        genetic_algo.run_save()
    else:
        genetic_algo.get_logger().info('Optimising...')
        genetic_algo.optimize().x    

    genetic_algo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
