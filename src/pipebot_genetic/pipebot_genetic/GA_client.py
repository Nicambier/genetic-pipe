import rclpy
import math
import time
import pickle
import sys
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from scipy.optimize import differential_evolution
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance
from std_srvs.srv import Empty
from pipebot_services.srv import Genes

SAVEFILE = 'optiWeights'
GENERATIONS = 1000
POP = 5


class GA_Client(Node):

    def __init__(self, savefile, gen, runs):
        super().__init__('GA_client')
        self.pos = (0,0)
        self.target = (4.5,0) #(10,-6)
        self.init_dist = 4.5#14.14
        self.time = 0
        self.maxtime = 30
        self.weights = 36#7 #16
        self.biases = 6#8

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
        self.training = False

        self.pause.wait_for_service()
        self.pause_sim()

    def save(self, res):
        with open(self.savefile, 'wb') as output:
            pickle.dump(res, output, pickle.HIGHEST_PROTOCOL)

    def run_save(self):
        with open(self.savefile, 'rb') as input:
            save = pickle.load(input)
            self.launch_instance(save.x)
    
    def odom_callback(self, msg):
        if(self.training):
            self.time = msg.header.stamp.sec
            self.pos = (msg.pose.pose.position.x,msg.pose.pose.position.y)

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

    def launch_instance(self, args):
        self.pos = (0,0)
        self.time = 0
        #w = [args[0],args[1],args[2],args[2],args[1],args[0],args[3],args[4],args[4],args[3],args[5],0.0,args[6],args[5],0.0,-args[6]]
        w = args[:self.weights]
        #b = [0.0 for i in range(self.biases)]
        b = args[self.weights:]
        
        self.reset_sim()
        self.init_NN(w,b)
        time.sleep(1)
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
        if(self.time>=self.maxtime):
            time_left = 0
        else:
            time_left = self.maxtime - self.time

        fitness = dist/self.init_dist - time_left/self.maxtime
        self.instance += 1
        self.get_logger().info('Instance #%d:' % (self.instance) + "Weights: {}\n".format(' '.join(map(str, w))) + "Biases: {}\n".format(' '.join(map(str, b))) + 'Fitness: %f \n' % (fitness))
        return fitness

    def optimize(self):
        self.instance = 0
        #differential_evolution(self.launch_instance, [(-1,1) for i in range(self.weights+self.biases)])
        res = differential_evolution(self.launch_instance, [(-1,1) for i in range(self.weights+self.biases)], maxiter=self.GENS, popsize=self.POP)
        self.save(res)
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
