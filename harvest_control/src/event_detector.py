import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import UInt16
from geometry_msgs.msg import WrenchStamped
import numpy as np

class EventDetector(Node):

    def __init__(self):

        super().__init__('event_detector')

        self.stop_controller_cli = self.create_client(Empty, 'stop_controller')
        self.wait_for_srv(self.stop_controller_cli)
        self.stop_controller_req = Empty.Request()
        
        self.subscriber = self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster/wrench', self.wrench_callback, 10)
        self.pressure_subscriber = self.create_subscription(UInt16, '/pressure', self.pressure_callback, 10)

        self.force_memory = []
        self.pressure_memory = []
        self.window = 10
        
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.flag = False
        
        #WUR should adjust these to suit their gripper
        self.engaged_pressure = 300.0
        self.disengaged_pressure = 1000.0
        self.failure_ratio = 0.57
        
        #don't change this though
        self.pressure_threshold = self.engaged_pressure + self.failure_ratio*(self.disengaged_pressure - self.engaged_pressure)
        
        #optionally change this (should work well for UR5)
        self.force_change_threshold = -1.0
        
        
    def clear_trial(self):
        
        self.force_memory = []
        self.pressure_memory = []
        self.flag = False
        
    def stop_controller(self):

        self.future = self.stop_controller_cli.call_async(self.stop_controller_req)
        rclpy.spin_until_future_complete(self, self.future)
        
        self.clear_trial()
        
    
    def wrench_callback(self,msg):
        wrench = msg.wrench 

        current_force = np.array([wrench.force.x, wrench.force.y,
                                  wrench.force.z])
        
        force_mag = np.linalg.norm(current_force)
        self.force_memory.append(force_mag)
        if len(self.force_memory) > self.window:
            self.force_memory = self.force_memory[0:self.window]
            
    def pressure_callback(self,msg):
        #expects a scalar as a message
        pressure = msg.data 

        self.pressure_memory.append(pressure)
        if len(self.pressure_memory) > self.window:
            self.pressure_memory = self.pressure_memory[0:self.window]
            
    def moving_average(self, force):
        
        window_size = 5
        i = 0
        filtered = []

        while i < len(force) - window_size + 1:
            window_average = round(np.sum(force[i:i+window_size]) / window_size, 2)
            filtered.append(window_average)
            i += 1

        return filtered        
            
    def timer_callback(self):
        
        
        #if for some reason the engaged pressure is higher, flip > and < for pressure
        
        if len(self.force_memory) < self.window or len(self.pressure_memory) < self.window:
            return
        

        filtered_force = self.moving_average(self.force_memory)
        avg_pressure = np.average(self.pressure_memory)
    
        backwards_diff = []
        h = 2
        for j in range(2*h, (len(filtered_force))):
            diff = ((3 * filtered_force[j]) - (4 * filtered_force[j - h]) + filtered_force[j - (2*h)]) / (2 * h)
            backwards_diff.append(diff)
            j += 1
    
        cropped_backward_diff = np.average(np.array(backwards_diff))
           
        #if the suction cups are disengaged, the pick failed
        if avg_pressure >= self.pressure_threshold:
            print("Apple was failed to be picked :( Force: {np.round(filtered_force[0])} Max Force: {np.max(force)}  Bdiff: {cropped_backward_diff}  Pressure: {avg_pressure}")
            self.stop_controller()
        
        #if there is a reasonable force 
        elif filtered_force[0] >= 5:
            
            self.flag = True #force was achieved
            
            #check for big force drop
            if float(cropped_backward_diff) <= self.force_change_threshold and avg_pressure < self.pressure_threshold: 
                print("Apple has been picked! Bdiff: {cropped_backward_diff}   Pressure: {avg_pressure}.\
                        #Force: {filtered_force[0]} vs. Max Force: {np.max(force)}")
                self.stop_controller() 
    
            elif float(cropped_backward_diff) <= self.force_change_threshold and avg_pressure >= self.pressure_threshold:
                print("Apple was failed to be picked :( Force: {np.round(filtered_force[0])} Max Force: {np.max(force)}  Bdiff: {cropped_backward_diff}  Pressure: {avg_pressure}")
                self.stop_controller()
                    
        #if force is low, but was high, that's a failure too    
        elif self.flag and filtered_force[0] < 4.5:
            print("Apple was failed to be picked :( Force: {np.round(filtered_force[0])} Max Force: {np.max(force)}  Bdiff: {cropped_backward_diff}  Pressure: {avg_pressure}")
            self.stop_controller()

    def wait_for_srv(self, srv):
        while not srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

def main():

    rclpy.init()

    node = EventDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()