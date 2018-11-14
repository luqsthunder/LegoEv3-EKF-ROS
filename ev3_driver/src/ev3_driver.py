from ev3dev.ev3 import *
from math import pi
from time import sleep
import rospy

RAD_TO_DEGREE =  180.0/pi #para conversão de radiano para graus

class RobotController:
    def __init__(self, port_wheel_l= 'A', port_wheel_r = 'D', frequency = 20,
                 radius_wheel_l = 0.025, radius_wheel_r = 0.025,
                 create_node_sub = False):
        """
        @params
        port_wheel_l

        port_wheel_r

        frequency

        radius_wheel_l

        radius_wheel_r

        create_node_sub
        """

        # velocidade linear da roda esquerda em metros por segundo
        self.vel_wheel_l = 0.0
        self.vel_wheel_r = 0.0
        # distancia percorrida pela roda esquerda em metros
        self.dist_wheel_l = 0.0
        self.dist_wheel_r = 0.0

        # velocidade da roda esquerda em graus por segundo
        self.w_wheel_l = 0.0
        self.w_wheel_r = 0.0

        self.radius_wheel_l = radius_wheel_l # angulo da roda esquerda
        self.radius_wheel_r = radius_wheel_r
        self.motor_l = LargeMotor('out' + port_wheel_l)
        self.motor_r = LargeMotor('out' + port_wheel_r)
        self.dt_ms = 1000.0 / frequency
        self.dt = 1.0 / frequency
        self.curr_t = 0

        ## colocar os topicos para subscrever
        if create_node_sub:
            self.create_node()


    def create_node():
        

    def updateVel(self, vel_wheel_l, vel_wheel_r, stop_action='coast'):
        self.vel_wheel_l = vel_wheel_l
        self.vel_wheel_r = vel_wheel_r
        # incremento do tempo esperando ser 1/frequencia
        self.cur_t += dt

        # v_linear = v_angular*radius
        self.w_wheel_l = RAD_TO_DEGREE*self.vel_wheel_l/self.radius_wheel_l
        self.w_wheel_r = RAD_TO_DEGREE*self.vel_wheel_r/self.radius_wheel_r

        self.motor_l.run_timed(time_sp = self.dt_ms, stop_action = stop_action,
                                speed_sp = self.w_wheel_l)
        self.motor_r.run_timed(time_sp = self.dt_ms, stop_action = stop_action,
                               speed_sp = self.w_wheel_r)

    def convertLinearToWheelAng(self, ):
        

    def curWheelsDistance(self):
        """

        """

        dist_l = self.motor_l.position / self.motor_l.count_per_rot
        dist_l *= (pi * self.radius_wheel_l)

        dist_r = self.motor_r.position / self.motor_r.count_per_rot
        dist_r *= (pi * self.radius_wheel_r)

        return (dist_l, dist_r)



if __name__ == "__main__":

    r = RobotController(frequency = 20) # Testing with 10 hz

    # Test for 100 iterations
    for i in range(0, 100):
        r.updateVel(0.2, 0.2)
        sleep(r.dt)
