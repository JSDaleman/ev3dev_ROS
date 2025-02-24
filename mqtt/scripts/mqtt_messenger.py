#!/usr/bin/env python3

__author__ = "Juan Sebastian Daleman Martinez"
__copyright__ = "Copyright 2025, Ev3 ROS atravez de MQTT"
__license__ = "MIT"
__version__ = "0.0.2"
__maintainer__ = "Juan Sebastian Daleman Martinez"
__email__ = "jdaleman@unal.edu.co"
__status__ = "Development"

import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from mqtt.mqtt_remote_method_calls import MqttClient
from mqtt.delegate import PcROSDelegate
from mqtt.units import UnitConverter

class MQTTNode(MqttClient):
    def __init__(self, delegate=None, id=1, mqtt_borker_ip="e9d73965c3fe4220b51bf8e5f1365a27.s1.eu.hivemq.cloud"):
        super().__init__(delegate, id, mqtt_borker_ip)

        rospy.init_node('mqtt_node')
        self.connect_to_ev3()
        self.convert_units = UnitConverter()

        # Publishers
        self.publish_topic_message = f"{self.robot_name}/mqtt_message"
        self.publish_topic_gyro = f"{self.robot_name}/gyro_sensor"

        self.pub_message = rospy.Publisher(self.publish_topic_message,
                                            String,
                                            queue_size=10)
        self.pub_gyro = rospy.Publisher(self.publish_topic_gyro, 
                                        String, 
                                        queue_size=10)
        self.delegate.set_gyro_publisher(self.pub_gyro)

        # Subscribers
        self.subscription_topic_wheels_vel = f"{self.robot_name}/wheels_vel"
        self.subscription_topic_command = f"{self.robot_name}/command"

        self.sub_wheels_vel = rospy.Subscriber(self.subscription_topic_wheels_vel, 
                                               JointState, 
                                               self.robot_wheels_vel_received)
        self.sub_command = rospy.Subscriber(self.subscription_topic_command, 
                                            String, 
                                            self.robot_command_received)

        rospy.loginfo("MQTT Node initialized")
        self.rate = rospy.Rate(1)  # 1 Hz

    def robot_wheels_vel_received(self, msg):
        try:
            # Suponiendo que las ruedas están en las primeras dos posiciones
            wheel_left_index = msg.name.index("wheel_left")
            wheel_right_index = msg.name.index("wheel_right")
            
            left_wheel_velocity = msg.velocity[wheel_left_index]
            right_wheel_velocity = msg.velocity[wheel_right_index]

            self.send_message_move_wheels(right_wheel_velocity, left_wheel_velocity, "ros_wheels_vel")
            rospy.loginfo(f"Velocities [rad/s] RW: {left_wheel_velocity}  RD: {right_wheel_velocity}")

        except ValueError:
            rospy.logwarn("No se encontraron los nombres de las ruedas en el mensaje de JointState")

        except Exception as e:
            rospy.logerr(f"Error inesperado al procesar velocidades de ruedas: {e}")

    def robot_command_received(self, msg):
        datos = msg.data.split()

        if not datos:
            rospy.logwarn("Mensaje de comando vacío recibido")
            return
        
        command = datos[0]
        args = datos[1:]

        # Responde al mensaje recibido
        self.send_message_command(command, args, "ros_command")
        self.pub_message.publish(msg.data)

    def send_message_move_wheels(self, wheel_right_velocity, wheel_left_velocity, msg):
        """Convierte velocidades de radianes/segundo a RPM y envía el mensaje MQTT."""

        #Declaración de elementos de la lista de parametros
        msg_left = self.convert_units.rad_per_sec_to_rpm(wheel_left_velocity)
        msg_right = self.convert_units.rad_per_sec_to_rpm(wheel_right_velocity)

        msgtype = "drive"
        msglist = [msg_left, msg_right]
        
        self.send_message(msgtype, msglist)
        rospy.loginfo(f"{msg} {self.publish_topic_name} {msgtype} {msglist} [rpm]")

    #Funcion de envio de mensajes MQTT para acciones diferentes a mover el robot
    def send_message_command(self, msg_command, msg_parameters, msg):
        """Envía comandos MQTT con o sin parámetros."""

        if msg_parameters:
            self.send_message(msg_command, msg_parameters)
        else:
            self.send_message(msg_command)

        rospy.loginfo(f"{msg} {self.publish_topic_name} {msg_command} {msg_parameters}")

    def run(self):
        while not rospy.is_shutdown():
            #self.send_message_command("Angle", [], "ros_angle_solicitud")
            self.rate.sleep()
        self.close()

if __name__ == '__main__':
    try:
        delegate = PcROSDelegate()
        lego_id = rospy.get_param('/robot_id', 1)
        rospy.set_param('/robot_name', lego_id)
        node = MQTTNode(delegate, lego_id)
        time.sleep(3)
        node.run()
        
    except rospy.ROSInterruptException:
        node.close()