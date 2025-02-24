#!/usr/bin/env python3

"""
Modulo de creación de cliente MQTT con elementos necesarios del
protocolo para uso con Robot Ev3 y PC.
"""

__author__ = "Juan Sebastian Daleman Martinez"
__copyright__ = "Copyright 2025, Ev3 ROS atravez de MQTT"
__credits__ = ["David Fisher"]
__license__ = "MIT"
__version__ = "0.0.3"
__maintainer__ = "Juan Sebastian Daleman Martinez"
__email__ = "jdaleman@unal.edu.co"
__status__ = "Development"

#Impotación de las librerias necesarias
import json
import paho.mqtt.client as paho
from collections.abc import Iterable
from paho import mqtt

class MqttClient(object):
    def __init__(self, delegate=None, id=1, mqtt_broker_ip="e9d73965c3fe4220b51bf8e5f1365a27.s1.eu.hivemq.cloud"):
        
        """
        Inicializa el cliente MQTT.
        """

        self.mqtt_broker_ip_address = mqtt_broker_ip

        self.lego_id = id
        self.robot_name = f"LegoEV3{self.lego_id:02d}"
        self.delegate = delegate
        self.port = 8883

        #Declaración del cliente y un delegado opcional de manejo de datos
        #Para funcionar en el Ev3 se quita la api version
        #ya que la verison que se usa en el robot de paho.mqtt.client es la 1.
        #Declaración del cliente y un delegado opcional de manejo de datos
        self.client = paho.Client(
            callback_api_version=paho.CallbackAPIVersion.VERSION2,
            client_id="", 
            userdata=None,
            protocol=paho.MQTTv5)
        
        #Declaración de topico de suscripción y publicación 
        self.subscription_topic_name = None
        self.publish_topic_name = None
        

    def connect_to_ev3(self, mqtt_broker_ip_address=None, lego_robot_number=None):
        """
        Conecta el PC al EV3.
        """
        
        if mqtt_broker_ip_address is None:
            mqtt_broker_ip_address = self.mqtt_broker_ip_address
        if lego_robot_number is None:
            lego_robot_number = self.lego_id

        #Sufijos para conectar el PC al Ev3
        self.connect("msgPC", "msgLegoEv3", mqtt_broker_ip_address, lego_robot_number)

    def connect_to_pc(self,  mqtt_broker_ip_address=None, lego_robot_number=None):

        """
        Conecta el EV3 al PC.
        """

        if mqtt_broker_ip_address is None:
            mqtt_broker_ip_address = self.mqtt_broker_ip_address
        if lego_robot_number is None:
            lego_robot_number = self.lego_id

        #Sufijos para conectar el EV3 al PC
        self.connect("msgLegoEv3", "msgPC", mqtt_broker_ip_address, lego_robot_number)

    def connect(self, subscription_suffix, publish_suffix, mqtt_broker_ip_address=None, lego_robot_number=None):
        """
        Establece la conexión con el broker MQTT.
        """

        if mqtt_broker_ip_address is None:
            mqtt_broker_ip_address = self.mqtt_broker_ip_address
        if lego_robot_number is None:
            lego_robot_number = self.lego_id
        
        #Declaración de ID del robot y sufijos necesarios para el topico MQTT
        self.robot_name = f"LegoEV3{lego_robot_number:02d}"
        self.subscription_topic_name = f"{self.robot_name}/{subscription_suffix}"
        self.publish_topic_name = f"{self.robot_name}/{publish_suffix}"
        
        # Configuración de eventos MQTT
        self.client.on_connect = self.on_connect
        self.client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
        self.client.message_callback_add(self.subscription_topic_name, self.on_message)

        
        # Configuración de credenciales
        self.client.username_pw_set(self.robot_name,  self.robot_name)

        #Se genera la conexión con el broker MQTT en el puerto
        self.client.connect(mqtt_broker_ip_address, self.port)
        print("Conectando al mqtt broker {}".format(mqtt_broker_ip_address), end="")

        # Inicio del loop de MQTT
        self.client.loop_start()

    def send_message(self, function_name, parameter_list=None):
        """
        Envía un mensaje MQTT con una función y sus parámetros.
        """

        message_dict = {"type": function_name}

        if parameter_list:

            #Verificación de que los parametros hayan sido ingresados esten en una estructura no iterable
            if not isinstance(parameter_list, Iterable):
                # Se le informa al usuario que los paramtros no estan en una lista y se corrige el error 
                print(f"The parameter_list {parameter_list} is not a list. Converting it to a list for you.")
                parameter_list = [parameter_list]

            #Se agraga al diccionario una key "payload" y valor la lista de parametros
            message_dict["payload"] = parameter_list

        #Conversión del diccionario message_dict en un mensaje jason
        message = json.dumps(message_dict)

        #Publicación del mensaje en el broker con el topico de publicación 
        self.client.publish(self.publish_topic_name, message)

    
    def on_connect(self,client, userdata, flags, rc, properties=None):
        """
        Evento ejecutado al conectar con el broker.
        """

        #Verificación de varaible de conexión
        if rc == 0:
            print(" ... Connected!")
        else:
            print(f"... Error de conexión: código {rc}")
            exit()
            """0: Connection successful
            1: Connection refused - incorrect protocol version
            2: Connection refused - invalid client identifier
            3: Connection refused - server unavailable
            4: Connection refused - bad username or password
            5: Connection refused - not authorised
            6-255: Currently unused."""
            #print("CONNACK received with code %s." % rc)
            

        #Impresión de cual es el topico de publicación al que se conecto
        print(f"Publish in the topic: {self.publish_topic_name}")

        #Declaración que la funcion on_subscribe es la misma de la clase cliente
        self.client.on_subscribe = self.on_subscribe
        self.client.subscribe(self.subscription_topic_name)

    def on_publish(self,client, userdata, mid, rc, properties=None):
        """
        Evento ejecutado tras una publicación exitosa.
        """

        #Cuando el mesaje qos es de confimación es exitoso se genera la siguiente impresión
        #impresión de identificardor de mensaje recibido
        print(f"Publish message with ID: {mid}")

    
    def on_subscribe(self, client, userdata, mid, granted_qos, properties=None):
        """
        Evento ejecutado tras una suscripción exitosa.
        """

        #Si se quiere saber el grado qos y el mesaje de dentificación descomentar la siguiente linea
        #print(f"Subscribed: {mid} {str(granted_qos)}")

        #Impresión del topico al que se ha generado la suscripción
        print(f"Subscribed to topic: {self.subscription_topic_name}")


    def on_message(self, client, userdata, msg):
        """
        Manejo de mensajes recibidos.
        """

        #Declaración de message con carga y decodificación del mensaje recibido
        message = msg.payload.decode()
        
        #Imprime el topico al que se asede el grado qos y el mensaje 
        #print(f"{msg.topic} {str(msg.qos)} {str(msg.payload)}")
        
        #Impresión del mensaje recibido por el servidor
        print(f"Received message: { message}")

        #Si no se creo un delegado para mensajes no se procesa el mensaje
        #solo se retorna
        if not self.delegate:
            print("Missing a delegate")
            return

        #Atención al mensaje recivido y llamado a la función apropiada.
        try:
            message_dict = json.loads(message)
        except ValueError:
            print("Unable to decode the received message as JSON")
            return

        if "type" not in message_dict:
            print("Received a messages without a 'type' parameter.")
            return
        
        message_type = message_dict["type"]
        
        #Se verifica si el delegado creado tiene el metodo ingresado
        if hasattr(self.delegate, message_type):
            method_to_call = getattr(self.delegate, message_type)

            #Se asume que el usuario dio parametros correctos
            #Se verfica si se ingreso la lista de parametros 
            if "payload" in message_dict:
                #Si se tiene la lista de parametros se obtine esta
                message_payload = message_dict["payload"]
                #Se desempaqueta la lista y se ingra al metodo llamado
                attempted_return = method_to_call(*message_payload)

            else:
                #Si no se tiene lista de parametros solo se llama al metodo
                attempted_return = method_to_call()

            if attempted_return:
                #Si el metodo retorno algun valor se le informa al usuario ya que no es posible el manejo de valores retornados
                print(f"The method {message_type} returned a value. That's not really how this library works. The value {attempted_return} was not magically sent back over")
        else:

            print(f"Attempt to call method {message_type} which was not found.")

    def close(self):
        """
        Cierra la conexión MQTT.
        """
        
        print("\nClose MQTT")
        self.delegate = None
        self.client.loop_stop()
        self.client.disconnect()
