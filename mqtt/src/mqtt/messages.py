#!/usr/bin/env python3

#Funcion de envio de mensajes MQTT para mover el robot
def send_message_movtank(mqtt_client, left_speed_entry, right_speed_entry, msg):

    #Declaración de elementos de la lista de parametros
    msg_left = left_speed_entry
    msg_right = right_speed_entry

    #Declaración del tipo de mensaje y lista de parametros
    msgtype = "drive"
    msglist = [msg_left, msg_right]
    
    #Envio de mensaje tipo "drive" y mensaje con lista de parametros
    mqtt_client.send_message(msgtype, msglist)

    """Impresión de acciónn realizada, topico en donde se publica
       el mensaje y el mensaje enviado"""
    print(msg, end="\t")
    print(mqtt_client.publish_topic_name, end=" ")
    print(msgtype, msglist)


#Funcion de envio de mensajes MQTT para acciones diferentes a mover el robot
def send_message_special(mqtt_client, msg_special, msg):

    #Declaración del tipo de mensaje sin lista de parametros
    msgtype = msg_special

    #Envio de mensaje
    mqtt_client.send_message(msgtype)

    """Impresión de acciónn realizada, topico en donde se publica
       el mensaje y el mensaje enviado"""
    print(msg, end="\t")
    print(mqtt_client.publish_topic_name, end=" ")
    print(msg_special)
