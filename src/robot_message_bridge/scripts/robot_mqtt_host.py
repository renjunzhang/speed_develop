#!/usr/bin/env python3

import paho.mqtt.client as mqtt
from robot_common_log import Log

logger = Log(__name__,"mqtt").getlog()


class RobotMqttHost:
    def __init__(self, ip="192.168.1.229", port=1883, client_name="robot"):
        self.mqtt_ip = ip
        self.mqtt_port = port
        self.mqtt_client = mqtt.Client(client_name)
        self.mqtt_ip = ip
        self.MqttConnect()
        self.MqttSubscribe()

    def MqttConnect(self):
        try:
            self.mqtt_client.connect(self.mqtt_ip, self.mqtt_port, 60)
            self.mqtt_client.username_pw_set("robochemist","robochemist")
            self.mqtt_client.loop_start()
        except Exception as e:
            logger.info("mqtt连接失败")

    def MqttPublish(self, topic, payload, qos):

        rc,mid=self.mqtt_client.publish(topic, payload, qos)
        logger.info(topic)
        if rc==0:
            logger.info("发送mqtt%r", payload.encode('utf-8').decode('utf-8'))
        else:
            logger.info("mqtt发送失败%r",payload)


    def MqttMessageCome(self, lient, userdata, msg):
        print('recv from ' + msg.topic + ':')
        print(msg.payload.decode())
        print('')

    def MqttSubscribe(self):
        self.mqtt_client.subscribe("topic1t", 1)
        self.mqtt_client.on_message = self.MqttMessageCome  # 消息到来处理函数
