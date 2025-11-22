import time
import json
import serial
import requests
import threading
from flask import Flask
from flask import request
import paho.mqtt.client as mqtt
from wrapped_socket import get_socket
from wsgiref.simple_server import make_server

import logging
logging.basicConfig(level=logging.INFO, filename='/home/mkq/LIBS/LIBS.log', filemode='a', format='%(asctime)s - %(levelname)s - %(message)s')


ser = serial.Serial('/dev/ttyS3', 9600)
ser.flushInput()  # Clear input buffer data
#ser.write('the data of LIBS is coming\n'.encode())
app = Flask(__name__)

MQTTHOST = '192.168.1.229'
MQTTPORT = 1883
mqttClient = mqtt.Client('LIBS')

status_http = 'IDLE'#BUSY

def on_mqtt_connect():
    mqttClient.connect(MQTTHOST, MQTTPORT, 60)
    mqttClient.reconnect_delay_set(min_delay=3, max_delay=10)
    mqttClient.loop_start()


def on_publish(topic, payload, qos):
    mqttClient.publish(topic, payload, qos)

@app.route('/ping', methods=['GET', 'POST'])
def get_profiles():
    return('pong')
@app.route('/', methods=['GET', 'POST'])
def get_profile():
    global data
    response = {'code': '', 'message': ''}
    string = request.get_data().decode('utf-8')
    data = json.loads(string)
    operate = data['param']['operate']
    logging.info(data)
    if 'start' in operate:
        ser.write(('start'+'\n').encode())
        response['code'] = '200'
        response['message'] = 'ok'
        response = json.dumps(response, indent=4, separators=(',', ':'))
    else:
        response['code'] = '500'
        response['message'] = 'false'
        response = json.dumps(response, indent=4, separators=(',', ':'))
    return response,200,[("Content-Type", "application/json")]


def Report():
    global data
    while True:
        receive = ser.readline().decode()
        logging.info(receive)
        if 'started' in receive:
            response = {'id': ''}
            try:
                response['id'] = data['id']
            except:
                response['id'] = 1
                print("There was a ['id'] failure")
            response['stemp'] = int(time.time())
            response['result'] = 'ok'
            response['detail'] = 'start'
            tcp_client_spec = get_socket()
            tcp_client_lase = get_socket()
            tcp_client_lase.connect(("192.168.1.144", 21567))
            tcp_client_spec.settimeout(10)
            tcp_client_spec.connect(("192.168.1.144", 12345))
            tcp_client_lase.settimeout(10)

            tcp_client_spec.send(bytes('hide', 'utf-8'))
            tcp_client_lase.send(bytes("login", "utf-8"))
            b = tcp_client_lase.recv(1024)
            print(b)

            tcp_client_lase.send(bytes("standby", "utf-8"))
            b = tcp_client_lase.recv(1024)
            print(b)

            tcp_client_lase.send(bytes("fire", "utf-8"))
            tcp_client_spec.send(bytes("start", "utf-8"))
            time.sleep(0.5)
            tcp_client_lase.close()
            tcp_client_spec.close()
            response = json.dumps(response, indent=4, separators=(',', ':'))
            on_publish('station-to-dms', response, 1)
        elif 'P_start' in receive:
            tcp_client_spec = get_socket()
            tcp_client_lase = get_socket()
            tcp_client_lase.connect(("192.168.1.144", 21567))
            tcp_client_spec.settimeout(10)
            tcp_client_spec.connect(("192.168.1.144", 12345))
            tcp_client_lase.settimeout(10)

            tcp_client_spec.send(bytes('hide', 'utf-8'))
            tcp_client_lase.send(bytes("login", "utf-8"))
            b = tcp_client_lase.recv(1024)
            print(b)

            tcp_client_lase.send(bytes("standby", "utf-8"))
            b = tcp_client_lase.recv(1024)
            print(b)

            tcp_client_lase.send(bytes("fire", "utf-8"))
            tcp_client_spec.send(bytes("start", "utf-8"))
            time.sleep(0.5)
            tcp_client_lase.close()
            tcp_client_spec.close()
        elif 'laser_ok' in receive:
            tcp_client_spec = get_socket()
            tcp_client_lase = get_socket()
            tcp_client_lase.connect(("192.168.1.144", 21567))
            tcp_client_spec.settimeout(10)
            tcp_client_spec.connect(("192.168.1.144", 12345))
            tcp_client_lase.settimeout(10)
            tcp_client_lase.send(bytes("standby", "utf-8"))
            tcp_client_spec.send(bytes("stop", "utf-8"))
            time.sleep(0.5)
            tcp_client_spec.send(bytes('show', 'utf-8'))
            time.sleep(0.5)
            tcp_client_spec.send(bytes("stop", "utf-8"))
            time.sleep(0.5)
            tcp_client_spec.send(bytes('show', 'utf-8'))
            time.sleep(0.5)
            tcp_client_lase.send(bytes("shutdown", "utf-8"))
            time.sleep(0.5)
            tcp_client_lase.close()
            tcp_client_spec.close()

        elif 'done' == receive:
            print('done')
        '''
            response = {'id': ''}
            try:
              response['id'] = data['id']
            except:
              response['id'] = 1
              print("There was a ['id'] failure")
            response['stemp'] = int(time.time())
            response['result'] = 'ok'
            response['detail'] = 'done'
            response = json.dumps(response, indent=4, separators=(',', ':'))
            on_publish('station-to-dms', response, 1)
            '''
def heartbeat():
    global status_http
    while (1):
        time.sleep(5)
        post_json = json.dumps({
            "ip": "",
            "port": 0,
            "stamp": int(time.time()*1000),
            "machineCode": "libs",
            "status": status_http,
            "detail": {
                "rackList": [
                    {
                        "rackIndex": 0
                    }
                ],
                "slotList": [
                    {
                        "slotIndex": 0,
                        "rackIndex": 0
                    }
                ]
            },
            "detailMsg": "",
            "currentStation": "",
            "electricityQuantity": 0.00
        })
        try:
            r3 = requests.post("http://192.168.1.229/api/workstation/heartbeat",
                               data=post_json, headers={"Content-Type": "application/json"})
        except requests.exceptions.ConnectionError as e:
            # handle the connection error
            print(f"ConnectionError: {e}")



if __name__ == '__main__':
    on_mqtt_connect()
    on_publish('station-to-dms', 'LIBS is online', 1)
    (threading.Thread(target=Report)).start()
    (threading.Thread(target=heartbeat)).start()
    make_server('0.0.0.0',5000,app).serve_forever()
    #app.run('0.0.0.0',5000)
    #app.run(host='192.168.1.52', port=5000)
