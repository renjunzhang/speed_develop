import time

import requests
import json


if __name__ == '__main__':
    url = "http://192.168.1.146:3030"
    data = {"nav": "lock"}
    data1 = {"nav": "unlock"}
    data = {'id': 1683372407232012288, 'stamp': None,
            'destination': 'charge_station', 'operations': [{'operation': 'charge'}]}

    try:
        response = requests.post(
            url, json.dumps(data).encode("utf-8"), headers={"content-type": "application/json"}, timeout=(30, 30))
        print(response.text)
    except:
        print("yichang")
