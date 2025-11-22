import time

import requests
import json



if __name__ == '__main__':
   url="http://0.0.0.0:3050"
   start_flag=0
   while 1:
        if start_flag:
          status={"radar":0,"platform":0,"arm":0,"camera":0,"electricityQuantity":80,"error_messages":"self.error_messages"}
          start_flag=0
        else:
           status={"radar":-1,"platform":-1,"arm":-1,"camera":-1,"electricityQuantity":0,"error_messages":"self.error_messages"}
           start_flag=1
        try:
            print(status)
            response = requests.post(
                url, json.dumps(status).encode("utf-8"), headers={"content-type": "application/json"})
        except:
            print("yichang")
        time.sleep(2)

