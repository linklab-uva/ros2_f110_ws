import streamlit as st
import time, socket, struct
from threading import Thread

import time  # to simulate a real time data, time loop

import numpy as np  # np mean, np random
import pandas as pd  # read csv, df manipulation
import plotly.express as px  # interactive charts


def setup():
    global car_data, listen_UDP

    num_cars = 9

    car_data = []
    for i in range(num_cars):
        car_data.append({'autonomous_enabled': "False", 'rpm': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'servo_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})

    st.set_page_config(
        page_title="F110 Live Telemetry Visualizer",
        page_icon="✅",
        layout="wide",
    )
    times = [ -0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0]
    rpm_data = {'car1' : car_data[0]['rpm'], 'car2' : car_data[1]['rpm'], 'car3' : car_data[2]['rpm'], 'car4' : car_data[3]['rpm'], 'car5' : car_data[4]['rpm'], 'car6' : car_data[5]['rpm'], 'car7' : car_data[6]['rpm'], 'car8' : car_data[7]['rpm'], 'car9' : car_data[8]['rpm']}
    chart_data = pd.DataFrame(data=rpm_data, index=times)

    st.line_chart(chart_data)
    

    listen_UDP = Thread(target=udp_socket_client())
    listen_UDP.start()
    listen_UDP.join()

    while True:
        time.sleep(1)
        st.experimental_rerun()
        print(car_data)


def udp_socket_client():
    UDP_address = '127.0.0.1' 
    UDP_port = 10000

    UDP_address_port = (UDP_address, UDP_port)

    control_format = '?ff'
    control_buff = struct.calcsize(control_format)


    socket.setdefaulttimeout(1)
    UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    UDP_socket.bind(UDP_address_port)
    UDP_socket.settimeout(60)

    UDP_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, control_buff)

    while(True):
        try:
            data, addr = UDP_socket.recvfrom(control_buff)
            control_struct = struct.unpack(control_format, data)

            car_num = int(addr[0][-1]) - 1
            car_data[car_num]['autonomous_enabled'] = control_struct[0]
            del car_data[car_num]['rpm'][0]
            del car_data[car_num]['servo_pos'][0]
            car_data[car_num]['rpm'].append(control_struct[1])
            car_data[car_num]['servo_pos'].append(control_struct[2])
            
        except Exception as e:
            if e == KeyboardInterrupt:
                UDP_socket.close()
                return
            time.sleep(0.2)
            print("No Connections")


if __name__ == '__main__':
    setup()
