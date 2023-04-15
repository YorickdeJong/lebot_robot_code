#!/usr/bin/env python3

import rospy
from ina219 import INA219
from ina219 import DeviceRangeError
from std_msgs.msg import Float32
from time import sleep
import requests
import uuid

API_URL = "http://10.7.191.125:3001/api/v1/power-measurement-results"

SHUNT_OHMS = 0.1
MAX_EXPECTED_AMPS = 0.4
record_number = str(uuid.uuid4())

def wait_for_param(param_name, timeout=None):
    start_time = rospy.get_time()
    while not rospy.has_param(param_name):
        if timeout and rospy.get_time() - start_time > timeout:
            raise ROSException(f"Parameter '{param_name}' not available within the given timeout.")
        rospy.sleep(0.1)

def update_row(api_url, record_number, time_data):
    query = f"""
        UPDATE power_data
        SET time = array_append(time, {time_data})
        WHERE record_number = {record_number};
    """
    response = requests.post(api_url, data={"query": query})
    response.raise_for_status()


def read_ina219():
    ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS)
    ina.configure(ina.RANGE_16V)

    #init node
    rospy.init_node('power_node')
    
    #wait for params
    wait_for_param('/power_node/userID')
    wait_for_param('/power_node/assignmentNumber')
    wait_for_param('/power_node/assignmentTitle')
    wait_for_param('/power_node/subjectTitle')

    #set params
    user_id = rospy.get_param('/power_node/userID')
    assignment_number = rospy.get_param('/power_node/assignmentNumber')
    assignment_title = rospy.get_param('/power_node/assignmentTitle')
    subject_title = rospy.get_param('/power_node/subjectTitle')


    rospy.init_node('power_monitor_node', anonymous=True)
    HZ = 3
    rate = rospy.Rate(HZ)  # 1 Hz
    time = 0

    while not rospy.is_shutdown():
        try:
            voltage = ina.voltage()
            current = ina.current()
            power = ina.power()

            rospy.loginfo('Bus Voltage: {:.3f} V'.format(voltage))
            rospy.loginfo('Current: {:.3f} mA'.format(current))
            rospy.loginfo('Power: {:.3f} mW'.format(power))

            payload = {
                "record_number": record_number,
                "voltage": voltage,
                "current": current,
                "power": power,
                "time": time,
                "user_id": user_id,
                "assignment_number": assignment_number,
                "title": assignment_title,
                "subject": subject_title,
            }

            update_row(API_URL, record_number, time_data)

            rate.sleep()

        except DeviceRangeError as e:
            rospy.logwarn(e)

        except Exception as e:
            rospy.logerr(e)

        time += 1/HZ

if __name__ == '__main__':
    try:
        read_ina219()
    except rospy.ROSInterruptException:
        pass

# Launch script
#roslaunch driver_bot_cpp power_test.launch user_id:=7 assignment_number:=1 assignment_title:="Vragen Opdracht"  subject_title:=CAR