import bagpy
from bagpy import bagreader
import pandas as pd

b = bagreader('coulomb_fric_test.bag')

# # replace the topic name as per your need
# LASER_MSG = b.message_by_topic('/vehicle/front_laser_points')
# LASER_MSG
# df_laser = pd.read_csv(LASER_MSG)
# df_laser # prints laser data in the form of pandas dataframe

measured_js = b.message_by_topic('/PSM2/measured_js')
setpoint_js = b.message_by_topic('/PSM2/setpoint_js')
gc_setpoint_js = b.message_by_topic('/PSM2/gravity_compensation/setpoint_js')