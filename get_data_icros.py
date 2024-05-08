#!/usr/bin/python3
# -*- coding: utf-8 -*-
import GetModelParams
import GetMpcParams
import GetPathParams
from RefStateInuputCalculator import RefStateInuputCalculator
from PredictFuture import PredictFuture
from OptimizeCost import OptimizeCost
from GetHunterState import HunterState
from GetSensorSate import GPS, VehicleState
from vehicle_control import VehicleControler

import numpy as np
import rospy

# for debug
import time

vehicle_model = "hunter"
mpc_model = "LMPC_tracking"
path_model = "0.1m_path"

class HUNTUR_MPC:
    def __init__(self):
        rospy.init_node("planner_vector", anonymous=False)
        self.model_params = GetModelParams.ModelConverter.getModelParams(vehicle_model)
        self.mpc_params = GetMpcParams.ModelConverter.getMpcParams(mpc_model)
        self.path_params = GetPathParams.ModelConverter.getPathParams(path_model)
        self.ref_state_input = RefStateInuputCalculator(self.model_params, self.mpc_params, self.path_params)
        self.predict_future = PredictFuture(self.mpc_params, self.model_params)
        self.optimize_cost = OptimizeCost(self.mpc_params, self.model_params)
        self.hunter_state = HunterState()
        self.gps_data = GPS()
        self.opt_input = [[0.0, 1.0] for _ in range(self.mpc_params.N)]
        print("MPC SETTING COMPLETE")
        self.vehicle_control = VehicleControler()
        self.rate = rospy.Rate(10)
        self.gps_front, self.gps_back = True, True
        self.pre_last_gps_inFront_to_sec_int = None
        self.file = open('current_states.csv', 'a', buffering=1) 
        self.current_states_0 = []
        self.current_states_1 = []
        self.rate = rospy.Rate(10) 
        print("HUNTER SETTING COMPLETE")

    def run(self):
        print("MPC START")

        while not rospy.is_shutdown():
            self.odom, self.heading, self.gps_status, self.gps_covariance, self.checkGPS, self.checkHeading, self.gps_back = self.gps_data.gps_data()
            self.odomFront, self.gps_statusFront, self.gps_covarianceFront, self.checkGPSFront, self.gps_front = self.gps_data.gps_dataFront()
            current_state, current_index, distance = self.hunter_state.update_hunter_state(self.odom, self.odomFront, self.gps_back, self.gps_front)
            self.current_states_0.append(current_state[0])
            self.current_states_1.append(current_state[1])
            self.save_state_to_file(current_state[0], current_state[1])
            print("ADD")
            # if current_state[0] != 0 and self.gps_back and self.gps_front and current_state[3] != 0:
            #     Xref = self.ref_state_input.calc_ref_trajectory(current_state, current_index) # xref -> ref x, ref y
            #     Xbar_k, Ak_list, Bk_list, Ck_list = self.predict_future.predict_state(current_state, self.opt_input, Xref)
            #     self.opt_input = self.optimize_cost.optimize(Xref, Ak_list,Bk_list, Ck_list, current_state)
            #     self.opt_input = self.opt_input.T
            #     self.vehicle_control.control(self.opt_input[0,1]*0.1375, self.opt_input[0,0])
            #     print(self.opt_input[0,1]*0.1375, self.opt_input[0,0])
            self.rate.sleep()

    def save_state_to_file(self, state_0, state_1):
        self.file.write(f"{state_0}, {state_1}\n")

    def __del__(self):
        self.file.close()
        
    def save_to_txt(self):
        with open('current_states.txt', 'w') as f:
            for s0, s1 in zip(self.current_states_0, self.current_states_1):
                f.write(f"{s0}, {s1}\n")
        print("Data saved to current_states.txt")


if __name__ == "__main__":
    try:
        hunter = HUNTUR_MPC()
        hunter.run()
    except rospy.ROSInterruptException:
        pass