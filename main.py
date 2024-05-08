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
        print("HUNTER SETTING COMPLETE")


    def run(self):
        print("MPC START")
        while not rospy.is_shutdown():
            self.odom, self.heading, self.gps_status, self.gps_covariance, self.checkGPS, self.checkHeading, self.gps_back = self.gps_data.gps_data()
            self.odomFront, self.gps_statusFront, self.gps_covarianceFront, self.checkGPSFront, self.gps_front = self.gps_data.gps_dataFront()
            current_state, current_index, distance = self.hunter_state.update_hunter_state(self.odom, self.odomFront, self.gps_back, self.gps_front)
            if current_state[0] != 0 and self.gps_back and self.gps_front and current_state[3] != 0:
            # if current_state[0] != 0 and self.gps_back and self.gps_front:
                # print(current_state)
                Xref = self.ref_state_input.calc_ref_trajectory(current_state, current_index) # xref -> ref x, ref y
                # print(Xref)
                # print("\nREF")
                # print(Xref[2,0])
                # print("\nCURR")
                # print(current_state[2])
                Xbar_k, Ak_list, Bk_list, Ck_list = self.predict_future.predict_state(current_state, self.opt_input, Xref)
                # print(Xbar_k)
                self.opt_input = self.optimize_cost.optimize(Xref, Ak_list,Bk_list, Ck_list, current_state)
                self.opt_input = self.opt_input.T
                self.vehicle_control.control(self.opt_input[0,1]*0.1375, self.opt_input[0,0])
                # now_time = time.time()
                # if (now_time - pre_time) // 3 > 0:
                #     print("\n\n\n")
                #     print(current_state)
                #     print("\n\n\n")
                #     pre_time = now_time
                
                print(self.opt_input[0,1]*0.1375, self.opt_input[0,0])
                self.rate.sleep()
if __name__ == "__main__":
    try:
        hunter = HUNTUR_MPC()
        hunter.run()
    except rospy.ROSInterruptException:
        pass

# def iterative_linear_mpc_control(xref, x0, dref, oa, od):
#     """
#     MPC contorl with updating operational point iteraitvely
#     """

#     if oa is None or od is None:
#         oa = [0.0] * T
#         od = [0.0] * T

#     for i in range(MAX_ITER):
#         xbar = predict_motion(x0, oa, od, xref)
#         poa, pod = oa[:], od[:]
#         oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)
#         du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
#         if du <= DU_TH:
#             break
#     else:
#         print("Iterative is max iter")

#     return oa, od, ox, oy, oyaw, ov

## 이건 아마 main에 들어가는게 좋을듯