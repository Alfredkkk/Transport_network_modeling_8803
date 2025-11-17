from SingleUESTA.network import Network
from SingleUESTA.node import Node
from SingleUESTA.link import Link
from SingleUESTA.path import Path
from SingleUESTA.od import OD
import pickle
# from nose.tools import assert_almost_equal
import numpy as np

import sys
import traceback
import SingleUESTA.utils as utils

# restored = {}
# with open('self-check.pickle', 'rb') as f:
#     restored = pickle.load(f)

# FRANK_WOLFE_STEPSIZE_PRECISION = 1e-7
# UE_PRECISION = 10

import sys
import os

# -----------------------------------------------------------
# 1. Setup PATH to import Network and other classes
# -----------------------------------------------------------
# Assuming all your classes (network.py, link.py, etc.) 
# are in the 'Single-class UE STA' folder.
try:
    sys.path.append(os.path.join(os.getcwd(), 'Single-class UE STA'))
    from SingleUESTA.network import Network
    from SingleUESTA.link import Link
    # Note: Other classes like Node, OD, Path, utils will be imported automatically
    # by network.py, but make sure they are in the same folder.
except ImportError as e:
    print(f"Error importing classes. Make sure all necessary files are in the 'Single-class UE STA' folder.")
    print(f"Details: {e}")
    sys.exit(1)

# -----------------------------------------------------------
# 2. Define File Paths (Adjust if your folders are named differently!)
# -----------------------------------------------------------
NETWORK_FILE = os.path.join(os.getcwd(), 'Team 2', 'SiouxFalls', 'SiouxFalls_net.tntp')
DEMAND_FILE = os.path.join(os.getcwd(), 'Team 2', 'SiouxFalls', 'SiouxFalls_trips.tntp')

def run_sta_tests():
    """
    Initializes the network, runs UE STA, and runs SO STA, 
    and compares the final objective values.
    """
    if not os.path.exists(NETWORK_FILE) or not os.path.exists(DEMAND_FILE):
        print("\n--- ERROR ---")
        print(f"Network file not found: {NETWORK_FILE}")
        print(f"Demand file not found: {DEMAND_FILE}")
        print("Please check your 'SiouxFalls' folder and file names.")
        return

    # -----------------------------------------------------------
    # Test 1: User Equilibrium (UE STA)
    # -----------------------------------------------------------
    print("==================================================")
    print("Starting Single-Class User Equilibrium (UE STA)")
    print("==================================================")

    # 1. Initialize Network and read files
    network_ue = Network(NETWORK_FILE, DEMAND_FILE)

    # 2. Run the UE Frank-Wolfe algorithm
    # The function returns (Beckmann_Objective, Final_TSTT)
    # The default targetGap (1e-4) and maxIterations (100) are used.
    ue_obj_value, ue_tstt = network_ue.userEquilibriumFW()

    print("\n--------------------------------------------------")
    print(f"UE STA FINISHED in {network_ue.iteration} iterations.")
    print(f"Final UE Objective (Beckmann): {ue_obj_value:,.2f}")
    print(f"Total System Travel Time (TSTT) under UE: {ue_tstt:,.2f}")
    print("--------------------------------------------------\n")


    # -----------------------------------------------------------
    # Test 2: System Optimal (SO STA)
    # -----------------------------------------------------------
    print("==================================================")
    print("Starting Single-Class System Optimal (SO STA)")
    print("==================================================")
    
    # We must re-read the network and reset flows to 0 before running SO
    network_so = Network(NETWORK_FILE, DEMAND_FILE)

    # 2. Run the SO Frank-Wolfe algorithm
    # The function returns the Final TSTT (which is the SO objective)
    so_tstt = network_so.systemOptimalFW()

    print("\n--------------------------------------------------")
    print(f"SO STA FINISHED in {network_so.iteration} iterations.")
    print(f"Final SO Objective (TSTT): {so_tstt:,.2f}")
    print("--------------------------------------------------\n")


    # -----------------------------------------------------------
    # 3. Final Comparison (The main takeaway!)
    # -----------------------------------------------------------
    print("==================================================")
    print("Assignment Principle Comparison")
    print("==================================================")
    
    # TSTT should always be lower under SO than under UE.
    # The difference is the "Cost of Anarchy."
    cost_of_anarchy = ue_tstt - so_tstt
    
    print(f"Total Travel Time (UE): {ue_tstt:,.2f}")
    print(f"Total Travel Time (SO): {so_tstt:,.2f}")
    print(f"Cost of Anarchy (UE TSTT - SO TSTT): {cost_of_anarchy:,.2f}")
    
    if so_tstt < ue_tstt:
        print("\nConclusion: The SO TSTT is correctly lower than the UE TSTT. The code works as expected for single-class assignment!")
    elif so_tstt == ue_tstt:
        print("\nWarning: The SO and UE TSTT are the same. This usually means the network is uncongested or one of your algorithms failed to converge correctly.")
    else:
        print("\nCRITICAL ERROR: The SO TSTT is higher than the UE TSTT. This is mathematically impossible and indicates a fundamental error in your SO Marginal Cost or Frank-Wolfe step size implementation.")

if __name__ == "__main__":
    run_sta_tests()