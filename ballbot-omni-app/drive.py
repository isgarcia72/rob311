# standard imports
from collections import deque
from math import sqrt
import signal
import sys
import time
import threading
from threading import Thread

# third party imports
import numpy as np

# local imports
from balance_controller import BalanceController
from DataLogger import dataLogger
import FIR as fir
from MBot.Messages.message_defs import mo_states_dtype, mo_cmds_dtype, mo_pid_params_dtype
from MBot.SerialProtocol.protocol import SerialProtocol
from realtime_loop import SoftRealtimeLoop, LoopKiller
from slew_filter import SlewFilter

# CUTJIG PS4 CONTROLLER MAC ADDRESS: 98:B6:E9:D7:1E:25 

def register_topics(ser_dev:SerialProtocol):
    ser_dev.serializer_dict[101] = [lambda bytes: np.frombuffer(bytes, dtype=mo_cmds_dtype), lambda data: data.tobytes()]
    ser_dev.serializer_dict[121] = [lambda bytes: np.frombuffer(bytes, dtype=mo_states_dtype), lambda data: data.tobytes()]

# SETTING GLOBAL PHYSICAL CONSTANTS AND CONVERSIONS
# controll loop values
FREQ = 200
DT = 1/FREQ

# physical constants
RW = 0.0048
RK = 0.1210
ALPHA = np.deg2rad(45)
MAX_PLANAR_DUTY = 0.8

# wheel rotation to ball rotation transformation matrix
J11 = 0
J12 = -np.sqrt(3) * RW/ (3 * RK * np.cos(ALPHA))
J13 = -1 * J12
J21 = -2 * RW/(3 * RK * np.cos(ALPHA))
J22 = RW / (3 * RK * np.cos(ALPHA))
J23 = J22
J31 = RW / (3 * RK * np.sin(ALPHA))
J32 = J31
J33 = J31
J = np.array([[J11, J12, J13], [J21, J22, J23], [J31, J32, J33]])


def compute_phi(psi_1, psi_2, psi_3):
    '''
    Parameters:
    -----------
    psi_1: motor 1 encoder rotation (rad)
    psi_2: motor 2 encoder rotation (rad)
    psi_3: motor 3 encoder rotation (rad)

    Returns:
    --------
    phi_x: ball rotation along x-axis (rad)
    phi_y: ball rotation along y-axis (rad)
    phi_z: ball rotation along z-axis (rad)
    '''
    # converting counts to rad
    psi = np.array([[psi_1], [psi_2], [psi_3]])
    # phi = J [3x3] * psi [3x1]
    phi = np.matmul(J, psi)
    return phi[0][0], phi[1][0], phi[2][0]


def compute_motor_torques(Tx, Ty, Tz):
    '''
    Parameters:
    ----------
    Tx: x-axis torque
    Ty: y-axis torque
    Tz: z-aixs torque

    Returns:
    --------
    T1: motor 1 torque
    T2: motor 2 torque
    T3: motor 3 torque
    '''
    T1 = (-0.3333) * (Tz - (2.8284 * Ty))
    T2 = (-0.3333) * (Tz + (1.4142 * (Ty + 1.7320 * Tx)))
    T3 = (-0.3333) * (Tz + (1.4142 * (Ty - 1.7320 * Tx)))
    if T1 > 1.0:
        T1 = 1.0
    elif T1 < -1.0:
        T1 = -1.0
    if T2 > 1.0:
        T2 = 1.0
    elif T2 < -1.0:
        T2 = -1.0
    if T3 > 1.0:
        T3 = 1.0
    elif T3 < -1.0:
        T3 = -1.0
    return T1, T2, T3


# SETTING UP DPHI FILTERING AND PID
# controller gains
KP_DPHI_X = 0.0
KP_DPHI_Y = 0.0
KD_DPHI_X = 0.0
KD_DPHI_Y = 0.0

# parameters for lowpass filter for dphi
Fs = FREQ # sampling rate in Hz
Fc_dphi = 1.0 # cut-off frequency of the filter in Hz
Fn_dphi = Fc_dphi/Fs # normalized equivalent of Fc
N = 60 # taps of the filter
lowpass_filter_dphi_x = fir.FIR()
lowpass_filter_dphi_x.lowpass(N, Fn_dphi)
lowpass_filter_dphi_y = fir.FIR()
lowpass_filter_dphi_y.lowpass(N, Fn_dphi)

# SETTING UP THETA FILTERING AND PID
# controller gains
KP_THETA_X = 7.0
KP_THETA_Y = 7.0
KI_THETA_X = 0.09
KI_THETA_Y = 0.09
KD_THETA_X = 0.13
KD_THETA_Y = 0.13

# parameters for lowpass filter for dtheta
Fc_dtheta = 1.0 # cut-off frequency of the filter in Hz
Fn_dtheta = Fc_dtheta/Fs
lowpass_filter_dtheta_x = fir.FIR()
lowpass_filter_dtheta_x.lowpass(N, Fn_dtheta)
lowpass_filter_dtheta_y = fir.FIR()
lowpass_filter_dtheta_y.lowpass(N, Fn_dtheta)

# Fc_ctrl = 0.5 # cut-off frequency of the filter in Hz
# Fn_ctrl = Fc_ctrl/Fs
# lowpass_filter_ctrl_x = fir.FIR()
# lowpass_filter_ctrl_x.lowpass(N, Fn_ctrl)
# lowpass_filter_ctrl_y = fir.FIR()
# lowpass_filter_ctrl_y.lowpass(N, Fn_ctrl)

# parameters for wma filter for theta
WMA_WEIGHTS = np.array([0.1, 0.3, 0.4, 0.8])
WMA_WINDOW_SIZE = len(WMA_WEIGHTS)
WMA_NORM = WMA_WEIGHTS/np.sum(WMA_WEIGHTS)

def wma_filter(wma_window):
    return np.sum(WMA_NORM * wma_window)


if __name__ == "__main__":

    # SETTING UP LOGGING AND COMMUNICATION
    # prompt for trail number and set up DataLogger
    trial_num = int(input('trial number: '))
    filename = 'drive_test_%i' % trial_num
    dl = dataLogger(f"data/{filename}.txt")

    # set up and initialize serial communication thread
    ser_dev = SerialProtocol()
    register_topics(ser_dev)
    serial_read_thread = Thread(target = SerialProtocol.read_loop, args=(ser_dev,), daemon=True)
    serial_read_thread.start()

    # set up and initialize BT controller <-> RPi communication thread
    bt_controller = BalanceController(interface="/dev/input/js0")
    bt_controller_thread = threading.Thread(target=bt_controller.listen, args=(10,))
    bt_controller_thread.start()


    # INITIALIZING VARIABLES
    # communication structs
    commands = np.zeros(1, dtype=mo_cmds_dtype)[0]
    states = np.zeros(1, dtype=mo_states_dtype)[0]
    commands['kill'] = 0.0

    # initialize dphi error
    dphi_err_x = 0.0
    dphi_err_y = 0.0

    # initialize theta error
    theta_err_x = 0.0
    theta_err_y = 0.0
    i_theta_err_x = 0.0
    i_theta_err_y = 0.0

    # initialize wma filter for theta
    theta_x_window = deque(maxlen=WMA_WINDOW_SIZE) # A sliding window of values
    theta_y_window = deque(maxlen=WMA_WINDOW_SIZE) # A sliding window of values
    for _ in range(WMA_WINDOW_SIZE):
        theta_x_window.append(0.0)
        theta_y_window.append(0.0)

    # innitialize slew filter for bt_controller Lx and Ly
    lx_filter = SlewFilter(200, 2, 0)
    ly_filter = SlewFilter(200, 2, 0)

    # EXECUTING CONTROL LOOP
    # Time for comms to sync
    time.sleep(1.0)
    ser_dev.send_topic_data(101, commands)
    i = 0
    for t in SoftRealtimeLoop(dt=DT, report=True):
        try:
            states = ser_dev.get_cur_topic_data(121)[0]
            if i == 0:
                t_start = time.time()
            i = i + 1
        except KeyError as e:
            continue

        # capture time for logging
        t_now = time.time() - t_start


        # BALL VELOCITY (DPHI) PID
        # capture and filter dphi
        dpsi_1 = states['dpsi_1']
        dpsi_2 = states['dpsi_2']
        dpsi_3 = states['dpsi_3']
        dphi_x, dphi_y, dphi_z = compute_phi(dpsi_1, dpsi_2, dpsi_3)
        dphi_x = lowpass_filter_dphi_x.filter(dphi_x)
        dphi_y = lowpass_filter_dphi_y.filter(dphi_y)

        # compute dphi PID errors
        dphi_err_x_old = dphi_err_x
        dphi_err_y_old = dphi_err_y
        dphi_err_x = dphi_x - bt_controller.Ly * -0.5 # goal value set by bt_controller
        dphi_err_y = dphi_y - bt_controller.Lx * 0.5 # goal value set by bt_controller
        d_dphi_err_x = dphi_err_x - dphi_err_x_old
        d_dphi_err_y = dphi_err_y - dphi_err_y_old
        
        # compute PID torque inputs
        Tx_dphi = KP_DPHI_X * dphi_err_x + KD_DPHI_X * d_dphi_err_x
        Ty_dphi = KP_DPHI_Y * dphi_err_y + KD_DPHI_Y * d_dphi_err_y

        # LEAN ANGLE (THETA) PID
        # capture and filter theta
        theta_x_window.append(states['theta_roll'])
        theta_y_window.append(states['theta_pitch'])
        theta_x = wma_filter(theta_x_window)
        theta_y = wma_filter(theta_y_window)

        # filter lx and ly inputs
        lx_temp = lx_filter.filter(bt_controller.Lx)
        ly_temp = ly_filter.filter(bt_controller.Ly)

        # compute theta PID errors
        theta_err_x_old = theta_err_x
        theta_err_y_old = theta_err_y
        theta_err_x = theta_x - ly_temp*0.04 - bt_controller.up_down*0.005 # positive bias causes -y direction movement
        theta_err_y = theta_y - lx_temp*(-0.04) - bt_controller.right_left*(-0.005) # positive bias causes -x direction movement
        d_theta_err_x = theta_err_x - theta_err_x_old
        d_theta_err_y = theta_err_y - theta_err_y_old
        d_theta_err_x = lowpass_filter_dtheta_x.filter(d_theta_err_x)
        d_theta_err_y = lowpass_filter_dtheta_y.filter(d_theta_err_y)
        i_theta_err_x += theta_err_x
        i_theta_err_y += theta_err_y

        # TORQUE COMPUTATION
        # compute xyz torques from theta PID and BT controller and convert to 123 torques
        Tx = KP_THETA_X * theta_err_x + KD_THETA_X * d_theta_err_x + KI_THETA_X * i_theta_err_x #+ Tx_dphi
        Ty = KP_THETA_Y * theta_err_y + KD_THETA_Y * d_theta_err_y + KI_THETA_Y * i_theta_err_y #+ Ty_dphi 
        Tz = -bt_controller.Rx * 2.8
        T1, T2, T3 = compute_motor_torques(Tx, Ty, Tz)

        # handle torque saturation
        if np.abs(Tx) > MAX_PLANAR_DUTY:
            Tx = np.sign(Tx) * MAX_PLANAR_DUTY
        if np.abs(Ty) > MAX_PLANAR_DUTY:
            Ty = np.sign(Ty) * MAX_PLANAR_DUTY

        # construct data matrix for recording values from this iteration
        data = [i, t_now, states["psi_1"], states["psi_2"], states["psi_3"], dphi_z]
        dl.appendData(data)

        # print values from this iteration to terminal
        print(f"biases: y[{bt_controller.right_left}]   x[{bt_controller.up_down}]")
        
        # send motor commands
        commands['motor_1_duty'] = T1
        commands['motor_2_duty'] = T2
        commands['motor_3_duty'] = T3
        ser_dev.send_topic_data(101, commands)
    
    # write recorded data with DataLogger
    dl.writeOut()

    # reset torques and shutdown
    time.sleep(0.25)
    commands['motor_1_duty'] = 0.0
    commands['motor_2_duty'] = 0.0
    commands['motor_3_duty'] = 0.0
    time.sleep(0.25)
    commands['kill'] = 1.0
    time.sleep(0.25)
    ser_dev.send_topic_data(101, commands)
    time.sleep(0.25)
