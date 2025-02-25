# Tuan Luong
# tdluong@crimson.ua.edu
# 2025/02/24
import argparse
import numpy as np
import matplotlib.pyplot as plt

class ScanPattern(object):
    frame_rate_hz = 0
    frame_period_s = 0
    num_point_per_line = 0
    num_scanline = 0
    total_point = 0
    time_vec_ns = np.array([])
    az_vec_deg = np.array([])
    el_vec_deg = np.array([])

    def __init__(self, frame_period_s, total_point, num_scanline, time_vec_ns, az_vec_deg, el_vec_deg):
        self.frame_period_s = frame_period_s
        self.frame_rate_hz = 1/frame_period_s
        self.num_scanline = num_scanline
        self.total_point = total_point
        self.num_point_per_line = total_point / num_scanline
        self.time_vec_ns = time_vec_ns
        self.az_vec_deg = az_vec_deg
        self.el_vec_deg = el_vec_deg




def parse_arg():
    # Expose as much real life configuration from Blickfeld Cube1 as posible along with emulate real-life constraint
    # The mirror frequency is not explicitly claimed but trial and errors shows that it is close to 250Hz. It is use selectable but should not be changed 
    # It is implied that the horizontal mirror oscilates with 80 deg amplitude and this is not user configurable in the documentation
    # This sim only works for the "combined" scan mode. Future implementation might include "up-only" and "down-only". RTX LiDAR implementation does not allow for separate
    # Only "equidistant" pulse pattern implemented. Future implementation might include "Interleave" as well
    
    parser = argparse.ArgumentParser(
                                     description="Generates scan patterns based for Cube1 based on user input and output JSON file for import in Omniverse as RTX LiDAR\n \
The output file is labeled BF1_<FREQ>_<HOR_MEAS_FOV>_<VER_MEAS_FOV>_<HOR_ANG_RES>_<NUM_SCAN_UP>_<NUM_SCAN_DOWN>.json",
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('-f', '--freq', type=int, default=250, help='Mirrors eigen frquency. Defaults to 250 Hz.')
    parser.add_argument('--hor_meas_fov', type=int, default=70, help='Horizontal measurement FOV in degrees. Defaults to 70 deg [1 - 72]')
    parser.add_argument('--ver_meas_fov', type=int, default=30, help='Vertical measuremetn FOV in degrees. Defaults to 30 deg [1 - 30]')
    parser.add_argument('--hor_ang_res', type=int, default=4, help='Horizontal angular resolution of measurements in deci degrees. Defaults to 4 deci deg [4 - 10] ')
    parser.add_argument('--num_scan_up', type=int, default=200, help='Number of scanlines in the up ramp phase. Default is 200 [1 - 200]')
    parser.add_argument('--num_scan_down', type=int, default=200, help='Number of scanlines in the down ramp phase. Default is 200 [1 - 200]')
    return parser

def is_valid_params(lidar_params):
    if not 1 <= lidar_params.hor_meas_fov <= 72:
        print("Invalid HOR_MEAS_FOV. Valid range is [1 - 72]")
        return False

    if not 1 <= lidar_params.ver_meas_fov <= 30:
        print("Invalid VER_MEAS_FOV. Valid range is [1 - 30]")
        return False

    if not 4 <= lidar_params.hor_ang_res <= 10:
        print("Invalid HOR_ANG_RES. Valid range is [4 - 10]")
        return False

    if not 1 <= lidar_params.num_scan_up <= 200:
        print("Invalid NUM_SCAN_UP. Valid range is [1 - 200]")
        return False

    if not 1 <= lidar_params.num_scan_down <= 200:
        print("Invalid NUM_SCAN_DOWN. Valid range is [1 - 200]")
        return False

    return True

def gen_scan_pattern(lidar_params):
    # Pre compute some constants
    HOR_MIR_FOV_MDEG = 80 * 1000 # Mirror FOV. Is not configurable
    hor_ang_res_mdeg = lidar_params.hor_ang_res * 100
    tot_num_scanline = lidar_params.num_scan_up + lidar_params.num_scan_down
    ramp_transition = lidar_params.num_scan_up / tot_num_scanline
    period_per_scanline_s = 1/lidar_params.freq
    frame_period_s = tot_num_scanline * period_per_scanline_s/2

    # Generate a time vector of points as they are equidistant in angle, not time
    angle_to_trig_down_mdeg = np.arange(HOR_MIR_FOV_MDEG/2, -HOR_MIR_FOV_MDEG/2, -hor_ang_res_mdeg)

    time_vec_half_period = np.arccos(2 * angle_to_trig_down_mdeg / HOR_MIR_FOV_MDEG) / (2 * np.pi * lidar_params.freq)

    time_vec_s = time_vec_half_period.copy()

    for ii in range(1,tot_num_scanline):
        time_vec_s = np.concatenate((time_vec_s,0.5 * ii * period_per_scanline_s + time_vec_half_period))
    time_vec_ns = np.round(time_vec_s * 1000000000)

    # Mirror movement function
    # Horizontal mirror
    hor_ang_deg = np.round(40 * np.cos(2 * np.pi * lidar_params.freq * time_vec_s),2)

    # Vertical mirror
    ramp_func = np.zeros_like(hor_ang_deg)
    ramp_transition_time = ramp_transition * frame_period_s

    # Line intercept of the down ramp amplitude function
    slope = -1 / (frame_period_s - ramp_transition_time)
    c = frame_period_s / (frame_period_s - ramp_transition_time)
    for ii in range(ramp_func.size):
        cur_t = time_vec_s[ii]
        if cur_t <= ramp_transition_time:
            # Ramp up
            ramp_func[ii] = 1/ramp_transition_time * cur_t
        else:
            ramp_func[ii] = slope * cur_t + c
    
    ver_ang_deg = np.round((0.5 * lidar_params.ver_meas_fov * np.sin(2 * np.pi * lidar_params.freq * time_vec_s)) * ramp_func,2)

    # Remove entris outside the horizontal meas FOV
    ind_out_FOV = np.flatnonzero(np.abs(hor_ang_deg) > lidar_params.hor_meas_fov/2)
    time_vec_ns = np.delete(time_vec_ns, ind_out_FOV)
    hor_ang_deg = np.delete(hor_ang_deg, ind_out_FOV)
    ver_ang_deg = np.delete(ver_ang_deg, ind_out_FOV)

    num_point = ver_ang_deg.size

    return ScanPattern(frame_period_s, num_point, tot_num_scanline, time_vec_ns, hor_ang_deg, ver_ang_deg)
    


if __name__ == "__main__":
    lidar_params = parse_arg().parse_args()

    # Construct output file name  
    output_json_filename = "BF1_"+str(lidar_params.freq)+"_"+str(lidar_params.hor_meas_fov)+"_"+str(lidar_params.ver_meas_fov)+"_"+str(lidar_params.hor_ang_res)+ \
        "_"+str(lidar_params.num_scan_up)+"_"+str(lidar_params.num_scan_down)+".json"
    
    # Check lidar param values
    if not is_valid_params(lidar_params):
        exit()

    scan_pattern = gen_scan_pattern(lidar_params)

    print(scan_pattern.time_vec_ns)