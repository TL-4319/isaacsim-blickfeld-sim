# Tuan Luong
# tdluong@crimson.ua.edu
# 2025/02/24
import argparse

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
    parser.add_argument('--hor_meas_fov', type=int, default=72, help='Horizontal measurement FOV in degrees. Defaults to 72 deg [1 - 72]')
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
    # Return arrays of size 3xN where N is the number of points in the pattern.
    # The information encodes in the rows are
    # Row 1 - offset from start of frame in nano second
    # Row 2 - azimuth in radian
    # Row 3 - elevation in radian

    hor_ang_res_mdeg = lidar_params.hor_ang_res * 100 
    
    # Generate mirror ramp functions
    tot_num_scanline = lidar_params.num_scan_up + lidar_params.num_scan_down
    ramp_transition = lidar_params.num_scan_up / tot_num_scanline
    period_per_scanline_s = 1/lidar_params.freq
    frame_period_s = tot_num_scanline * period_per_scanline_s
    num_point_per_scanline = lidar_params.hor_meas_fov * 1000 / hor_ang_res_mdeg
    total_num_point = num_point_per_scanline * tot_num_scanline
    


if __name__ == "__main__":
    lidar_params = parse_arg().parse_args()

    # Construct output file name  
    output_json_filename = "BF1_"+str(lidar_params.freq)+"_"+str(lidar_params.hor_meas_fov)+"_"+str(lidar_params.ver_meas_fov)+"_"+str(lidar_params.hor_ang_res)+ \
        "_"+str(lidar_params.num_scan_up)+"_"+str(lidar_params.num_scan_down)+".json"
    
    # Check lidar param values
    if not is_valid_params(lidar_params):
        exit()


    print(output_json_filename)