# Tuan Luong
# tdluong@crimson.ua.edu
# 2025/02/24
import argparse
import numpy as np

class ScanPattern(object):
    frame_rate_hz = 0
    frame_period_s = 0
    num_point_per_line = 0
    num_scanline = 0
    total_point = 0
    time_vec_ns = np.array([])
    az_vec_deg = np.array([])
    el_vec_deg = np.array([])
    HFOV = 0
    VFOV = 0
    # Store mirror movement
    mirror_vec_deg = np.array([]) 
    mirror_time_vec = np.array([])

    def __init__(self, frame_period_s, total_point, num_scanline, time_vec_ns, az_vec_deg, el_vec_deg, hfov, vfov, mirror_hor_vec, mirror_time_vec):
        self.frame_period_s = frame_period_s
        self.frame_rate_hz = 1/frame_period_s
        self.num_scanline = num_scanline
        self.total_point = total_point
        self.num_point_per_line = total_point / num_scanline
        self.time_vec_ns = time_vec_ns
        self.az_vec_deg = az_vec_deg
        self.el_vec_deg = el_vec_deg
        self.HFOV = hfov
        self.VFOV = vfov
        self.mirror_vec_deg = mirror_hor_vec
        self.mirror_time_vec = mirror_time_vec

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
    parser.add_argument('--draw', action='store_true', help='Generate visualization of mirror movement and projected scan pattern')
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

def gen_scan_pattern(lidar_params) -> ScanPattern:
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
    mirror_hor_ang_deg = hor_ang_deg.copy()
    mirror_time_vec = time_vec_ns.copy()
    ind_out_FOV = np.flatnonzero(np.abs(hor_ang_deg) > lidar_params.hor_meas_fov/2)
    time_vec_ns = np.delete(time_vec_ns, ind_out_FOV)
    hor_ang_deg = np.delete(hor_ang_deg, ind_out_FOV)
    ver_ang_deg = np.delete(ver_ang_deg, ind_out_FOV)

    num_point = ver_ang_deg.size

    return ScanPattern(frame_period_s, num_point, tot_num_scanline, time_vec_ns, 
                       hor_ang_deg, ver_ang_deg, lidar_params.hor_meas_fov, lidar_params.ver_meas_fov, mirror_hor_ang_deg,mirror_time_vec)

def print_field(file_obj, key: str, val, comma = True, dec_place = 0, indent_level=1):
    for ii in range(indent_level):
        file_obj.write("  ")

    if isinstance(val,str):
        file_obj.write("\"" + key + "\": \"" + val + "\"")
    else:
        if dec_place == 0:
            # Integer printing
            file_obj.write("\"" + key + "\": " + str(val))  
        else:
            # Float val with defined decimal places
            val_str = "{:.{prec}f}".format(val, prec=dec_place)
            file_obj.write("\"" + key + "\": " + val_str)

    if comma:
        file_obj.write(",\n")
    else:
        file_obj.write("\n")
    file_obj.flush()

def print_array(file_obj, key: str, val, comma = True, dec_place = 0, indent_level=1):
    for ii in range(indent_level):
        file_obj.write("  ")
    
    file_obj.write("\"" + key + "\": [\n") 

    num_row, num_col = val.shape

    for ii in range(num_row):
        for jj in range(indent_level+1):
            file_obj.write("  ")

        for jj in range(num_col):
            if dec_place == 0:
                file_obj.write(str(int(val[ii][jj])))
            else:
                val_str = "{:.{prec}f}".format(val[ii][jj], prec=dec_place)
                file_obj.write(val_str)
            if not jj == num_col-1:
                file_obj.write(", ")
        if not ii == num_row-1:
            file_obj.write(",\n")
        else:
            file_obj.write("\n")

    for ii in range(indent_level+1):
        file_obj.write("  ")
    file_obj.write("]")

    if comma:
        file_obj.write(",\n")
    else:
        file_obj.write("\n")
    file_obj.flush()

def create_lidar_json(scan_pattern: ScanPattern, filename: str):
    f = open(filename+".json", "w")
    f.write("{\n")
    # Write meta data
    print_field(f,"name",filename)
    print_field(f, "class", "sensor")
    print_field(f, "type", "lidar")
    print_field(f, "driveWorksId", "GENERIC")
    print_field(f, "comment1", "Automatically generated using https://github.com/TL-4319/isaacsim-blickfeld-sim")
    
    # Scan profile
    f.write("  \"profile\": {\n")
    print_field(f, "scanType", "solidState", indent_level=2)
    print_field(f, "intensityProcessing", "normalization", indent_level=2)
    print_field(f, "rayType", "IDEALIZED", indent_level=2)
    print_field(f, "nearRangeM", 1.5, dec_place=1, indent_level=2)
    print_field(f, "farRangeM", 75, indent_level=2)
    print_field(f, "effectiveApertureSize", 0.01, dec_place=2, indent_level=2)
    print_field(f, "focusDistM", 0.12, dec_place=2, indent_level=2)
    print_field(f, "startAzimuthDeg", -scan_pattern.HFOV/2, dec_place=1, indent_level=2)
    print_field(f, "endAzimuthDeg", scan_pattern.HFOV/2, dec_place=1, indent_level=2)
    print_field(f, "upElevationDeg", scan_pattern.VFOV/2, dec_place=1, indent_level=2)
    print_field(f, "downElevationDeg", scan_pattern.VFOV/2, dec_place=1, indent_level=2)
    print_field(f, "rangeResolutionM", 0.01, dec_place=2, indent_level=2)
    print_field(f, "rangeAccuracyM", 0.02, dec_place=2, indent_level=2)
    print_field(f, "avgPowerW", 9, indent_level=2)
    print_field(f, "minReflectance", 0.1, dec_place=1, indent_level=2)
    print_field(f, "minReflectanceRange", 30, dec_place=1, indent_level=2)
    print_field(f, "wavelengthNm", 905, indent_level=2)
    print_field(f, "pulseTimeNs", 6, indent_level=2)
    print_field(f, "maxReturns", 1, indent_level=2)
    print_field(f, "scanRateBaseHz", scan_pattern.frame_rate_hz, indent_level=2)
    print_field(f, "reportRateBaseHz", scan_pattern.frame_rate_hz, indent_level=2)
    print_field(f, "numberOfEmitters", scan_pattern.total_point, indent_level=2)
    print_field(f, "numberOfChannels", scan_pattern.total_point, indent_level=2)
    # Obtained from spec
    print_field(f, "azimuthErrorMean", 0, indent_level=2)
    print_field(f, "azimuthErrorStd", 0.025, dec_place=3, indent_level=2)
    print_field(f, "elevationErrorMean", 0, indent_level=2)
    print_field(f, "elevationErrorStd", 0.025, dec_place=3, indent_level=2)
    print_field(f, "stateResolutionStep", 1, indent_level=2)
    print_field(f, "numLines", scan_pattern.num_scanline, indent_level=2)

    # Handling of array type fields
    num_ray_per_line_array = np.ones((scan_pattern.num_scanline,1)) * scan_pattern.num_point_per_line
    print_array(f, "numRaysPerLine", num_ray_per_line_array, indent_level=2)
    print_field(f, "emitterStateCount", 1, indent_level=2)

    # Handling of emmite state object which include all arrays of az, el and firing time
    f.write("    \"emitterStates\": [\n      {\n")

    # Construct arrays with num_line rows and num_points_per_line columns
    az_mat = scan_pattern.az_vec_deg.reshape((int(scan_pattern.num_scanline), int(scan_pattern.num_point_per_line)))
    el_mat = scan_pattern.el_vec_deg.reshape((int(scan_pattern.num_scanline), int(scan_pattern.num_point_per_line)))
    time_mat = scan_pattern.time_vec_ns.reshape((int(scan_pattern.num_scanline), int(scan_pattern.num_point_per_line)))
    
    print_array(f, "azimuthDeg", az_mat, dec_place=2, indent_level=3)
    print_array(f, "elevationDeg", el_mat, dec_place=2, indent_level=3)
    print_array(f, "fireTimeNs", time_mat, comma=False, indent_level=3)

    # Closing characters for emmiterStates object
    f.write("      }\n    ],\n")

    print_field(f, "intensityMappingType", "LINEAR", comma=False)

    # Closing characters for document
    f.write("  }\n")
    f.write("}")
    f.close()
    
def visualize(scan_pattern: ScanPattern, project_dist:float):
    import matplotlib.pyplot as plt

    ax1 = plt.subplot(1,2,1)
    ax1.plot(scan_pattern.mirror_time_vec, scan_pattern.mirror_vec_deg, label='Hor')
    ax1.plot(scan_pattern.time_vec_ns, scan_pattern.el_vec_deg, label='Ver')
    ax1.set_xlabel('Time (ns)', fontsize=20)
    ax1.set_ylabel('Mirror angle ($^o$)',fontsize=20)
    ax1.set_title('Mirror movement',fontsize=20)
    ax1.legend()

    # Project the point to a plane
    DEG2RAD = 0.0174533
    x_proj = project_dist * np.tan(DEG2RAD * scan_pattern.az_vec_deg)
    y_proj = project_dist * np.tan(DEG2RAD * scan_pattern.el_vec_deg)
    color_vec = scan_pattern.time_vec_ns / scan_pattern.time_vec_ns[-1]
    ax2 = plt.subplot(1,2,2)
    ax2.scatter(x_proj, y_proj, 2,color_vec)
    ax2.set_xlabel('X (m)', fontsize=20)
    ax2.set_ylabel('Y (m)',fontsize=20)
    ax2.set_title('Projected scan pattern',fontsize=20)
    plt.show()


if __name__ == "__main__":
    lidar_params = parse_arg().parse_args()

    # Construct output file name  
    output_filename = "BF1_"+str(lidar_params.freq)+"_"+str(lidar_params.hor_meas_fov)+"_"+str(lidar_params.ver_meas_fov)+"_"+str(lidar_params.hor_ang_res)+ \
        "_"+str(lidar_params.num_scan_up)+"_"+str(lidar_params.num_scan_down)
    
    # Check lidar param values
    if not is_valid_params(lidar_params):
        exit()

    # Generate scan patter
    scan_pattern = gen_scan_pattern(lidar_params)

    # Generate JSON file
    create_lidar_json(scan_pattern, output_filename)

    # Visualize
    if lidar_params.draw:
        visualize(scan_pattern, 2)