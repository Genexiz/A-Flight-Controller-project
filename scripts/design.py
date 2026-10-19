"""Netlist of the MiniDrone flight controller (single source of truth).

Every visible pin of every symbol must be listed: a net name, or None for
no-connect. gen_schematic.py refuses to run if a pin is missing.
"""

FP_R = "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder"
FP_C = "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder"
FP_C1206 = "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder"
FP_LED = "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder"
FP_SOT23 = "Package_TO_SOT_SMD:SOT-23"
FP_SOD123 = "Diode_SMD:D_SOD-123"
FP_MOTOR = "Connector_Wire:SolderWire-0.15sqmm_1x02_P4mm_D0.5mm_OD1.5mm"

# (ref, lib_id, value, footprint, {pin: net}, block, mpn)
C = []

def add(ref, lib, value, fp, pins, block, mpn=""):
    C.append(dict(ref=ref, lib=lib, value=value, fp=fp, pins=pins, block=block, mpn=mpn))

def R(ref, val, a, b, block, mpn=""):
    add(ref, "Device:R", val, FP_R, {"1": a, "2": b}, block, mpn)

def Cap(ref, val, a, b, block, fp=FP_C, mpn=""):
    add(ref, "Device:C", val, fp, {"1": a, "2": b}, block, mpn)

# ---------------- Power ----------------
add("J1", "Connector_Generic:Conn_01x02", "BATT 1S", "Connector_JST:JST_PH_S2B-PH-SM4-TB_1x02-1MP_P2.00mm_Horizontal",
    {"1": "VBAT", "2": "GND"}, "power", "S2B-PH-SM4-TB")
Cap("C1", "100uF 6.3V", "VBAT", "GND", "power", FP_C1206, "GRM31CR60J107ME39")
Cap("C2", "22uF", "VBAT", "GND", "power")
Cap("C3", "22uF", "VBAT", "GND", "power")
# Power path: USB -> Schottky -> VIN ; battery -> P-MOSFET (off while USB present) -> VIN
add("Q5", "Transistor_FET:AO3401A", "AO3401A", FP_SOT23, {"1": "VBUS", "2": "VIN", "3": "VBAT"}, "power", "AO3401A")
R("R20", "100k", "VBUS", "GND", "power")
add("D1", "Device:D_Schottky", "B5819W", FP_SOD123, {"1": "VIN", "2": "VBUS"}, "power", "B5819W")
add("U2", "Regulator_Linear:AP2112K-3.3", "AP2112K-3.3", "Package_TO_SOT_SMD:SOT-23-5",
    {"1": "VIN", "2": "GND", "3": "VIN", "5": "+3V3"}, "power", "AP2112K-3.3TRG1")
Cap("C4", "10uF", "VIN", "GND", "power")
Cap("C5", "10uF", "+3V3", "GND", "power")
# Battery voltage sense (VBAT/2 -> ADC)
R("R15", "100k", "VBAT", "VBAT_SENSE", "power")
R("R16", "100k", "VBAT_SENSE", "GND", "power")
Cap("C15", "100nF", "VBAT_SENSE", "GND", "power")
for n in ("VBAT", "GND", "VBUS", "VIN"):
    add("#FLG_" + n, "power:PWR_FLAG", "PWR_FLAG", "", {"1": n}, "power")

# ---------------- MCU ----------------
esp = {
    # pins chosen so each signal leaves the module on the side facing its load
    "1": "GND", "2": "+3V3", "3": "EN",
    "4": "MOT4_PWM",                 # IO4  -> front-left motor
    "11": "MOT3_PWM",                # IO18 -> rear-left motor
    "23": "MOT2_PWM",                # IO21 -> rear-right motor
    "39": "MOT1_PWM",                # IO1  -> front-right motor
    "17": "VBAT_SENSE",              # IO9  (ADC1_CH8)
    "19": "I2C_SDA",                 # IO11
    "20": "I2C_SCL",                 # IO12
    "22": "IMU_INT",                 # IO14
    "38": "LED_R",                   # IO2
    "33": "LED_G",                   # IO40
    "27": "BOOT",                    # IO0 (strapping, boot button)
    "13": "USB_DN", "14": "USB_DP",  # IO19 / IO20 native USB
    "36": "UART_RX", "37": "UART_TX",
    "5": None, "6": None, "7": None, "8": None, "9": None, "10": None, "12": None,
    "15": None, "16": None, "18": None, "21": None, "24": None, "25": None, "26": None,
    "28": None, "29": None, "30": None, "31": None, "32": None, "34": None, "35": None,
}
add("U1", "RF_Module:ESP32-S3-WROOM-1", "ESP32-S3-WROOM-1-N8", "RF_Module:ESP32-S3-WROOM-1", esp, "mcu",
    "ESP32-S3-WROOM-1-N8")
Cap("C6", "22uF", "+3V3", "GND", "mcu")
Cap("C7", "100nF", "+3V3", "GND", "mcu")
R("R1", "10k", "+3V3", "EN", "mcu")
Cap("C8", "1uF", "EN", "GND", "mcu")
add("SW1", "Switch:SW_Push", "RESET", "Button_Switch_SMD:SW_SPST_PTS810", {"1": "EN", "2": "GND"}, "mcu", "PTS810SJM250SMTRLFS")
R("R2", "10k", "+3V3", "BOOT", "mcu")
add("SW2", "Switch:SW_Push", "BOOT", "Button_Switch_SMD:SW_SPST_PTS810", {"1": "BOOT", "2": "GND"}, "mcu", "PTS810SJM250SMTRLFS")
add("J3", "Connector_Generic:Conn_01x06", "DEBUG", "Connector_PinHeader_1.27mm:PinHeader_1x06_P1.27mm_Vertical",
    {"1": "+3V3", "2": "GND", "3": "UART_TX", "4": "UART_RX", "5": "I2C_SDA", "6": "I2C_SCL"}, "mcu")
R("R17", "1k", "LED_R", "LED_R_A", "mcu")
add("D6", "Device:LED", "RED", FP_LED, {"1": "GND", "2": "LED_R_A"}, "mcu")
R("R18", "1k", "LED_G", "LED_G_A", "mcu")
add("D7", "Device:LED", "GREEN", FP_LED, {"1": "GND", "2": "LED_G_A"}, "mcu")

# ---------------- USB ----------------
add("J2", "Connector:USB_C_Receptacle_USB2.0_16P", "USB-C", "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12",
    {"A1": "GND", "A4": "VBUS", "A5": "CC1", "A6": "USB_DP", "A7": "USB_DN", "A8": None,
     "B5": "CC2", "B6": "USB_DP", "B7": "USB_DN", "B8": None, "SH": "GND"}, "usb", "TYPE-C-31-M-12")
R("R3", "5.1k", "CC1", "GND", "usb")
R("R4", "5.1k", "CC2", "GND", "usb")
Cap("C9", "4.7uF", "VBUS", "GND", "usb")

# ---------------- IMU ----------------
add("U3", "Sensor_Motion:MPU-6050", "MPU-6050", "Sensor_Motion:InvenSense_QFN-24_4x4mm_P0.5mm",
    {"1": "GND", "6": None, "7": None, "8": "+3V3", "9": "GND", "10": "IMU_REGOUT", "11": "GND",
     "12": "IMU_INT", "13": "+3V3", "18": "GND", "20": "IMU_CPOUT", "23": "I2C_SCL", "24": "I2C_SDA"},
    "imu", "MPU-6050")
Cap("C10", "100nF", "+3V3", "GND", "imu")   # VDD
Cap("C11", "10nF", "+3V3", "GND", "imu")    # VLOGIC
Cap("C12", "100nF", "IMU_REGOUT", "GND", "imu")
Cap("C13", "2.2nF", "IMU_CPOUT", "GND", "imu")
R("R5", "4.7k", "+3V3", "I2C_SDA", "imu")
R("R6", "4.7k", "+3V3", "I2C_SCL", "imu")

# ---------------- Motors ----------------
for i in range(1, 5):
    blk = "motors"
    R("R%d" % (6 + i), "100", "MOT%d_PWM" % i, "MOT%d_G" % i, blk)
    R("R%d" % (10 + i), "10k", "MOT%d_G" % i, "GND", blk)
    add("Q%d" % i, "Transistor_FET:AO3400A", "AO3400A", FP_SOT23,
        {"1": "MOT%d_G" % i, "2": "GND", "3": "MOT%d_N" % i}, blk, "AO3400A")
    add("D%d" % (1 + i), "Device:D_Schottky", "B5819W", FP_SOD123,
        {"1": "VBAT", "2": "MOT%d_N" % i}, blk, "B5819W")
    add("M%d" % i, "Motor:Motor_DC", "Motor %s" % ("CW" if i in (1, 4) else "CCW"), FP_MOTOR,
        {"1": "VBAT", "2": "MOT%d_N" % i}, blk)

BLOCK_TITLES = {
    "power": "POWER: 1S LiPo -> power path -> 3.3V LDO, battery sense",
    "mcu": "MCU: ESP32-S3-WROOM-1, reset/boot, LEDs, debug header",
    "usb": "USB-C (native USB on GPIO19/20, programming + 5V)",
    "imu": "IMU: MPU-6050 (I2C addr 0x68)",
    "motors": "MOTOR DRIVERS: 4x low-side N-MOSFET + flyback diode",
}
