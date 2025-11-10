# File: GS2_Lidar.py
# Author: Robert Stevenson
# Date: 05/09/2023 (DD/MM/YYYY)
# Description:
# Class for communication with GS2 Lidar

"""
NOTES:
- OBTAINING DEVICE ADDRESS:
    - n cascading, if N devices (up to 3 supported) are threaded, the command returns N answers at 0x01, 0x02, 0x04, corresponding to 1-3 modules respectively.
- Default Baud rate of the device is 921600
- 
- 
- 
"""

import serial
import cmath as math


# Lidar class
class GS2_Lidar:
    # Packet header (Fixed Values)
    GS2_PACKET_HEADER = b"\xA5\xA5\xA5\xA5"
    # Packet types/Command types
    GS2_GET_DEVICE_ADDRESS = b"\x60"
    GS2_GET_DEVICE_PARAMETERS = b"\x61"
    GS2_GET_DEVICE_VERSION_INFO = b"\x62"
    GS2_START_SCAN = b"\x63"
    GS2_STOP_SCAN = b"\x64"
    GS2_SOFT_RESET = b"\x67"
    GS2_SET_BAUD_RATE = b"\x68"
    GS2_SET_EDGE_MODE = b"\x69"

    # Initialize the lidar
    def __init__(self, serial_port, baud_rate=921600, autoBegin=True):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        # device address
        self.device_address = b"\x00"
        self.version_number = b""
        self.serial_number = b""
        # device parameters
        # The K and B received by the protocol are of uint16 type, which need to be converted to float type and then divided by 10000 before being substituted into the calculation function.
        self.compensated_k0 = 0.0
        self.compensated_k1 = 0.0
        self.compensated_b0 = 0.0
        self.compensated_b1 = 0.0
        self.bias = 0.0  # Bias is of type int8, which needs to be converted to float type and divided by 10 before substituting into the calculation function
        # Point calculation parameters (Source: GS2 SDK - Line 109-111 - https://github.com/YDLIDAR/EaiSdkForGS2/blob/dd417d6ffae6bd10c4f8d9e6c3d8e155b282433e/include/ydlidar_protocol.h#L109 (accessed 07/09/2023))
        self.angle_p_x = 1.22
        self.angle_p_y = 5.315
        self.angle_p_angle = 22.5
        # serial port object
        self.ser = serial.Serial(serial_port, baud_rate, timeout=1)

        if autoBegin:
            self.initialise_device()

    def initialise_device(self):
        """
        Initializes the device by performing the following actions:

        - Opens the serial port if it is not already open.
        - Fetches the device address.
        - Fetches the device version information.
        - Fetches the device parameters.
        """
        # open the serial port if not open
        if not self.ser.isOpen():
            self.ser.open()
        # fetch the device info
        self.fetch_device_address()
        self.fetch_device_version_info()
        self.fetch_device_parameters()

    def fetch_device_address(self):
        """
        Fetches the device address from the command response and stores it in itself.

        """
        # get the device address from the command response
        response = self.send_command(
            device_address=b"\x00",
            packet_type=GS2_Lidar.GS2_GET_DEVICE_ADDRESS,
            data_length=b"",
            data_segment=b"\x00\x00",
            check_code=b"\x60",
        )

        # Extract the device address from the response
        self.device_address = response[4:5]

    def fetch_device_version_info(self):
        """
        Fetches the device version and serial number information by sending a command to the device and extracting the response to store in itself.

        """
        # get the device parameters from the command response
        response = self.send_command(
            device_address=b"\x00",
            packet_type=GS2_Lidar.GS2_GET_DEVICE_VERSION_INFO,
            data_length=b"\x00\x00",
            data_segment=b"",
            check_code=b"\x62",
        )

        # response_data_len = response[6:8]
        response_data_segment = response[8:-1]
        # response_check_code = response[-1:]

        self.version_number = response_data_segment[:3]
        self.serial_number = response_data_segment[3:]

        # print("Response:")
        # print(response)
        # print("Response (Byte Array):")
        # print(bytearray(response))
        # # print the response data len
        # print("Response data len:")
        # print(response_data_len)
        # # print the response data segment
        # print("Response data segment:")
        # print(response_data_segment)
        # # print the response check code
        # print("Response check code:")
        # print(response_check_code)

    def fetch_device_parameters(self):
        """
        Fetches the device parameters from the command response.
        Function fetches the and calculates the compensated K and B values along with the bias value of the device.

        """
        # get the device parameters from the command response
        response = self.send_command(
            device_address=b"\x00",
            packet_type=GS2_Lidar.GS2_GET_DEVICE_PARAMETERS,
            data_length=b"\x00\x00",
            data_segment=b"",
            check_code=b"\x61",
        )

        response_data_len = response[6:8]
        response_data_segment = response[8:-1]
        response_check_code = response[-1:]

        # print("Response:")
        # print(response)
        # print("Response (Byte Array):")
        # print(bytearray(response))
        # # print the response data len
        # print("Response data len (byte):")
        # print(response_data_len)
        # print("Response data len:")
        # print(int.from_bytes(response_data_len, byteorder="little"))
        # # print the response data segment
        # print("Response data segment:")
        # print(response_data_segment)
        # print("Response data segment (Length):")
        # print(len(response_data_segment))
        # # print the response check code
        # print("Response check code:")
        # print(response_check_code)

        # Extract the parameters based on the provided byte offset information
        k0_bytes = response_data_segment[0:1]
        b0_bytes = response_data_segment[2:3]
        k1_bytes = response_data_segment[4:5]
        b1_bytes = response_data_segment[6:7]
        bias_byte = response_data_segment[7]
        # Convert the bytes to integers (LSB first for 2-byte values)
        k0 = int.from_bytes(k0_bytes, byteorder="little")
        b0 = int.from_bytes(b0_bytes, byteorder="little")
        k1 = int.from_bytes(k1_bytes, byteorder="little")
        b1 = int.from_bytes(b1_bytes, byteorder="little")
        bias = int.from_bytes(bytes([bias_byte]), byteorder="little")

        # store the compensated values
        self.compensated_k0 = k0 / 10000.0
        self.compensated_k1 = k1 / 10000.0
        self.compensated_b0 = b0 / 10000.0
        self.compensated_b1 = b1 / 10000.0
        self.bias = (float)(bias) / 10.0

    def get_device_address(self):
        """
        Get the device address.

        Returns:
            The device address.
        """
        return self.device_address

    def send_command(
        self, device_address, packet_type, data_length, data_segment, check_code
    ):
        """
        Sends a command to the device with the specified address and retrieves the response.

        Parameters:
            device_address (str): The address of the device. (1 Byte Length)
            packet_type (str): The type of packet to be sent. (1 Byte Length)
            data_length (str): The expected length of the data. (2 Byte Length)
            data_segment (str): The data segment to be sent. (N Byte Length)
            check_code (str): The check code for validating the command. (1 Byte Length)

        Returns:
            str: The response received from the device.
        """
        command = (
            self.GS2_PACKET_HEADER  # The Header for the lidar's commands is fixed
            + device_address
            + packet_type
            + data_length
            + data_segment
            + check_code
        )

        # print("Sending: ")
        # print(command)

        # Send a command to the lidar
        self.ser.write(command)

        # Read the response from the lidar
        response = self.ser.readline()

        # return the response
        return response

    def start_scan(self):
        """
        Starts the lidar scan.

        """
        response = self.send_command(
            device_address=b"\x00",
            packet_type=GS2_Lidar.GS2_START_SCAN,
            data_length=b"\x00\x00",
            data_segment=b"",
            check_code=b"\x63",
        )

    def get_basic_scan_data(self):
        """
        Reads 322 bytes from the lidar and extracts the ambient light intensity and ranging points.
        Calculates the distance and intensity for each ranging point.

        Returns:
            dist_list (list): A list of distances for each ranging point.
            intens_list (list): A list of intensities for each ranging point.
        """
        # read 322 bytes from the lidar
        scan_data = self.ser.read(322)
        # get the ENV data bytes (First 2 bytes) and the 160 ranging points (S1 - S160, Remaining bytes)
        env_data = scan_data[0:2]  # Ambient Light intensity
        ranging_data = scan_data[2:]
        # every 2 bytes is a single ranging point with the upper 7 bits being the intensity data and the lower 9 bits being the distance data

        # Lists to Store the distance and intensity to return later
        dist_list = []
        intens_list = []

        # Si(2B) == Distance Measurement data ==> The lower 9 bits are the distance, the upper 7 bits are the intensity value
        # Distance calculation formula: Distance𝑖=(𝑆𝑖_𝑀𝑆𝐵≪8|𝑆𝑖_𝐿𝑆𝐵) &0x01ff, unit is mm.
        for i in range(0, 160):
            # Combine Si_MSB and Si_LSB into a 16-bit value
            combined_value = (ranging_data[i + 1] << 8) | ranging_data[i]
            distance = combined_value & 0x01FF  # Get the lower 9 bits
            # Extract the intensity
            intensity = (combined_value >> 9) & 0x007F

            dist_list.append(distance)
            intens_list.append(intensity)

        return dist_list, intens_list

    def get_scan_data(self):
        """
        Retrieves the scan data from the Lidar sensor.

        Returns:
            distance_points (list): A list of distances from the Lidar sensor to each scan point.
            theta_points (list): A list of angles in degrees representing the orientation of each scan point.
        """
        # call the get basic scan data function
        dist_list, intens_list = self.get_basic_scan_data()
        # TODO: extend the data to include the co-ordinates and angles for the scan points

        distance_points = []
        theta_points = []
        # for each distance point in the scan
        for i in range(0, 160):
            Dist = 0
            theta = 0
            # Ref Source: YDLIDAR GS2 Development Manual V1.6.3 (Section 4)
            # is point Left? (i < 80)
            if i < 80:
                if self.compensated_b0 > 1:
                    tempTheta = self.compensated_k0 * (80 - i) - self.compensated_b0
                else:
                    tempTheta = (
                        math.atan(
                            self.compensated_k0 * (80 - i) - self.compensated_b0
                        ).real
                        * 180
                        / math.pi
                    )

                # NOTE: make sure to get the real value of the trig function complex number result

                tempDist = (dist_list[i] - self.angle_p_x) / math.cos(
                    (self.angle_p_angle + self.bias - (tempTheta)) * math.pi / 180
                ).real

                # tempTheta = tempTheta * math.pi / 180

                tempX = math.cos(
                    (self.angle_p_angle + self.bias) * math.pi / 180
                ).real * tempDist * math.cos(tempTheta).real + math.sin(
                    (self.angle_p_angle + self.bias) * math.pi / 180
                ).real * (
                    tempDist * math.sin(tempTheta).real
                )

                tempY = -math.sin(
                    (self.angle_p_angle + self.bias) * math.pi / 180
                ).real * tempDist * math.cos(tempTheta).real + math.cos(
                    (self.angle_p_angle + self.bias) * math.pi / 180
                ).real * (
                    tempDist * math.sin(tempTheta).real
                )

                tempX = tempX + self.angle_p_x
                tempY = tempY - self.angle_p_y

                # prevent division by zero crash
                if tempX != 0.0:
                    Dist = math.sqrt(tempX * tempX + tempY * tempY).real
                    theta = math.atan(tempY / tempX).real * 180 / math.pi
                else:
                    Dist = 0
                    theta = 0
            else:
                # point is right
                if self.compensated_b1 > 1:
                    tempTheta = self.compensated_k1 * (160 - i) - self.compensated_b1
                else:
                    tempTheta = (
                        math.atan(
                            self.compensated_k1 * (160 - i) - self.compensated_b1
                        ).real
                        * 180
                        / math.pi
                    )

                tempDist = (dist_list[i] - self.angle_p_x) / math.cos(
                    (self.angle_p_angle + self.bias - (tempTheta)) * math.pi / 180
                ).real

                # tempTheta = tempTheta * math.pi / 180

                tempX = math.cos(
                    -(self.angle_p_angle + self.bias) * math.pi / 180
                ).real * tempDist * math.cos(tempTheta).real + math.sin(
                    -(self.angle_p_angle + self.bias) * math.pi / 180
                ).real * (
                    tempDist * math.sin(tempTheta).real
                )

                tempY = -math.sin(
                    -(self.angle_p_angle + self.bias) * math.pi / 180
                ).real * tempDist * math.cos(tempTheta).real + math.cos(
                    -(self.angle_p_angle + self.bias) * math.pi / 180
                ).real * (
                    tempDist * math.sin(tempTheta).real
                )

                tempX = tempX + self.angle_p_x
                tempY = tempY + self.angle_p_y

                Dist = math.sqrt(tempX * tempX + tempY * tempY).real
                theta = math.atan(tempY / tempX).real * 180 / math.pi
            if theta < 0:
                theta = theta + 360
            distance_points.append(Dist)
            theta_points.append(theta)

        return distance_points, theta_points

    def stop_scan(self):
        """
        Stops the lidar scan.

        :return: None
        """
        response = self.send_command(
            device_address=b"\x00",
            packet_type=GS2_Lidar.GS2_STOP_SCAN,
            data_length=b"\x00\x00",
            data_segment=b"",
            check_code=b"\x64",
        )

    # TODO: Implement set lidar baudrate
    # TODO: Implement set lidar edge mode
    # TODO: Implement soft reset

    # function to print the lidar parameters (with parameter headers)
    def print_lidar_parameters(self):
        """
        Print the LIDAR parameters including device address, version number, serial number,
        compensated K0, compensated K1, compensated B0, compensated B1, and bias.

        """
        print("Device Address: ", self.device_address)
        print("Version Number: ", self.version_number)
        print("Serial Number: ", self.serial_number)
        print("Compensated K0: ", self.compensated_k0)
        print("Compensated K1: ", self.compensated_k1)
        print("Compensated B0: ", self.compensated_b0)
        print("Compensated B1: ", self.compensated_b1)
        print("Bias: ", self.bias)
