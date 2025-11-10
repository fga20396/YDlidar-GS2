# File: GS2_Lidar_Viewer.py
# Author: Robert Stevenson
# Date: 08/09/2023 (DD/MM/YYYY)
# Description:
# Lidar Data Visualizer

from GSlidar import *
import cmath as math

# Main Entry point of the script
if __name__ == "__main__":
    # Open the serial port connection
    lidar = GS2lidar(serial_port="/dev/ttyUSB0")

    # start the lidar
    try:
        lidar.start_scan()

        # get the scan data and print it constantly
        while True:
            distances, thetas = lidar.get_scan_data()
            x = []
            y = []

            for i in range(len(distances)):
                x.append(distances[i]*math.sin(thetas[i]).real)
                y.append(distances[i]*math.cos(thetas[i]).real)
            

    except KeyboardInterrupt:
        plt.close()
    finally:
        lidar.stop_scan()
