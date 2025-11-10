import tkinter as tk
import math
import threading
from GS2_Lidar import GS2_Lidar  # Ensure this matches the actual filename and class name

# Configuration
PORT = '/dev/ttyUSB0'  # Adjust as needed
BAUDRATE = 921600

class LidarApp:
    def __init__(self, root):
        self.root = root
        self.root.title("YDLidar GS2 Viewer")
        self.canvas_size = 600
        self.canvas = tk.Canvas(root, width=self.canvas_size, height=self.canvas_size, bg='black')
        self.canvas.pack()
        self.center = self.canvas_size // 2
        self.lidar = GS2_Lidar(PORT, BAUDRATE)
        self.running = True
        self.update_interval = 50  # milliseconds

        threading.Thread(target=self.lidar_loop, daemon=True).start()
        self.root.after(self.update_interval, self.update_canvas)

    def polar_to_canvas(self, angle, distance):
        # Convert polar coordinates to canvas coordinates
        rad = math.radians(angle)
        x = self.center + distance * math.cos(rad)
        y = self.center - distance * math.sin(rad)
        return x, y

    def update_canvas(self):
        self.canvas.delete("all")
        for angle, distance in self.lidar.scan_data:
            if distance > 0:
                x, y = self.polar_to_canvas(angle, distance)
                self.canvas.create_oval(x-2, y-2, x+2, y+2, fill='lime', outline='')
        if self.running:
            self.root.after(self.update_interval, self.update_canvas)

    def lidar_loop(self):
        self.lidar.start()
        while self.running:
            self.lidar.update()

    def stop(self):
        self.running = False
        self.lidar.stop()

if __name__ == "__main__":
    root = tk.Tk()
    app = LidarApp(root)
    try:
        root.mainloop()
    finally:
        app.stop()
