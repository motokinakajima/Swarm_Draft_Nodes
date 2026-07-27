import tkinter as tk
from tkinter import filedialog
import cv2
import numpy as np
from PIL import Image, ImageTk
import json
from tkinter import ttk
import time
import threading
from queue import Queue, Empty

class HSVContourApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Real-time HSV Contour & Coordinate Detector")
        self.root.geometry("1200x800")
        self.root.minsize(1000, 600)
        
        # macOS or Linux Camera Backend
        self.cap = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION) # Change to CAP_V4L2 for Linux
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        self.image = None
        self.running = True
        self.resize_width = 320
        self.resize_height = 240
        self.frame_skip_counter = 0
        
        # --- Camera & Robot Parameters ---
        # 実際のカメラ環境に合わせて調整してください
        self.fov_rad = np.radians(90.0)
        self.heading_rad = np.radians(0.0)
        self.marker_R = 0.1016  # 4 inches in meters
        self.cam_pos_x = 0.0
        self.cam_pos_y = 0.0

        # Threading for camera capture
        self.frame_queue = Queue(maxsize=2)
        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.capture_thread.daemon = True
        self.capture_thread.start()

        self.create_gui()
        self.update_frame()
    
    def create_gui(self):
        main_frame = tk.Frame(self.root)
        main_frame.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)
        
        control_frame = tk.Frame(main_frame)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        display_frame = tk.Frame(main_frame)
        display_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Buttons
        button_frame = tk.Frame(control_frame)
        button_frame.pack(fill=tk.X, pady=(0, 10))
        tk.Button(button_frame, text="Save Config", command=self.save_config).pack(side=tk.LEFT, padx=2)
        tk.Button(button_frame, text="Load Config", command=self.load_config).pack(side=tk.LEFT, padx=2)
        
        # Resolution
        res_frame = tk.Frame(control_frame)
        res_frame.pack(fill=tk.X, pady=(0, 10))
        tk.Label(res_frame, text="Resolution:").pack(side=tk.LEFT, padx=(0, 5))
        self.resolution_combobox = ttk.Combobox(
            res_frame,
            values=["160x120", "320x240", "640x480", "1280x720"],
            state="readonly", width=10
        )
        self.resolution_combobox.set("320x240")
        self.resolution_combobox.pack(side=tk.LEFT)
        self.resolution_combobox.bind("<<ComboboxSelected>>", self.update_resolution)
        
        # Status labels
        status_frame = tk.Frame(control_frame)
        status_frame.pack(fill=tk.X, pady=(0, 10))
        self.count_label = tk.Label(status_frame, text="Target found: False", font=("Arial", 10, "bold"))
        self.count_label.pack()
        self.lag_label = tk.Label(status_frame, text="Processing lag: 0ms", font=("Arial", 9))
        self.lag_label.pack()

        # Sliders
        sliders_frame = tk.Frame(control_frame)
        sliders_frame.pack(fill=tk.X, pady=5)
        
        self.sliders = {}
        labels = [
            ("Hue Min", 0, 179, 0), ("Hue Max", 0, 179, 179),
            ("Sat Min", 0, 255, 50), ("Sat Max", 0, 255, 255),
            ("Val Min", 0, 255, 50), ("Val Max", 0, 255, 255),
            ("Min Area", 1, 1000, 20), ("Max Area", 10, 50000, 1000),
            ("Skip Frames", 0, 5, 0), ("Enable Morph", 0, 1, 1),
        ]
        
        for name, min_val, max_val, default in labels:
            slider_row = tk.Frame(sliders_frame)
            slider_row.pack(fill=tk.X, pady=3)
            tk.Label(slider_row, text=name, width=12, anchor="w").pack(side=tk.LEFT, padx=(0, 5))
            self.sliders[name] = tk.Scale(slider_row, from_=min_val, to=max_val, orient=tk.HORIZONTAL, length=250)
            self.sliders[name].set(default)
            self.sliders[name].pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Top Display: Original & Mask
        top_display = tk.Frame(display_frame)
        top_display.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        original_frame = tk.Frame(top_display)
        original_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        tk.Label(original_frame, text="Original", font=("Arial", 10, "bold")).pack()
        self.original_canvas = tk.Canvas(original_frame, width=350, height=250, bg="gray90")
        self.original_canvas.pack(fill=tk.BOTH, expand=True)
        
        mask_frame = tk.Frame(top_display)
        mask_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        tk.Label(mask_frame, text="HSV Mask", font=("Arial", 10, "bold")).pack()
        self.mask_canvas = tk.Canvas(mask_frame, width=350, height=250, bg="gray90")
        self.mask_canvas.pack(fill=tk.BOTH, expand=True)
        
        # Bottom Display: Camera Result & 2D Plot
        bottom_display = tk.Frame(display_frame)
        bottom_display.pack(fill=tk.BOTH, expand=True)
        
        result_frame = tk.Frame(bottom_display)
        result_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        tk.Label(result_frame, text="Camera Result (Last Only)", font=("Arial", 10, "bold")).pack()
        self.result_canvas = tk.Canvas(result_frame, width=350, height=250, bg="gray90")
        self.result_canvas.pack(fill=tk.BOTH, expand=True)
        
        plot_frame = tk.Frame(bottom_display)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        tk.Label(plot_frame, text="Robot Coords Plot (X, Y)", font=("Arial", 10, "bold")).pack()
        self.plot_canvas = tk.Canvas(plot_frame, width=350, height=250, bg="white")
        self.plot_canvas.pack(fill=tk.BOTH, expand=True)

    def capture_frames(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                if not self.frame_queue.full():
                    self.frame_queue.put(frame)
                else:
                    try:
                        self.frame_queue.get_nowait()
                        self.frame_queue.put(frame)
                    except Empty:
                        pass
            time.sleep(0.001)

    def update_resolution(self, event=None):
        text = self.resolution_combobox.get()
        try:
            w, h = map(int, text.lower().split("x"))
            self.resize_width, self.resize_height = w, h
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        except:
            pass

    def save_config(self):
        config = {name: slider.get() for name, slider in self.sliders.items()}
        file_path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON files", "*.json")])
        if file_path:
            with open(file_path, 'w') as f:
                json.dump(config, f, indent=4)

    def load_config(self):
        file_path = filedialog.askopenfilename(filetypes=[("JSON files", "*.json")])
        if file_path:
            with open(file_path, 'r') as f:
                config = json.load(f)
            if config:
                for name, value in config.items():
                    if name in self.sliders:
                        self.sliders[name].set(value)

    def detection_to_robot_coords(self, x_cam, y_cam, marker_R_cam):
        # Snippet 2: Coordinate math
        phi_rad = self.fov_rad * (x_cam / self.resize_width - 0.5)
        d = (self.marker_R * self.resize_width) / (marker_R_cam * self.fov_rad)
        total_heading_rad = phi_rad + self.heading_rad
        x_robot = d * np.cos(total_heading_rad) + self.cam_pos_x
        y_robot = d * np.sin(total_heading_rad) + self.cam_pos_y
        return x_robot, y_robot

    def update_frame(self):
        if not self.running:
            return

        total_start = time.time()
        try:
            frame = self.frame_queue.get_nowait()
            ret = True
        except Empty:
            ret = False
        
        if ret:
            skip_frames = int(self.sliders["Skip Frames"].get())
            if skip_frames > 0 and self.frame_skip_counter % (skip_frames + 1) != 0:
                self.frame_skip_counter += 1
                self.root.after(16, self.update_frame)
                return
            
            self.frame_skip_counter += 1
            if frame.shape[1] != self.resize_width or frame.shape[0] != self.resize_height:
                frame = cv2.resize(frame, (self.resize_width, self.resize_height))
            self.image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            self.update_image()
            total_time = (time.time() - total_start) * 1000
            self.lag_label.config(text=f"Total Proc Time: {total_time:.1f}ms")

        self.root.after(30, self.update_frame)

    def update_image(self):
        if self.image is None:
            return

        # 1. Masking
        hsv = cv2.cvtColor(self.image, cv2.COLOR_RGB2HSV)
        hmin, hmax = self.sliders["Hue Min"].get(), self.sliders["Hue Max"].get()
        smin, smax = self.sliders["Sat Min"].get(), self.sliders["Sat Max"].get()
        vmin, vmax = self.sliders["Val Min"].get(), self.sliders["Val Max"].get()
        amin, amax = self.sliders["Min Area"].get(), self.sliders["Max Area"].get()

        if hmin <= hmax:
            mask = cv2.inRange(hsv, np.array([hmin, smin, vmin]), np.array([hmax, smax, vmax]))
        else:
            mask1 = cv2.inRange(hsv, np.array([hmin, smin, vmin]), np.array([179, smax, vmax]))
            mask2 = cv2.inRange(hsv, np.array([0, smin, vmin]), np.array([hmax, smax, vmax]))
            mask = cv2.bitwise_or(mask1, mask2)

        # 2. Morphological Operations
        if self.sliders["Enable Morph"].get():
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 3. Contour Detection & Processing
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        output = self.image.copy()
        
        valid_detections = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if amin <= area <= amax:
                ((x, y), radius) = cv2.minEnclosingCircle(cnt)
                if radius > 5:
                    valid_detections.append((cnt, x, y, radius))

        target_found = False
        
        # "最後の一個だけでいい" という条件の適用
        if valid_detections:
            target_found = True
            last_det = valid_detections[-1]
            cnt, x_cam, y_cam, r_cam = last_det
            
            # Draw on camera image
            cv2.drawContours(output, [cnt], -1, (255, 0, 0), 2)
            cv2.circle(output, (int(x_cam), int(y_cam)), int(r_cam), (0, 255, 0), 2)
            cv2.circle(output, (int(x_cam), int(y_cam)), 5, (0, 0, 255), -1)
            
            # Math
            x_rob, y_rob = self.detection_to_robot_coords(x_cam, y_cam, r_cam)
            
            # Plot
            self.plot_coordinates(x_rob, y_rob)
        else:
            # Nothing detected
            self.plot_coordinates(None, None)

        self.count_label.config(text=f"Target found: {target_found}")
        
        # Update Displays
        self.display_image(self.image, self.original_canvas)
        self.display_image(cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB), self.mask_canvas)
        self.display_image(output, self.result_canvas)

    def plot_coordinates(self, x, y):
        """Draws a 2D top-down grid and plots the target X, Y relative to the robot"""
        self.plot_canvas.delete("all")
        
        # Set drawing area center
        w = self.plot_canvas.winfo_width()
        h = self.plot_canvas.winfo_height()
        if w <= 1: w = 350
        if h <= 1: h = 250
        cx, cy = w // 2, h // 2
        
        # Draw Axis
        self.plot_canvas.create_line(cx, 0, cx, h, fill="gray80", dash=(4, 4))
        self.plot_canvas.create_line(0, cy, w, cy, fill="gray80", dash=(4, 4))
        
        # Draw Robot (Center)
        self.plot_canvas.create_oval(cx-6, cy-6, cx+6, cy+6, fill="blue")
        self.plot_canvas.create_text(cx+10, cy+10, text="Robot", fill="blue", anchor="nw")
        
        if x is not None and y is not None:
            # Scale factor: Pixels per meter (Adjust if your target is very far)
            # Default 50px = 1 meter
            scale_px_per_m = 50 
            
            # ROS conventions: X is forward (Up on screen), Y is right (Right on screen)
            plot_x = cx + (y * scale_px_per_m)
            plot_y = cy - (x * scale_px_per_m)
            
            # Draw Target point
            self.plot_canvas.create_oval(plot_x-5, plot_y-5, plot_x+5, plot_y+5, fill="red")
            self.plot_canvas.create_text(
                10, 10, anchor="nw", 
                text=f"Target Pos:\n X: {x:.2f} m (Forward)\n Y: {y:.2f} m (Left)", 
                font=("Arial", 11, "bold"), fill="red"
            )

    def display_image(self, img, canvas):
        h, w = img.shape[:2]
        canvas_w = canvas.winfo_width()
        canvas_h = canvas.winfo_height()
        if canvas_w <= 1: canvas_w = 350
        if canvas_h <= 1: canvas_h = 250
        
        scale = min(canvas_w / w, canvas_h / h)
        new_size = (int(w * scale), int(h * scale))
        resized = cv2.resize(img, new_size)
        img_pil = Image.fromarray(resized)
        img_tk = ImageTk.PhotoImage(img_pil)
        
        canvas.delete("all")
        canvas.create_image(canvas_w//2, canvas_h//2, image=img_tk, anchor="center")
        canvas.image = img_tk

    def on_closing(self):
        self.running = False
        self.cap.release()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = HSVContourApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()