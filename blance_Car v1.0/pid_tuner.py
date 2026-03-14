# -*- coding: utf-8 -*-
"""
平衡小车 PID 自动调参工具
功能：
1. 实时读取小车数据（角度、角速度、PWM等）
2. 可视化显示数据曲线
3. 自动/手动调整PID参数
4. 参数优化算法
"""

import serial
import serial.tools.list_ports
import threading
import time
import re
import tkinter as tk
from tkinter import ttk, messagebox
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

class BalanceCarTuner:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("平衡小车 PID 调参工具 v1.0")
        self.root.geometry("1200x800")
        self.root.configure(bg='#2b2b2b')
        
        self.serial_port = None
        self.is_connected = False
        self.is_running = False
        self.debug_enabled = False
        
        self.angle_data = deque(maxlen=200)
        self.gyro_data = deque(maxlen=200)
        self.pwm_data = deque(maxlen=200)
        self.time_data = deque(maxlen=200)
        self.start_time = time.time()
        
        self.current_params = {
            'upright_Kp': 0,
            'upright_Kd': 0,
            'speed_Kp': 0,
            'speed_Ki': 0,
            'balance_angle': 0
        }
        
        self.setup_ui()
        self.update_plot()
        
    def setup_ui(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TFrame', background='#2b2b2b')
        style.configure('TLabel', background='#2b2b2b', foreground='white', font=('Microsoft YaHei', 10))
        style.configure('TButton', font=('Microsoft YaHei', 10))
        style.configure('Header.TLabel', font=('Microsoft YaHei', 12, 'bold'))
        
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        left_frame = ttk.Frame(main_frame, width=350)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        left_frame.pack_propagate(False)
        
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        self.setup_connection_panel(left_frame)
        self.setup_pid_panel(left_frame)
        self.setup_status_panel(left_frame)
        self.setup_plot_panel(right_frame)
        self.setup_auto_tune_panel(left_frame)
        
    def setup_connection_panel(self, parent):
        conn_frame = ttk.LabelFrame(parent, text="串口连接", padding=10)
        conn_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(conn_frame, text="串口:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.port_combo = ttk.Combobox(conn_frame, width=15)
        self.port_combo.grid(row=0, column=1, pady=5, padx=5)
        
        self.refresh_btn = ttk.Button(conn_frame, text="刷新", command=self.refresh_ports)
        self.refresh_btn.grid(row=0, column=2, pady=5)
        
        ttk.Label(conn_frame, text="波特率:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.baud_combo = ttk.Combobox(conn_frame, width=15, values=['9600', '115200', '460800'])
        self.baud_combo.set('115200')
        self.baud_combo.grid(row=1, column=1, pady=5, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=2, column=0, columnspan=3, pady=10, sticky=tk.EW)
        
        self.debug_btn = ttk.Button(conn_frame, text="开启调试模式", command=self.toggle_debug)
        self.debug_btn.grid(row=3, column=0, columnspan=3, pady=5, sticky=tk.EW)
        
        self.refresh_ports()
        
    def setup_pid_panel(self, parent):
        pid_frame = ttk.LabelFrame(parent, text="PID 参数调整", padding=10)
        pid_frame.pack(fill=tk.X, pady=(0, 10))
        
        params = [
            ("直立环 Kp:", "upright_Kp", 0, 1000, 1),
            ("直立环 Kd:", "upright_Kd", 0, 2, 0.01),
            ("速度环 Kp:", "speed_Kp", 0, 1, 0.01),
            ("速度环 Ki:", "speed_Ki", 0, 0.01, 0.0001),
            ("机械零点:", "balance_angle", -5, 5, 0.1),
        ]
        
        self.pid_vars = {}
        self.pid_entries = {}
        self.pid_scales = {}
        
        for i, (label, name, min_val, max_val, resolution) in enumerate(params):
            ttk.Label(pid_frame, text=label).grid(row=i, column=0, sticky=tk.W, pady=3)
            
            var = tk.DoubleVar(value=0)
            self.pid_vars[name] = var
            
            entry = ttk.Entry(pid_frame, textvariable=var, width=8)
            entry.grid(row=i, column=1, pady=3, padx=5)
            self.pid_entries[name] = entry
            
            scale = ttk.Scale(pid_frame, from_=min_val, to=max_val, variable=var, 
                            orient=tk.HORIZONTAL, length=120,
                            command=lambda v, n=name: self.on_param_change(n))
            scale.grid(row=i, column=2, pady=3, padx=5)
            self.pid_scales[name] = scale
            
            send_btn = ttk.Button(pid_frame, text="发送", width=6,
                                 command=lambda n=name: self.send_param(n))
            send_btn.grid(row=i, column=3, pady=3, padx=2)
            
        ttk.Button(pid_frame, text="发送全部参数", 
                  command=self.send_all_params).grid(row=len(params), column=0, 
                                                     columnspan=4, pady=10, sticky=tk.EW)
        
    def setup_status_panel(self, parent):
        status_frame = ttk.LabelFrame(parent, text="实时数据", padding=10)
        status_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.status_labels = {}
        status_items = [
            ("当前角度:", "angle"),
            ("目标角度:", "target_angle"),
            ("角速度:", "gyro"),
            ("直立输出:", "upright_out"),
            ("速度输出:", "speed_out"),
            ("左轮速度:", "speed_l"),
            ("右轮速度:", "speed_r"),
        ]
        
        for i, (label, key) in enumerate(status_items):
            ttk.Label(status_frame, text=label).grid(row=i, column=0, sticky=tk.W, pady=2)
            value_label = ttk.Label(status_frame, text="--", width=10)
            value_label.grid(row=i, column=1, sticky=tk.W, pady=2, padx=10)
            self.status_labels[key] = value_label
            
    def setup_auto_tune_panel(self, parent):
        auto_frame = ttk.LabelFrame(parent, text="自动调参", padding=10)
        auto_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(auto_frame, text="调参算法:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.algo_combo = ttk.Combobox(auto_frame, width=15, 
                                       values=['Ziegler-Nichols', '试凑法', '遗传算法'])
        self.algo_combo.set('试凑法')
        self.algo_combo.grid(row=0, column=1, pady=5, padx=5)
        
        ttk.Label(auto_frame, text="目标环:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.loop_combo = ttk.Combobox(auto_frame, width=15, 
                                       values=['直立环', '速度环', '循迹环'])
        self.loop_combo.set('直立环')
        self.loop_combo.grid(row=1, column=1, pady=5, padx=5)
        
        self.auto_tune_btn = ttk.Button(auto_frame, text="开始自动调参", 
                                        command=self.start_auto_tune)
        self.auto_tune_btn.grid(row=2, column=0, columnspan=2, pady=10, sticky=tk.EW)
        
        self.stop_tune_btn = ttk.Button(auto_frame, text="停止", 
                                        command=self.stop_auto_tune, state=tk.DISABLED)
        self.stop_tune_btn.grid(row=3, column=0, columnspan=2, pady=5, sticky=tk.EW)
        
    def setup_plot_panel(self, parent):
        plot_frame = ttk.LabelFrame(parent, text="数据曲线", padding=10)
        plot_frame.pack(fill=tk.BOTH, expand=True)
        
        self.fig = Figure(figsize=(8, 6), dpi=100, facecolor='#2b2b2b')
        self.ax1 = self.fig.add_subplot(311)
        self.ax2 = self.fig.add_subplot(312)
        self.ax3 = self.fig.add_subplot(313)
        
        for ax in [self.ax1, self.ax2, self.ax3]:
            ax.set_facecolor('#1e1e1e')
            ax.tick_params(colors='white')
            ax.spines['bottom'].set_color('white')
            ax.spines['top'].set_color('white')
            ax.spines['left'].set_color('white')
            ax.spines['right'].set_color('white')
            ax.xaxis.label.set_color('white')
            ax.yaxis.label.set_color('white')
            ax.title.set_color('white')
        
        self.ax1.set_ylabel('角度 (°)')
        self.ax1.set_title('姿态角度')
        self.ax2.set_ylabel('角速度 (°/s)')
        self.ax2.set_title('角速度')
        self.ax3.set_ylabel('PWM')
        self.ax3.set_title('电机输出')
        self.ax3.set_xlabel('时间 (s)')
        
        self.fig.tight_layout()
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.set(port_list[0])
            
    def toggle_connection(self):
        if self.is_connected:
            self.disconnect()
        else:
            self.connect()
            
    def connect(self):
        try:
            port = self.port_combo.get()
            baud = int(self.baud_combo.get())
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
            self.is_connected = True
            self.is_running = True
            self.connect_btn.config(text="断开")
            
            self.recv_thread = threading.Thread(target=self.receive_data, daemon=True)
            self.recv_thread.start()
            
            self.start_time = time.time()
            self.angle_data.clear()
            self.gyro_data.clear()
            self.pwm_data.clear()
            self.time_data.clear()
            
        except Exception as e:
            messagebox.showerror("错误", f"连接失败: {str(e)}")
            
    def disconnect(self):
        self.is_running = False
        self.is_connected = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.connect_btn.config(text="连接")
        
    def toggle_debug(self):
        if not self.is_connected:
            messagebox.showwarning("警告", "请先连接串口")
            return
            
        self.debug_enabled = not self.debug_enabled
        cmd = f"[Debug,En,{1 if self.debug_enabled else 0}]\r\n"
        self.serial_port.write(cmd.encode())
        
        if self.debug_enabled:
            self.debug_btn.config(text="关闭调试模式")
        else:
            self.debug_btn.config(text="开启调试模式")
            
    def receive_data(self):
        buffer = ""
        while self.is_running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        self.parse_data(line.strip())
                        
            except Exception as e:
                pass
            time.sleep(0.001)
            
    def parse_data(self, line):
        if line.startswith('[DBG,'):
            try:
                match = re.match(r'\[DBG,([\d.\-]+),([\d.\-]+),([\d.\-]+),([\d.\-]+),([\d.\-]+),([\d.\-]+),([\d.\-]+),([\d.\-]+),([\d.\-]+),([\d.\-]+)\]', line)
                if match:
                    angle = float(match.group(1))
                    target_angle = float(match.group(2))
                    gyro_x = float(match.group(3))
                    gyro_z = float(match.group(4))
                    upright_out = float(match.group(5))
                    speed_out = float(match.group(6))
                    speed_l = float(match.group(7))
                    speed_r = float(match.group(8))
                    kp = float(match.group(9))
                    kd = float(match.group(10))
                    
                    current_time = time.time() - self.start_time
                    self.time_data.append(current_time)
                    self.angle_data.append(angle)
                    self.gyro_data.append(gyro_x)
                    self.pwm_data.append(upright_out)
                    
                    self.root.after(0, lambda: self.update_status(
                        angle, target_angle, gyro_x, upright_out, speed_out, speed_l, speed_r
                    ))
                    
                    self.current_params['upright_Kp'] = kp
                    self.current_params['upright_Kd'] = kd
                    
            except Exception as e:
                pass
                
    def update_status(self, angle, target_angle, gyro, upright_out, speed_out, speed_l, speed_r):
        self.status_labels['angle'].config(text=f"{angle:.2f}°")
        self.status_labels['target_angle'].config(text=f"{target_angle:.2f}°")
        self.status_labels['gyro'].config(text=f"{gyro:.1f}°/s")
        self.status_labels['upright_out'].config(text=f"{upright_out:.1f}")
        self.status_labels['speed_out'].config(text=f"{speed_out:.1f}")
        self.status_labels['speed_l'].config(text=f"{speed_l:.1f}")
        self.status_labels['speed_r'].config(text=f"{speed_r:.1f}")
        
    def update_plot(self):
        if len(self.time_data) > 1:
            time_list = list(self.time_data)
            
            self.ax1.clear()
            self.ax1.plot(time_list, list(self.angle_data), 'c-', linewidth=1, label='角度')
            self.ax1.set_ylabel('角度 (°)')
            self.ax1.set_title('姿态角度')
            self.ax1.grid(True, alpha=0.3)
            
            self.ax2.clear()
            self.ax2.plot(time_list, list(self.gyro_data), 'g-', linewidth=1, label='角速度')
            self.ax2.set_ylabel('角速度 (°/s)')
            self.ax2.set_title('角速度')
            self.ax2.grid(True, alpha=0.3)
            
            self.ax3.clear()
            self.ax3.plot(time_list, list(self.pwm_data), 'r-', linewidth=1, label='PWM')
            self.ax3.set_ylabel('PWM')
            self.ax3.set_title('电机输出')
            self.ax3.set_xlabel('时间 (s)')
            self.ax3.grid(True, alpha=0.3)
            
            self.fig.tight_layout()
            self.canvas.draw()
            
        self.root.after(100, self.update_plot)
        
    def on_param_change(self, name):
        pass
        
    def send_param(self, name):
        if not self.is_connected:
            messagebox.showwarning("警告", "请先连接串口")
            return
            
        value = self.pid_vars[name].get()
        
        cmd_map = {
            'upright_Kp': f'[upright,Kp,{value}]',
            'upright_Kd': f'[upright,Kd,{value}]',
            'speed_Kp': f'[Spd,Kp,{value}]',
            'speed_Ki': f'[Spd,Ki,{value}]',
            'balance_angle': f'[Zero,Z,{value}]',
        }
        
        if name in cmd_map:
            cmd = cmd_map[name] + '\r\n'
            self.serial_port.write(cmd.encode())
            print(f"发送: {cmd.strip()}")
            
    def send_all_params(self):
        if not self.is_connected:
            messagebox.showwarning("警告", "请先连接串口")
            return
            
        for name in self.pid_vars:
            self.send_param(name)
            time.sleep(0.05)
            
    def start_auto_tune(self):
        if not self.is_connected:
            messagebox.showwarning("警告", "请先连接串口")
            return
            
        if not self.debug_enabled:
            messagebox.showwarning("警告", "请先开启调试模式")
            return
            
        self.auto_tune_btn.config(state=tk.DISABLED)
        self.stop_tune_btn.config(state=tk.NORMAL)
        
        self.auto_tune_thread = threading.Thread(target=self.auto_tune_process, daemon=True)
        self.auto_tune_thread.start()
        
    def stop_auto_tune(self):
        self.auto_tune_btn.config(state=tk.NORMAL)
        self.stop_tune_btn.config(state=tk.DISABLED)
        
    def auto_tune_process(self):
        algo = self.algo_combo.get()
        loop = self.loop_combo.get()
        
        if loop == '直立环':
            self.tune_upright_loop()
        elif loop == '速度环':
            self.tune_speed_loop()
            
        self.root.after(0, self.stop_auto_tune)
        
    def tune_upright_loop(self):
        kp = 200
        kd = 0.1
        
        self.pid_vars['upright_Kp'].set(kp)
        self.send_param('upright_Kp')
        time.sleep(0.5)
        
        for i in range(20):
            kp += 20
            self.pid_vars['upright_Kp'].set(kp)
            self.send_param('upright_Kp')
            
            time.sleep(2)
            
            if len(self.angle_data) > 50:
                angle_std = np.std(list(self.angle_data)[-50:])
                if angle_std < 2.0:
                    print(f"找到合适Kp: {kp}")
                    break
                    
        for i in range(20):
            kd += 0.02
            self.pid_vars['upright_Kd'].set(kd)
            self.send_param('upright_Kd')
            
            time.sleep(2)
            
            if len(self.pwm_data) > 50:
                pwm_std = np.std(list(self.pwm_data)[-50:])
                if pwm_std < 500:
                    print(f"找到合适Kd: {kd}")
                    break
                    
    def tune_speed_loop(self):
        kp = 0.1
        ki = 0.0001
        
        self.pid_vars['speed_Kp'].set(kp)
        self.send_param('speed_Kp')
        
        for i in range(15):
            kp += 0.05
            self.pid_vars['speed_Kp'].set(kp)
            self.send_param('speed_Kp')
            time.sleep(3)
            
    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    app = BalanceCarTuner()
    app.run()
