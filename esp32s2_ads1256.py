import os
import serial
import time
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button
import threading
import csv
import datetime

from statistics import mean, pvariance, pstdev
import math

import json
import pathlib
import cv2

SAVE_DIR = r"C:\Users\rriis\Desktop\新しいフォルダー\SMA_FT232-main\ADS1256\esp32_ads1256_data"
os.makedirs(SAVE_DIR, exist_ok=True)

def crc8(buf: bytes) -> int:
    crc = 0
    for byte in buf:
        in_byte = byte
        for _ in range(8):
            mix = (crc ^ in_byte) & 0x01
            crc >>= 1
            if mix:
                crc ^= 0x8C
            in_byte >>= 1
    return crc

class ADS1256DataAcquisition:
    def __init__(self, port="COM10", baudrate=2000000, plot_enabled=True, *args, **kwargs):
        try:
            self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            raise

        self.data = {i: [] for i in range(8)}  # 存储8个通道的数据
        self.time_data = {i: [] for i in range(8)}
        self.start_time = None
        self.mode = None
        self.channel = None
        self.VREF = 2.5
        self.plot_enabled = plot_enabled

        if self.plot_enabled:
            # 使用交互模式创建图形及坐标轴
            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(12, 7))
            self.ax.set_title("ADS1256 Data Visualization")
            self.ax.set_xlabel("Time (Samples)")
            self.ax.set_ylabel("Voltage (V)")
        else:
            self.fig = None
            self.ax = None

        self.stop_flag = False
        self.N = 10000
        self.median_window = 5
          # 显示的点数
        self.margin = 0.00005

        # 为每个通道创建 line 对象，后续通过 set_data 更新数据
        self.line_objects = {}
        self.update_job = None
        if self.plot_enabled:
            for i in range(8):
                line, = self.ax.plot([], [], label=f'Channel {i}')
                self.line_objects[i] = line

            # 添加停止按钮
            ax_stop = plt.axes([0.8, 0.05, 0.1, 0.075])
            self.btn_stop = Button(ax_stop, 'Stop')
            self.btn_stop.on_clicked(self.stop)

        self.start_time = None
        self.sample_count = 0

        self.outlier_std_threshold = 3

    def send_command(self, command):
        """发送命令到Arduino"""
        try:
            self.arduino.write((command + '\n').encode())
            time.sleep(0.1)
        except Exception as e:
            print(f"Error sending command '{command}': {e}")

    def median0(self):
        # 清空缓冲
        self.arduino.reset_input_buffer()

        while not self.stop_flag:
            chan, raw = self.read_frame()
            if chan is None:
                continue
            if chan != 0:
                continue
            voltage = (raw * 2 * self.VREF) / (8388607.0 )
            return voltage

        return None
    # def median0(self):
    #     """
    #     读取 DIFF0 通道的中位数 ADC 值（由 Arduino 端计算好），
    #     并转换为电压后返回。
    #     """
    #     self.arduino.reset_input_buffer()
    #     # start = time.time()
    #     # while not self.stop_flag and self.arduino.in_waiting == 0:
    #     #     time.sleep(0.0005)
    #     #     if time.time() - start > 0.02:
    #     #         return None
    #     while not self.stop_flag:
    #         try:
    #             # 读一行串口
    #             line = self.arduino.readline().decode('utf-8', errors='ignore').strip()
    #             if not line:
    #                 continue
    #             parts = line.split(',')
    #             # 期待格式：MEDIAN,0,<adc_value>
    #             if len(parts) == 3 and parts[0] == "MEDIAN" and parts[1] == "0":
    #                 adc_value = int(parts[2])
    #                 # 电压转换：fullScale = 2*VREF / PGA
    #                 voltage = (adc_value * 2 * self.VREF) / (8388607.0 * 64)
    #                 return voltage
    #             # 如果不是目标行，就继续循环读取
    #         except Exception as e:
    #             print(f"串口读取错误: {e}")
    #             return None

    #     # 如果 stop_flag 被置位，直接返回
    #     return None

    def read_data(self):
        """后台线程中持续从Arduino读取数据（仅更新数据，不调用GUI更新函数）"""
        self.sample_count = 0
        while not self.stop_flag:
            try:
                if self.arduino.in_waiting == 0:
                    time.sleep(0.01)
                    continue
                chan, raw = self.read_frame()
                if chan is None:
                        continue
                voltage = (raw * 2 * self.VREF) / (8388607.0 )
                t = time.time() - self.start_time
                self.mode = 'BINARY'
                self.channel = chan
                self.data[chan].append(voltage)
                self.time_data[chan].append(t)
                self.sample_count += 1
                # current_time = time.time()
                # if current_time - start_time >= 1.0:
                #     print(f"Samples per second: {sample_count / (current_time - start_time):.2f} Hz")-*
                #     start_time = current_time
                #     sample_count = 0
            except Exception as e:
                if not self.stop_flag:
                    print(f"Error in read_data: {e}")
                break

    def update_plot(self, mode=None):
        """在主线程中更新绘图"""
        if not self.plot_enabled or not plt.fignum_exists(self.fig.number):
            return
        if mode == "WEIGHT":
            return
        
        def median_filter(raw_list, window):
            arr = np.array(raw_list)
            n = arr.size
            if n == 0:
                return arr
            half = window // 2
            out = np.empty(n)
            for i in range(n):
                lo = max(0, i - half)
                hi = min(n, i + half + 1)
                out[i] = np.median(arr[lo:hi])
            return out
        
        if self.mode == "CYCLE":
            lines = []
            labels = []
            for i in range(8):
                raw = self.data[i][-self.N:]
                if not raw:
                    continue
                filtered = raw if self.median_window <= 1 else median_filter(raw, self.median_window)
                x = np.arange(len(filtered))
                self.line_objects[i].set_data(x, filtered)
                lines.append(self.line_objects[i])
                labels.append(f'Channel {i}')
            self.ax.relim()
            self.ax.autoscale_view()
            self.ax.set_title("CYCLE - All Channels")
            self.ax.set_xlabel("Time (Samples)")
            self.ax.set_ylabel("Voltage (V)")
            self.ax.legend(lines, labels, loc='upper right')
        else:
            if self.channel is None or self.channel not in self.data:
                return
            raw = self.data[self.channel][-self.N:]
            if not raw:
                return
            filtered = raw if self.median_window <= 1 else median_filter(raw, self.median_window)
            x = np.arange(len(filtered))
            self.line_objects[self.channel].set_data(x, filtered)
            self.ax.relim()
            self.ax.autoscale_view()
            self.ax.set_title(f"{self.mode} - Channel {self.channel}")
            self.ax.set_xlabel("Time (Samples)")
            self.ax.set_ylabel("Voltage")
            self.ax.legend([self.line_objects[self.channel]], [f'{self.mode} Channel {self.channel}'])
        try:
            self.fig.canvas.draw_idle()
            plt.pause(0.05)
        except Exception as e:
            print(f"Plot update error: {e}")

    def stop(self, event=None):
        """停止数据采集并关闭图形"""
        if self.stop_flag:
            return
        self.stop_flag = True
        self.send_command("STOP")
        print("\nData collection stopped.")
        if hasattr(self, 'arduino') and self.arduino.is_open:
            self.arduino.close()
        if self.plot_enabled and self.fig is not None:
            plt.close(self.fig)
        if hasattr(self, 'update_job') and self.update_job is not None:
            self.fig.canvas.manager.window.after_cancel(self.update_job)
        run_time = time.time() - self.start_time if self.start_time else 0
        rate = self.sample_count / run_time if run_time > 0 else 0
        print(f"本次运行时长: {run_time:.2f} 秒，平均采样率: {rate:.2f} Hz")

        # 保存 CSV
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.mode}_{timestamp}.csv"
        path = os.path.join(SAVE_DIR, filename)
        with open(path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([f"# Runtime: {run_time:.2f} sec"])
            writer.writerow([f"# SampleRate: {rate:.2f} Hz"])
            writer.writerow(['Channel', 'Runtime(s)', 'Voltage'])
            # 各通道依次写入
            for ch in self.data:
                for t_rel, v in zip(self.time_data[ch], self.data[ch]):
                    writer.writerow([ch, f"{t_rel:.4f}", f"{v:.6f}"])
        print(f"数据已保存到 {path}")

    def periodic_update(self, interval=50):
        if self.plot_enabled and plt.fignum_exists(self.fig.number):
            self.update_plot()
            self.update_job = self.fig.canvas.manager.window.after(interval, self.periodic_update, interval)

    def run_acquisition(self):
        print("\nData Acquisition Modes:")
        print("Commands: SINGLE[0-7], DIFF[0-3], CYCLE, Quit")
        while True:
            if self.stop_flag:
                break
            command = input("Enter command: ").strip().upper()
            if command == "QUIT":
                print("Exiting data acquisition.")
                self.stop()
                break
            elif command.startswith("SINGLE") or command.startswith("DIFF") or command == "CYCLE":
                self.stop_flag = False
                try:
                    if not self.arduino.is_open:
                        self.arduino.open()
                except Exception as e:
                    print(f"Failed to reopen serial port: {e}")
                    continue
                self.data = {i: [] for i in range(8)}
                self.time_data = {i: [] for i in range(8)}
                self.sample_count = 0
                self.start_time = time.time()
                self.send_command(command)
                # 启动后台线程采集数据
                threading.Thread(target=self.read_data, daemon=True).start()
                # 在主线程中定时更新图形
                self.periodic_update()
            else:
                print("Invalid command!")

    def read_frame(self):
        """从串口读取一个完整的 6 字节帧，并返回 (chan, signed24)"""
        state = 0
        while True:
            b = self.arduino.read(1)
            if not b: continue
            if state==0 and b==b'\xA5': state=1
            elif state==1 and b==b'\x5A': state=2
            elif state==2 and b==b'\xA5':
                body = self.arduino.read(5)  # chan, b2,b1,b0,crc
                if len(body)!=5:
                    state=0; continue
                chan, b2,b1,b0,crc_recv = body
                frame = b'\xA5\x5A\xA5' + body[:4]
                if crc8(frame) == crc_recv:
                    raw = (b2<<16)|(b1<<8)|b0
                    if raw & 0x800000: raw -= 1<<24
                    return chan, raw
                else:
                    state = 0
            else:
                state = 0


class WeightMeasurementSubsystem:
    def __init__(self, daq: ADS1256DataAcquisition):
        self.daq = daq
        self.zero_offset = None
        self.scale_factor = None
        self.weight_data = []
        self.weight_time = []
        self.voltage_data = []
        self.stop_flag = False
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 7))
        self.ax.set_title("Weight Measurement")
        self.ax.set_xlabel("Time (Samples)")
        self.ax.set_ylabel("Weight (g)")
        self.N = 10000  # 显示的点数
        self.weight_line, = self.ax.plot([], [], label='Weight (g)')
        self.ax.legend()

        ax_stop = plt.axes([0.8, 0.05, 0.1, 0.075])
        self.btn_stop = Button(ax_stop, 'Stop')
        self.btn_stop.on_clicked(self.stop)
        self.update_job = None

        self.calib_file = "calibration_data.txt"
        self.calibration_coeffs = None
        self._load_calibration_data()

        self.start_time = None
        self.sample_count = 0


    def calibrate_zero_offset(self):
        print("Starting zero point calibration, ensure no load on MEDIAN0 channel...")
        self.daq.send_command("MEDIAN 0")
        zero_samples = []
        for _ in range(200):
            if self.stop_flag:
                return
            sample = self.daq.median0()
            if sample is not None:
                zero_samples.append(sample)
        self.daq.send_command("STOP")
        if zero_samples:
            self.zero_offset = np.median(zero_samples)
            print(f"MEDIAN 0 zero point: {self.zero_offset:.6f} V")
            print("Zero point calibration completed!")
        else:
            print("Zero point calibration failed, please retry!")

    def calibrate_scale_factor(self):
        # 此处保留原有逻辑（省略实现），确保校准数据的获取
        if self.zero_offset is None:
            print("Calibrate zero first!")
            return
        print("请选择标定模式：")
        print("1. 全程非线性标定（至少20个点）")
        print("2. 预使用二点校准")
        mode = input("输入1或2选择模式: ").strip()
        if mode == "1":
            if hasattr(self, 'calibration_coeffs') and self.calibration_coeffs is not None:
                choice = input("检测到已有标定数据，要覆盖吗？(y/n): ").lower()
                if choice != 'y':
                    return
            num_points = int(input("请输入标定点数量（至少20个）: ").strip())
            if num_points < 20:
                print("标定点数量至少需要20个！")
                return
            weights = []
            voltages = []
            data_points = []
            print("开始全程非线性标定...")
            for i in range(num_points):
                known_weight = float(input(f"请输入第{i+1}个标定点的已知重量（g）: ").strip())
                print(f"请放置 {known_weight}g 的砝码，准备好后按Enter继续...")
                input("按Enter继续...")
                self.daq.send_command("MEDIAN 0")
                known_samples = []
                for _ in range(10):
                    sample = self.daq.median0()
                    if sample is not None:
                        known_samples.append(sample)
                if known_samples:
                    known_voltage = np.median(known_samples)
                    voltage_val = known_voltage - self.zero_offset
                    voltages.append(voltage_val)
                    weights.append(known_weight)
                    data_points.append((voltage_val, known_weight))
                else:
                    print(f"第{i+1}个标定点采样失败！")
            if len(weights) > 1:
                coeffs = np.polyfit(voltages, weights, 2)
                self.calibration_coeffs = coeffs
                print("全程非线性标定完成。拟合二次多项式系数为：")
                print(coeffs)
                try:
                    with open(self.calib_file, "w") as f:
                        f.write("Voltage,Weight\n")
                        for v, w in data_points:
                            if not isinstance(v, float) or not isinstance(w, float):
                                raise ValueError("无效的数据类型")
                            f.write(f"{v},{w}\n")
                    print("标定数据已保存到 calibration_data.txt")

                    if not os.path.exists(self.calib_file):
                        raise RuntimeError("数据保存失败")
                    self._load_calibration_data()
                    
                except Exception as e:
                    print("保存标定数据时出错：", e)
                    if os.path.exists("calibration_data.txt"):
                        os.remove("calibration_data.txt")
            else:
                print("标定点不足，无法进行拟合！")
        elif mode == "2":
            # 模式2：二点校准，读取两次称重以确定 full-scale 多项式的位置
            if self.calibration_coeffs is None:
                print("未找到全程标定数据，请先进行全程非线性标定！")
                return

            print("开始二点校准（位置校正）...")
            measured_errors = []
            for i in range(2):
                try:
                    known_weight = float(input(f"请输入第{i+1}个校准点的已知重量（g）: ").strip())
                except ValueError:
                    print("输入无效，请输入数字！")
                    return
                print(f"请放置 {known_weight}g 的砝码，准备好后按Enter继续...")
                input("按Enter继续...")
                self.daq.send_command("MEDIAN 0")
                samples = []
                for _ in range(10):
                    sample = self.daq.median0()
                    if sample is not None:
                        samples.append(sample)
                if not samples:
                    print(f"第{i+1}个校准点未能采集到数据，请检查传感器连接！")
                    return
                measured_voltage = np.median(samples)
                voltage_val = measured_voltage - self.zero_offset
                poly_func = np.poly1d(self.calibration_coeffs)
                predicted_weight = poly_func(voltage_val)
                error = known_weight - predicted_weight
                measured_errors.append(error)
                print(f"校准点{i+1}: 测得电压={measured_voltage:.6f}V, 电压差={voltage_val:.6f}V, " +
                    f"多项式预测重量={predicted_weight:.6f}g, 误差={error:.6f}g")
            # 求两点误差的平均值作为位置修正偏移
            offset_correction = np.mean(measured_errors)
            self.offset_correction = offset_correction
            print(f"二点校准完成，校正偏移量为: {offset_correction:.6f} g")
        else:
            print("选择无效，请输入1或2。")

    def calculate_weight(self):
        """后台线程中连续计算重量（仅更新数据，不调用GUI更新函数）"""
        if self.zero_offset is None:
            print("Please perform zero point calibration first!")
            return
        if not hasattr(self, "calibration_coeffs") or self.calibration_coeffs is None:
            print("Please calibrate scale factor first!")
            return
        print("Starting weight calculation for MEDIAN0...")
        self.daq.send_command("MEDIAN 0")
        if self.start_time is None:
            self.start_time = time.time()
        while not self.stop_flag:
            weight_samples = []
            for _ in range(1):
                if self.stop_flag:
                    break
                sample = self.daq.median0()
                if sample is not None:
                    weight_samples.append(sample)
            if not weight_samples:
                print("Failed to collect weight data, check connection!")
                continue
            voltage = weight_samples[0]
            self.voltage_data.append(voltage)
            net_voltage = voltage - self.zero_offset
            poly_func = np.poly1d(self.calibration_coeffs)
            weight = poly_func(net_voltage)
            if hasattr(self, "offset_correction"):
                weight += self.offset_correction
            elapsed = time.time() - self.start_time
            print(f"Voltage: {voltage:.6f} V, Time: {elapsed:.2f}s, Weight: {weight:.6f} g")
            self.weight_time.append(elapsed)
            self.weight_data.append(weight)
            self.sample_count += len(weight_samples)
        self.daq.send_command("STOP")

    def _load_calibration_data(self):
        """尝试加载已保存的标定数据"""
        try:
            with open(self.calib_file, "r") as f:
                # 跳过标题行
                next(f)
                voltages = []
                weights = []
                for line in f:
                    v, w = line.strip().split(',')
                    voltages.append(float(v))
                    weights.append(float(w))
                
                if len(voltages) >= 20:  # 符合全程标定要求
                    coeffs = np.polyfit(voltages, weights, 2)
                    self.calibration_coeffs = coeffs
                    print("检测到已保存的标定数据，已成功加载！")
        except FileNotFoundError:
            print("未找到标定数据文件，请先进行全程标定")
        except Exception as e:
            print(f"加载标定数据失败: {e}")

    def update_plot(self):
        """在主线程中更新重量测量的图形显示"""
        if not plt.fignum_exists(self.fig.number):
            return
        data = self.weight_data[-self.N:]
        x = np.arange(len(data))
        self.weight_line.set_data(x, data)
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.set_title("Weight Measurement")
        self.ax.set_xlabel("Time (Samples)")
        self.ax.set_ylabel("Weight (g)")
        self.ax.legend([self.weight_line], ['Weight (g)'])
        plt.pause(0.05)

    def stop(self, event=None):
        """停止重量计算并关闭图形"""
        if self.stop_flag:
            return
        self.stop_flag = True
        self.daq.send_command("STOP")
        print("\nWeight measurement stopped.")
        if hasattr(self, 'fig') and self.fig is not None:
            plt.close(self.fig)
        if hasattr(self, 'update_job') and self.update_job is not None:
            self.fig.canvas.manager.window.after_cancel(self.update_job)
        run_time = time.time() - self.start_time if self.start_time else 0
        rate = self.sample_count / run_time if run_time > 0 else 0
        print(f"本次运行时长: {run_time:.2f} 秒，平均采样率: {rate:.2f} Hz")
        # 保存CSV
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"weight_{timestamp}.csv"
        path = os.path.join(SAVE_DIR, filename)
        with open(path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([f"# Runtime: {run_time:.2f} sec"])
            writer.writerow([f"# SampleRate: {rate:.2f} Hz"])
            writer.writerow(['Index', 'Runtime(s)','Voltage(V)', 'Weight_g'])
            for idx, (t_rel, v, w) in enumerate(zip(self.weight_time, self.voltage_data, self.weight_data)):
                writer.writerow([idx, f"{t_rel:.4f}", f"{v:.6f}", f"{w:.6f}"])
        print(f"数据已保存到 {path}")

    def periodic_update(self, interval=50):
        """在主线程中定时更新重量测量图形"""
        if plt.fignum_exists(self.fig.number):
            self.update_plot()
            self.update_job = self.fig.canvas.manager.window.after(interval, self.periodic_update, interval)

    def run(self):
        """重量测量子系统主循环，接收用户命令并启动后台计算线程，同时启动GUI更新"""
        while True:
            print("\nWeight Measurement Modes:")
            print("1. Zero Point Calibration")
            print("2. Scale Factor Calibration")
            print("3. Start Weight Measurement")
            print("4. Quit")
            mode_input = input("Select mode (1/2/3/4): ").strip()
            try:
                if mode_input == "1":
                    self.calibrate_zero_offset()
                elif mode_input == "2":
                    self.calibrate_scale_factor()
                elif mode_input == "3":
                    self.stop_flag = False
                    self.weight_data = []
                    self.weight_time = []
                    self.voltage_data = []
                    self.start_time = time.time()
                    self.sample_count = 0
                    self.stop_flag = False
                    threading.Thread(target=self.calculate_weight, daemon=True).start()
                    self.periodic_update()
                elif mode_input == "4":
                    print("Exiting weight measurement subsystem.")
                    break
                else:
                    print("Invalid input!")
            except Exception as e:
                print(f"An error occurred: {e}")


class SNRAnalysis:
    def __init__(self):
        self.daq = ADS1256DataAcquisition(plot_enabled=False)
        self.weight_sys = WeightMeasurementSubsystem(self.daq)
        self.mode = None  # 1: DAQ, 2: Weight
        self.command = None
        self.noise_stats = {}
        self.signal_stats = {}

    def _get_channel_from_command(self, command):
        if command.startswith("SINGLE"):
            return int(command[6:])
        elif command.startswith("DIFF"):
            return int(command[4:])
        return 0

    def select_mode(self):
        print("请选择数据采集模式：")
        print("1. Data Acquisition (Single/Diff)")
        print("2. Weight Measurement")
        choice = input("Enter choice (1/2): ").strip()
        if choice == '1':
            self.mode = 1
            print("Commands: SINGLE[0-7], DIFF[0-3], Quit")
            command = input("Enter command: ").strip().upper()
            if not (command.startswith("SINGLE") or command.startswith("DIFF") or command == "CYCLE"):
                print("无效命令")
                return False
            self.command = command
            return True
        elif choice == '2':
            self.mode = 2
            while True:
                print("\nWeight Measurement Modes:")
                print("1. Zero Point Calibration")
                print("2. Scale Factor Calibration")
                print("3. Start Weight Measurement")
                print("4. Quit SNR Analysis")
                choice2 = input("Select mode (1/2/3/4): ").strip()
                if choice2 == '1':
                    self.weight_sys.calibrate_zero_offset()
                elif choice2 == '2':
                    self.weight_sys.calibrate_scale_factor()
                elif choice2 == '3':
                    return True
                elif choice2 == '4':
                    return False
                else:
                    print("无效选择，请重新输入。")
        else:
            print("无效选择")
            return False

    def collect_noise_data(self):
        print("采集噪声数据...")
        if self.mode == 1:
            ch = self._get_channel_from_command(self.command)
            self.daq.data[ch] = []
            self.daq.stop_flag = False
            threading.Thread(target=self.daq.read_data, daemon=True).start()
            self.daq.send_command(self.command)
            deadline = time.time() + 10
            while len(self.daq.data[ch]) < 1000 and time.time() < deadline:
                time.sleep(0.01)
            self.daq.send_command("STOP")
            self.daq.stop_flag = True
            data = self.daq.data[ch][-1000:]
        else:
            data = []
            for _ in range(1000):
                v = self.daq.median0()
                if v is not None:
                    data.append(v)
                time.sleep(0.002)

        if len(data) < 1000:
            print("未采集到足够噪声数据")
            return False

        self.noise_stats = {
            'mean': mean(data),
            'var': pvariance(data),
            'std': pstdev(data)
        }
        return True

    def collect_signal_data(self):
        print("采集信号数据...")
        if self.mode == 1:
            ch = self._get_channel_from_command(self.command)
            self.daq.data[ch] = []
            self.daq.stop_flag = False
            threading.Thread(target=self.daq.read_data, daemon=True).start()
            self.daq.send_command(self.command)
            deadline = time.time() + 10
            while len(self.daq.data[ch]) < 1000 and time.time() < deadline:
                time.sleep(0.01)
            self.daq.send_command("STOP")
            self.daq.stop_flag = True
            data = self.daq.data[ch][-1000:]
        else:
            data = []
            for _ in range(1000):
                v = self.daq.median0()
                if v is not None:
                    data.append(v)
                time.sleep(0.002)

        if len(data) < 1000:
            print("未采集到足够信号数据")
            return False

        self.signal_stats = {
            'mean': mean(data),
            'std': pstdev(data)
        }
        return True

    def calculate_snr(self):
        sig_m = self.signal_stats['mean']
        noise_m = self.noise_stats['mean']
        signal_power = (sig_m - noise_m) ** 2
        noise_power = self.noise_stats['var']
        if noise_power == 0:
            print("噪声方差为零，无法计算SNR")
            return

        snr_db = 10 * math.log10(signal_power / noise_power)
        print(f"\nCalculated SNR (dB): {snr_db:.2f}")
        print(f"噪声均值: {noise_m:.6f}, 噪声方差: {noise_power:.6f}")
        print(f"信号均值: {sig_m:.6f}")

        # 保存结果
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"snr_{ts}.csv"
        path = os.path.join(SAVE_DIR, filename)
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['Metric', 'Value'])
            w.writerow(['Noise_Mean', f"{noise_m:.6f}"])
            w.writerow(['Noise_Var', f"{noise_power:.6f}"])
            w.writerow(['Signal_Mean', f"{sig_m:.6f}"])
            w.writerow(['Signal_Std', f"{self.signal_stats['std']:.6f}"])
            w.writerow(['SNR_dB', f"{snr_db:.2f}"])
        print(f"SNR及统计数据已保存到 {path}")
        return snr_db

    def run(self):
        if not self.select_mode():
            return
        input("\nEnsure no signal is applied, then press Enter to start noise acquisition...")
        if not self.collect_noise_data():
            return
        input("\nApply the known signal, then press Enter to start signal acquisition...")
        if not self.collect_signal_data():
            return
        self.calculate_snr()

class CameraTracker:
    def __init__(self, daq: ADS1256DataAcquisition, save_dir=SAVE_DIR, hsv_file="hsv_config.json"):
        self.daq = daq
        self.save_dir = save_dir
        os.makedirs(self.save_dir, exist_ok=True)
        self.hsv_path = os.path.join(self.save_dir, hsv_file)
        # 默认 HSV（可覆盖）
        self.hsv_lower = [0, 100, 100]
        self.hsv_upper = [10, 255, 255]
        self._load_hsv()
        self.origin = None
        self.last_frame = None
        self.last_mask = None
        self.frame_lock = threading.Lock()
        self.disp_data = []  # list of (rel_time, cx, cy, dx, dy, dist)
        self.video_writer = None
        self.cap = None
        self.frame_size = None
        self.fps = None

    def _load_hsv(self):
        try:
            with open(self.hsv_path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
                self.hsv_lower = cfg.get("lower", self.hsv_lower)
                self.hsv_upper = cfg.get("upper", self.hsv_upper)
                print("Loaded HSV config:", self.hsv_lower, self.hsv_upper)
        except FileNotFoundError:
            print("HSV config not found, using defaults.")
        except Exception as e:
            print("Error loading HSV config:", e)

    def _save_hsv(self):
        cfg = {"lower": self.hsv_lower, "upper": self.hsv_upper}
        with open(self.hsv_path, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
        print("Saved HSV config to", self.hsv_path)

    def adjust_hsv_gui(self, cam_index=0):
        """打开摄像头，调整 HSV（可选择保存），按 s 保存并退出，按 q 退出不保存"""
        cap = cv2.VideoCapture(cam_index)
        if not cap.isOpened():
            print("无法打开摄像头进行 HSV 调参")
            return
        cv2.namedWindow("Adjust")
        # 创建 trackbars
        cv2.createTrackbar('H_min', 'Adjust', self.hsv_lower[0], 179, lambda x: None)
        cv2.createTrackbar('S_min', 'Adjust', self.hsv_lower[1], 255, lambda x: None)
        cv2.createTrackbar('V_min', 'Adjust', self.hsv_lower[2], 255, lambda x: None)
        cv2.createTrackbar('H_max', 'Adjust', self.hsv_upper[0], 179, lambda x: None)
        cv2.createTrackbar('S_max', 'Adjust', self.hsv_upper[1], 255, lambda x: None)
        cv2.createTrackbar('V_max', 'Adjust', self.hsv_upper[2], 255, lambda x: None)

        print("调整窗口已打开：按 's' 保存并退出；按 'q' 不保存退出。")
        while True:
            ret, frame = cap.read()
            if not ret:
                print("读帧失败（HSV 调参）")
                break
            hmin = cv2.getTrackbarPos('H_min', 'Adjust')
            smin = cv2.getTrackbarPos('S_min', 'Adjust')
            vmin = cv2.getTrackbarPos('V_min', 'Adjust')
            hmax = cv2.getTrackbarPos('H_max', 'Adjust')
            smax = cv2.getTrackbarPos('S_max', 'Adjust')
            vmax = cv2.getTrackbarPos('V_max', 'Adjust')
            lower = np.array([hmin, smin, vmin], dtype=np.uint8)
            upper = np.array([hmax, smax, vmax], dtype=np.uint8)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower, upper)
            mask_vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            vis = np.hstack((frame, mask_vis))
            cv2.imshow('Adjust', vis)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                self.hsv_lower = [int(hmin), int(smin), int(vmin)]
                self.hsv_upper = [int(hmax), int(smax), int(vmax)]
                self._save_hsv()
                break
            elif key == ord('q'):
                break
        cap.release()
        cv2.destroyWindow('Adjust')

    def start_tracking(self, start_event: threading.Event, stop_event: threading.Event, cam_index=0, record_video=True):
        """启动追踪线程（非阻塞）：
           - start_event: 等待该 event 后正式开始记录（并以 daq.start_time 为时间基准）
           - stop_event: 设置后停止录制并保存 CSV/视频
        """
        t = threading.Thread(target=self._tracking_loop, args=(start_event, stop_event, cam_index, record_video), daemon=True)
        t.start()
        return t

    def _tracking_loop(self, start_event, stop_event, cam_index, record_video):
        self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            print("无法打开摄像头（tracking）")
            stop_event.set()
            return
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 640)
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 480)
        self.frame_size = (w, h)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        if not fps or fps <= 0:
            fps = 20.0
        self.fps = fps

        # 视频文件
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        video_path = os.path.join(self.save_dir, f"camera_{timestamp}.avi")
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        if record_video:
            self.video_writer = cv2.VideoWriter(video_path, fourcc, self.fps, self.frame_size)
            print("Recording video to:", video_path)

        # 等待 start_event（同时主程序会设置 daq.start_time）
        print("Camera waiting for start_event ...")
        start_event.wait()
        print("Camera started tracking")
        # 如果 daq.start_time 未设置，也设一个基准
        if not getattr(self.daq, "start_time", None):
            self.daq.start_time = time.time()

        # 主循环：读帧、分割、质心、记录（时间使用 daq.start_time 基准）
        kernel = np.ones((5, 5), np.uint8)
        while not stop_event.is_set():
            ret, frame = self.cap.read()
            if not ret:
                print("Camera read failed")
                break
            ts_rel = time.time() - self.daq.start_time  # 对齐时间戳
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array(self.hsv_lower, dtype=np.uint8)
            upper = np.array(self.hsv_upper, dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cx = cy = None
            if contours:
                max_cnt = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(max_cnt)
                if area > 30:
                    M = cv2.moments(max_cnt)
                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        cv2.drawContours(frame, [max_cnt], -1, (0,255,0), 2)
                        cv2.circle(frame, (cx, cy), 4, (0,0,255), -1)

            if self.origin is None and cx is not None:
                self.origin = (cx, cy)

            dx = dy = dist = None
            if self.origin is not None and cx is not None:
                dx = cx - self.origin[0]
                dy = cy - self.origin[1]
                dist = math.hypot(dx, dy)
                cv2.putText(frame, f"dx={dx}, dy={dy}, d={dist:.1f}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

            # 记录到内存（稍后写 csv）
            self.disp_data.append((ts_rel, cx, cy, dx, dy, dist))

            # 写视频
            if self.video_writer is not None:
                # 如果 frame size 不匹配，resize
                if (frame.shape[1], frame.shape[0]) != self.frame_size:
                    frame_to_write = cv2.resize(frame, self.frame_size)
                else:
                    frame_to_write = frame
                self.video_writer.write(frame_to_write)

            with self.frame_lock:
                # 存入副本以防冲突
                try:
                    self.last_frame = frame.copy()
                    self.last_mask = mask.copy()
                except Exception:
                    # 若 frame 无法复制（极少见），直接存引用也行
                    self.last_frame = frame
                    self.last_mask = mask

        # 退出：保存 CSV + 释放资源
        cv2.destroyAllWindows()
        if self.video_writer is not None:
            self.video_writer.release()
        if self.cap is not None:
            self.cap.release()
        # 保存位移 CSV
        csv_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(self.save_dir, f"camera_disp_{csv_ts}.csv")
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['time_rel_s', 'cx', 'cy', 'dx', 'dy', 'dist'])
            for row in self.disp_data:
                w.writerow([f"{row[0]:.6f}", row[1], row[2], row[3], row[4], row[5]])
        print("Camera displacement CSV saved:", csv_path)

    def show_latest(self, stop_event):
        """在主线程调用，用来显示最新帧并响应按键（q 停止）"""
        frame = None
        mask = None
        with self.frame_lock:
            if self.last_frame is not None:
                try:
                    frame = self.last_frame.copy()
                except Exception:
                    frame = self.last_frame
            if self.last_mask is not None:
                try:
                    mask = self.last_mask.copy()
                except Exception:
                    mask = self.last_mask

        if frame is not None:
            cv2.imshow("Tracking", frame)
        if mask is not None:
            cv2.imshow("Mask", mask)

        # cv2.waitKey 必须在主线程被调用以响应 GUI 事件
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            stop_event.set()

    def stop(self):
        # not used directly; stop via stop_event
        pass

# ----------------------------------------
# 在 WeightMeasurementSubsystem 中新增方法
# ----------------------------------------
def weight_integrated_loop(self, daq: ADS1256DataAcquisition, stop_event: threading.Event, median_count=3):
    """
    用于同步采集模式：从 daq.data[0] 读取最新 sample 并用已知校准系数计算重量，
    并把结果追加到 self.weight_data/self.weight_time/self.voltage_data。
    这个函数不替换已有的 calculate_weight()，仅供集成场景使用。
    """
    if self.zero_offset is None:
        print("请先完成零点校准（zero_offset 为空）")
        return
    if not hasattr(self, "calibration_coeffs") or self.calibration_coeffs is None:
        print("请先完成 scale calibration（calibration_coeffs 为空）")
        return

    print("Integrated weight loop started (using daq channel 0)...")
    # 循环读取 daq.data[0]
    last_idx = 0
    while not stop_event.is_set():
        # 若 daq 中还没数据，sleep 等待
        if not daq.time_data[0]:
            time.sleep(0.001)
            continue
        # 取最近几帧的中位数防抖
        ch_data = daq.data[0]
        ch_time = daq.time_data[0]
        if not ch_data:
            time.sleep(0.001)
            continue
        # 取尾部最近 median_count 个样本
        samples = ch_data[-median_count:]
        times = ch_time[-median_count:]
        volt = float(np.median(np.array(samples)))
        t_rel = float(times[-1])  # 使用 latest timestamp from daq
        net_v = volt - self.zero_offset
        poly = np.poly1d(self.calibration_coeffs)
        weight = float(poly(net_v))
        if hasattr(self, "offset_correction"):
            weight += self.offset_correction

        # 追加到本地数组（线程安全性：此处为简单追加）
        self.voltage_data.append(volt)
        self.weight_time.append(t_rel)
        self.weight_data.append(weight)
        self.sample_count += 1
        # 频率控制（可按需改）
        time.sleep(0.01)
    print("Integrated weight loop stopped.")

# 把方法绑定到类
WeightMeasurementSubsystem.integrated_weight_loop = weight_integrated_loop

# --------------------
# 新的 main()，增加选项4
# --------------------
def main():
    print("Select System:")
    print("1. Data Acquisition (Single/Diff/Cycle)")
    print("2. Weight Measurement")
    print("3. SNR Analysis")
    print("4. Camera + Synchronized Acquisition (新)")
    choice = input("Enter choice (1/2/3/4): ").strip()
    if choice == "1":
        daq = ADS1256DataAcquisition()
        daq.run_acquisition()
    elif choice == "2":
        daq = ADS1256DataAcquisition(plot_enabled=False)
        weight_sys = WeightMeasurementSubsystem(daq)
        weight_sys.run()
    elif choice == "3":
        snr = SNRAnalysis()
        snr.run()
    elif choice == "4":
        # 集成流程
        daq = ADS1256DataAcquisition(plot_enabled=False)
        weight_sys = WeightMeasurementSubsystem(daq)
        camtracker = CameraTracker(daq)

        while True:
            print("\nCamera + Sync Submenu:")
            print("1. HSV 调参并保存")
            print("2. 称重系统：零点校准 (same as weight subsystem)")
            print("3. 称重系统：标定 (same as weight subsystem)")
            print("4. 开始同步采集（摄像头 + DAQ + 计算重量）")
            print("5. 返回主菜单")
            sub = input("Select (1/2/3/4/5): ").strip()
            if sub == "1":
                camtracker.adjust_hsv_gui()
            elif sub == "2":
                weight_sys.calibrate_zero_offset()
            elif sub == "3":
                weight_sys.calibrate_scale_factor()
            elif sub == "4":
                # 检查校准是否就绪
                if weight_sys.zero_offset is None:
                    print("警告：zero_offset 为空，建议先执行零点校准（选项2）。继续请确认 (y/n).")
                    if input().strip().lower() != 'y':
                        continue
                if getattr(weight_sys, "calibration_coeffs", None) is None:
                    print("警告：calibration_coeffs 为空，建议先执行标定（选项3）。继续请确认 (y/n).")
                    if input().strip().lower() != 'y':
                        continue

                if weight_sys.zero_offset is None:
                    print("警告：zero_offset 为空，建议先执行零点校准（选项2）。继续请确认 (y/n).")
                    if input().strip().lower() != 'y':
                        continue
                if getattr(weight_sys, "calibration_coeffs", None) is None:
                    print("警告：calibration_coeffs 为空，建议先执行标定（选项3）。继续请确认 (y/n).")
                    if input().strip().lower() != 'y':
                        continue

                # 准备事件
                start_evt = threading.Event()
                stop_evt = threading.Event()

                # 清空 daq 缓存与设置 start_time（在 set 之前）
                daq.data = {i: [] for i in range(8)}
                daq.time_data = {i: [] for i in range(8)}
                daq.sample_count = 0
                daq.start_time = time.time()
                daq.stop_flag = False  # 确保标志位清除

                if weight_sys.zero_offset is None:
                    print("警告：zero_offset 为空，建议先执行零点校准（选项2）。继续请确认 (y/n).")
                    if input().strip().lower() != 'y':
                        continue
                if getattr(weight_sys, "calibration_coeffs", None) is None:
                    print("警告：calibration_coeffs 为空，建议先执行标定（选项3）。继续请确认 (y/n).")
                    if input().strip().lower() != 'y':
                        continue

                # 准备事件
                start_evt = threading.Event()
                stop_evt = threading.Event()

                # 不启动 daq.read_data() —— 采用与选项2相同的 median0()/calculate_weight() 路径
                # 清空旧缓存/标志
                daq.data = {i: [] for i in range(8)}
                daq.time_data = {i: [] for i in range(8)}
                daq.sample_count = 0
                daq.stop_flag = False

                # 启动摄像头追踪（等待 start_evt）
                t_cam = camtracker.start_tracking(start_evt, stop_evt, cam_index=0, record_video=True)

                # 启动一个线程，在收到 start_evt 后调用 calculate_weight()（直接复用选项2 的逻辑）
                def weight_runner():
                    # 等待 start 信号后真正开始调用 calculate_weight()
                    start_evt.wait()
                    # calculate_weight() 内会 send_command("MEDIAN 0") 并使用 median0()
                    weight_sys.stop_flag = False
                    weight_sys.calculate_weight()

                t_weight = threading.Thread(target=weight_runner, daemon=True)
                t_weight.start()

                # 设定统一的起始时间（摄像头与重量都使用同一基准）
                common_start = time.time()
                daq.start_time = common_start
                # 允许 weight_sys 使用同一个 start_time（calculate_weight 只有在 start_time None 时才会重写）
                weight_sys.start_time = common_start

                # 正式开始
                print("准备就绪，按 Enter 开始同步采集（按 ctrl+C 或 在摄像头窗口按 'q' 停止）")
                input()
                start_evt.set()
                print("同步采集已开始（CTRL+C 或 摄像头窗口 q 停止）。")

                try:
                    # 主线程循环：显示摄像头帧并更新重量图（必须在主线程执行）
                    while not stop_evt.is_set():
                        camtracker.show_latest(stop_evt)
                        try:
                            weight_sys.update_plot()
                        except Exception:
                            pass
                        time.sleep(0.01)
                except KeyboardInterrupt:
                    print("KeyboardInterrupt -> 停止采集")
                    stop_evt.set()

                # 结束并保存（与之前一致）
                stop_evt.set()
                time.sleep(0.5)
                # 停止重量线程的 flag 并调用保存接口（保持原有 CSV 格式）
                weight_sys.stop_flag = True
                weight_sys.stop()
                # 停止 daq（确保 Arduino 收到 STOP）
                daq.stop_flag = True
                daq.stop()
                print("同步采集结束，所有数据已保存。")

                break
            elif sub == "5":
                break
            else:
                print("Invalid selection.")
    else:
        print("Invalid choice!")

if __name__ == "__main__":
    main()
