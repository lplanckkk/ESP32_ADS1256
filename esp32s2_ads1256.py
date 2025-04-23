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

SAVE_DIR = r"C:\Users\rriis\Desktop\新しいフォルダー\SMA_FT232-main\ADS1256\esp32_ads1256_data"
os.makedirs(SAVE_DIR, exist_ok=True)

class ADS1256DataAcquisition:
    def __init__(self, port="COM7", baudrate=2000000, plot_enabled=True):
        try:
            self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            raise

        self.data = {i: [] for i in range(8)}  # 存储8个通道的数据
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
        self.N = 80  # 显示的点数
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

    def send_command(self, command):
        """发送命令到Arduino"""
        try:
            self.arduino.write((command + '\n').encode())
            time.sleep(0.1)
        except Exception as e:
            print(f"Error sending command '{command}': {e}")

    def read_diff0(self, num_samples=30):
        """读取 DIFF0 通道的多个样本，并返回中位数电压"""
        samples = []
        for _ in range(num_samples):
            if self.stop_flag:
                return None
            try:
                line = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    parts = line.split(',')
                    if len(parts) == 3 and parts[0] == "DIFF" and parts[1] == "0":
                        samples.append(int(parts[2]))
            except Exception as e:
                print(f"Reading error: {e}")
        if not samples:
            print("Failed to collect data, check hardware connection!")
            return None
        
        samples = np.sort(samples)
        trimmed_adc = samples[1:-1] if len(samples) > 2 else samples
        median_adc = np.median(trimmed_adc)
        voltage = (median_adc * 2 * self.VREF) / (8388607.0 * 64)
        return voltage

    def read_data(self):
        """后台线程中持续从Arduino读取数据（仅更新数据，不调用GUI更新函数）"""
        self.sample_count = 0
        while not self.stop_flag:
            try:
                if self.arduino.in_waiting == 0:
                    time.sleep(0.01)
                    continue
                line = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    parts = line.split(',')
                    if len(parts) == 3:
                        mode_str, ch_str, adc_value_str = parts
                        try:
                            adc_value = int(adc_value_str)
                        except ValueError:
                            continue
                        voltage = (adc_value * 2 * self.VREF) / (8388607.0 * 64)
                        self.mode = mode_str
                        try:
                            self.channel = int(ch_str)
                        except ValueError:
                            continue
                        print(f"Mode: {self.mode}, Channel: {self.channel}, Voltage: {voltage:.6f} V")
                        self.data[self.channel].append(voltage)
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
        if self.mode == "CYCLE":
            lines = []
            labels = []
            for i in range(8):
                data = self.data[i][-self.N:]
                if data:
                    x = np.arange(len(data))
                    self.line_objects[i].set_data(x, data)
                    lines.append(self.line_objects[i])
                    labels.append(f'Channel {i}')
            self.ax.relim()
            self.ax.autoscale_view()
            self.ax.set_title("CYCLE - All Channels")
            self.ax.set_xlabel("Time (Samples)")
            self.ax.set_ylabel("Voltage (V)")
            self.ax.legend(lines, labels, loc='upper right')
        else:
            if self.channel is None:
                return
            data = self.data[self.channel][-self.N:]
            if not data:
                return
            x = np.arange(len(data))
            self.line_objects[self.channel].set_data(x, data)
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
            # 写表头
            writer.writerow(['Channel', 'Voltage'])
            # 各通道依次写入
            for ch, voltages in self.data.items():
                for v in voltages:
                    writer.writerow([ch, f"{v:.6f}"])
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
                self.sample_count = 0
                self.start_time = time.time()
                self.send_command(command)
                # 启动后台线程采集数据
                threading.Thread(target=self.read_data, daemon=True).start()
                # 在主线程中定时更新图形
                self.periodic_update()
            else:
                print("Invalid command!")


class WeightMeasurementSubsystem:
    def __init__(self, daq: ADS1256DataAcquisition):
        self.daq = daq
        self.zero_offset = None
        self.scale_factor = None
        self.weight_data = []
        self.stop_flag = False
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 7))
        self.ax.set_title("Weight Measurement")
        self.ax.set_xlabel("Time (Samples)")
        self.ax.set_ylabel("Weight (g)")
        self.N = 80  # 显示的点数
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
        print("Starting zero point calibration, ensure no load on DIFF0 channel...")
        self.daq.send_command("DIFF0")
        zero_samples = []
        for _ in range(30):
            if self.stop_flag:
                return
            sample = self.daq.read_diff0()
            if sample is not None:
                zero_samples.append(sample)
        if zero_samples:
            self.zero_offset = np.median(zero_samples)
            print(f"DIFF0 zero point: {self.zero_offset:.6f} V")
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
                self.daq.send_command("DIFF0")
                known_samples = []
                for _ in range(30):
                    sample = self.daq.read_diff0()
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
                self.daq.send_command("DIFF0")
                samples = []
                for _ in range(30):
                    sample = self.daq.read_diff0()
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
        print("Starting weight calculation for DIFF0...")
        self.daq.send_command("DIFF0")
        while not self.stop_flag:
            weight_samples = []
            for _ in range(30):
                if self.stop_flag:
                    break
                sample = self.daq.read_diff0()
                if sample is not None:
                    weight_samples.append(sample)
            if not weight_samples:
                print("Failed to collect weight data, check connection!")
                continue
            voltage = np.median(weight_samples)
            net_voltage = voltage - self.zero_offset
            poly_func = np.poly1d(self.calibration_coeffs)
            weight = poly_func(net_voltage)
            if hasattr(self, "offset_correction"):
                weight += self.offset_correction
            print(f"Voltage: {voltage:.6f} V, Zero: {self.zero_offset:.6f} V, Weight: {weight:.6f} g")
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
            writer.writerow(['SampleIndex', 'Weight_g'])
            for idx, w in enumerate(self.weight_data):
                writer.writerow([idx, f"{w:.6f}"])
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
                v = self.daq.read_diff0(num_samples=10)
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
                v = self.daq.read_diff0(num_samples=10)
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

def main():
    print("Select System:")
    print("1. Data Acquisition (Single/Diff/Cycle)")
    print("2. Weight Measurement")
    print("3. SNR Analysis")
    choice = input("Enter choice (1/2/3): ").strip()
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
    else:
        print("Invalid choice!")

if __name__ == "__main__":
    main()
