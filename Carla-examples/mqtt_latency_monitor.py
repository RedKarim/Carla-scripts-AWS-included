#!/usr/bin/env python

"""
MQTT Latency Monitor for CARLA follower system.

This module provides real-time latency monitoring and CSV logging.
"""

import csv
import datetime
import threading
import time
import queue
from collections import deque

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    import tkinter as tk
    from tkinter import ttk
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("警告: matplotlib/tkinterがインストールされていません。")
    print("レイテンシモニターを使用する場合は: pip install matplotlib")


class MQTTLatencyMonitor:
    """Real-time MQTT latency monitor with CSV logging"""
    
    def __init__(self, csv_filename=None, max_history=1000):
        self.csv_filename = csv_filename or f"mqtt_latency_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.max_history = max_history
        self.latencies = deque(maxlen=max_history)
        self.timestamps = deque(maxlen=max_history)
        self.lock = threading.Lock()
        self.running = False
        self.csv_file = None
        self.csv_writer = None
        self._init_csv()
        self.window = None
        self.fig = None
        self.ax = None
        self.canvas = None
        
    def _init_csv(self):
        """Initialize CSV file for logging"""
        try:
            self.csv_file = open(self.csv_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['Timestamp', 'Latency_ms', 'Latency_type'])
            print(f"レイテンシデータを保存中: {self.csv_filename}")
        except Exception as e:
            print(f"CSVファイル初期化エラー: {e}")
            self.csv_file = None
    
    def record_latency(self, latency_ms, latency_type='round_trip'):
        """Record a latency measurement"""
        timestamp = time.time()
        with self.lock:
            self.latencies.append(latency_ms)
            self.timestamps.append(timestamp)
        
        if self.csv_writer:
            try:
                dt = datetime.datetime.fromtimestamp(timestamp)
                self.csv_writer.writerow([dt.isoformat(), f"{latency_ms:.3f}", latency_type])
                self.csv_file.flush()
            except Exception as e:
                print(f"CSV書き込みエラー: {e}")
    
    def get_stats(self):
        """Get current latency statistics"""
        with self.lock:
            if not self.latencies:
                return None
            latencies_list = list(self.latencies)
            return {
                'current': latencies_list[-1] if latencies_list else 0,
                'min': min(latencies_list),
                'max': max(latencies_list),
                'avg': sum(latencies_list) / len(latencies_list),
                'count': len(latencies_list)
            }
    
    def start_monitor(self):
        """Start the monitoring window"""
        if not MATPLOTLIB_AVAILABLE:
            print("matplotlibが利用できないため、モニターウィンドウを開けません。")
            print("CSVモードで動作します。")
            self.running = True
            return
        
        import threading
        try:
            is_main_thread = threading.current_thread() is threading.main_thread()
        except:
            is_main_thread = False
        
        if not is_main_thread:
            print("別スレッドから実行されているため、GUIモニターは無効です。")
            print("CSVモードで動作します。")
            self.running = True
            return
        
        # Windowsでのtkinterスレッド問題を回避するため、GUIを無効化
        print("Windows環境ではGUIモニターを無効化しています。")
        print("CSVモードで動作します。")
        self.running = True
        return
        
        self.running = True
        
        try:
            self.window = tk.Tk()
            self.window.title("MQTT Latency Monitor")
            self.window.geometry("800x600")
        except Exception as e:
            print(f"モニターウィンドウ初期化エラー: {e}")
            print("CSVモードで動作します。")
            return
        
        self.fig, self.ax = plt.subplots(figsize=(8, 5))
        self.ax.set_xlabel('Time (seconds)')
        self.ax.set_ylabel('Latency (ms)')
        self.ax.set_title('MQTT Latency (Real-time)')
        self.ax.grid(True)
        
        self.line, = self.ax.plot([], [], 'b-', label='Latency')
        self.ax.legend()
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.window)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        stats_frame = ttk.Frame(self.window)
        stats_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.stats_labels = {}
        for i, stat in enumerate(['current', 'min', 'max', 'avg']):
            label = ttk.Label(stats_frame, text=f"{stat.capitalize()}: -- ms")
            label.grid(row=0, column=i, padx=10)
            self.stats_labels[stat] = label
        
        count_label = ttk.Label(stats_frame, text="Count: 0")
        count_label.grid(row=0, column=4, padx=10)
        self.stats_labels['count'] = count_label
        
        csv_label = ttk.Label(stats_frame, text=f"CSV: {self.csv_filename}", font=('Arial', 8))
        csv_label.grid(row=1, column=0, columnspan=5, pady=5)
        
        def update_plot(frame):
            with self.lock:
                if not self.latencies:
                    return
                
                latencies_list = list(self.latencies)
                timestamps_list = list(self.timestamps)
                
                if len(timestamps_list) > 1:
                    start_time = timestamps_list[0]
                    relative_times = [(t - start_time) for t in timestamps_list]
                    self.line.set_data(relative_times, latencies_list)
                    self.ax.relim()
                    self.ax.autoscale_view()
                    self.canvas.draw()
                
                stats = self.get_stats()
                if stats:
                    self.stats_labels['current'].config(text=f"Current: {stats['current']:.2f} ms")
                    self.stats_labels['min'].config(text=f"Min: {stats['min']:.2f} ms")
                    self.stats_labels['max'].config(text=f"Max: {stats['max']:.2f} ms")
                    self.stats_labels['avg'].config(text=f"Avg: {stats['avg']:.2f} ms")
                    self.stats_labels['count'].config(text=f"Count: {stats['count']}")
        
        def on_closing():
            self.running = False
            if self.csv_file:
                try:
                    stats = self.get_stats()
                    if stats:
                        print("\n=== レイテンシ統計 ===")
                        print(f"サンプル数: {stats['count']}")
                        print(f"現在値: {stats['current']:.2f} ms")
                        print(f"最小値: {stats['min']:.2f} ms")
                        print(f"最大値: {stats['max']:.2f} ms")
                        print(f"平均値: {stats['avg']:.2f} ms")
                        print("==================\n")
                    self.csv_file.close()
                    print(f"レイテンシデータを保存しました: {self.csv_filename}")
                except Exception as e:
                    print(f"CSVファイルクローズエラー: {e}")
            try:
                self.window.quit()
            except:
                pass
            try:
                self.window.destroy()
            except:
                pass
        
        self.window.protocol("WM_DELETE_WINDOW", on_closing)
        
        self.window.after(100, self._update_plot)
        
        try:
            import sys
            if sys.platform == 'win32':
                try:
                    import ctypes
                    ctypes.windll.shcore.SetProcessDpiAwareness(1)
                except:
                    pass
            
            self.window.after(100, self._update_plot)
            self.window.mainloop()
        except RuntimeError as e:
            if "different apartment" in str(e) or "Tcl" in str(e) or "main loop" in str(e):
                print("警告: モニターウィンドウを別スレッドで実行できません。")
                print("レイテンシデータはCSVファイルに保存されます。")
                if self.window:
                    try:
                        self.window.destroy()
                    except:
                        pass
                    self.window = None
            else:
                raise
        except Exception as e:
            print(f"モニターウィンドウエラー: {e}")
            print("CSVモードで動作します。")
            if self.window:
                try:
                    self.window.destroy()
                except:
                    pass
                self.window = None
    
    def _update_plot(self):
        """Update plot periodically"""
        if not self.running or not self.window:
            return
        
        try:
            with self.lock:
                if not self.latencies:
                    self.window.after(100, self._update_plot)
                    return
                
                latencies_list = list(self.latencies)
                timestamps_list = list(self.timestamps)
                
                if len(timestamps_list) > 1:
                    start_time = timestamps_list[0]
                    relative_times = [(t - start_time) for t in timestamps_list]
                    self.line.set_data(relative_times, latencies_list)
                    self.ax.relim()
                    self.ax.autoscale_view()
                    self.canvas.draw()
                
                stats = self.get_stats()
                if stats:
                    self.stats_labels['current'].config(text=f"Current: {stats['current']:.2f} ms")
                    self.stats_labels['min'].config(text=f"Min: {stats['min']:.2f} ms")
                    self.stats_labels['max'].config(text=f"Max: {stats['max']:.2f} ms")
                    self.stats_labels['avg'].config(text=f"Avg: {stats['avg']:.2f} ms")
                    self.stats_labels['count'].config(text=f"Count: {stats['count']}")
            
            self.window.after(100, self._update_plot)
        except Exception as e:
            print(f"プロット更新エラー: {e}")
            if self.window:
                self.window.after(100, self._update_plot)
    
    def stop_monitor(self):
        """Stop the monitoring window"""
        self.running = False
        if self.csv_file:
            try:
                stats = self.get_stats()
                if stats:
                    print("\n=== レイテンシ統計 ===")
                    print(f"サンプル数: {stats['count']}")
                    print(f"現在値: {stats['current']:.2f} ms")
                    print(f"最小値: {stats['min']:.2f} ms")
                    print(f"最大値: {stats['max']:.2f} ms")
                    print(f"平均値: {stats['avg']:.2f} ms")
                    print("==================\n")
                self.csv_file.close()
                print(f"レイテンシデータを保存しました: {self.csv_filename}")
            except Exception as e:
                print(f"CSVファイルクローズエラー: {e}")
        if self.window:
            try:
                self.window.quit()
            except:
                pass
            try:
                self.window.destroy()
            except:
                pass
