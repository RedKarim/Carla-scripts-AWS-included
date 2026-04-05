#!/usr/bin/env python

"""
Simple MQTT Latency Monitor (CSV only, no GUI)

This is a fallback version that only logs to CSV without GUI.
Useful when matplotlib/tkinter has threading issues.
"""

import csv
import datetime
import threading
import time
from collections import deque


class SimpleMQTTLatencyMonitor:
    """Simple latency monitor with CSV logging only (no GUI)"""
    
    def __init__(self, csv_filename=None, max_history=1000):
        self.csv_filename = csv_filename or f"mqtt_latency_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.max_history = max_history
        self.latencies = deque(maxlen=max_history)
        self.timestamps = deque(maxlen=max_history)
        self.lock = threading.Lock()
        self.running = True
        self.csv_file = None
        self.csv_writer = None
        self._init_csv()
        
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
        """Start monitoring (CSV only, no GUI)"""
        print(f"レイテンシモニター開始 (CSVモード): {self.csv_filename}")
        print("統計情報を表示するには、CSVファイルを確認してください。")
        self.running = True
    
    def stop_monitor(self):
        """Stop the monitoring"""
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
