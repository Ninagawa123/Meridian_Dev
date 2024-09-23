import sys
import time
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

class SineWavePlotter:
    def __init__(self, num_waves=60, period=2, width=800, height=600):
        self.app = QApplication([])
        self.win = pg.GraphicsLayoutWidget(show=True, title="Sine Wave Plotter")
        self.win.resize(width, height)
        self.plot = self.win.addPlot(title="Sine Waves")
        self.plot.setYRange(-1, 1)
        self.plot.setXRange(0, 6)  # 6秒分の幅
        self.plot.setLabel('left', 'Amplitude')
        self.plot.setLabel('bottom', 'Time', units='s')

        self.num_waves = num_waves
        self.period = period
        self.time_range = 6  # 6秒分の幅
        self.update_interval = 10  # 10ミリ秒ごとに更新（1/100秒）
        self.scroll_threshold = 5  # 5秒後にスクロール開始

        self.waves = [np.random.uniform(0, self.period) for _ in range(num_waves)]
        self.start_time = time.time()

        self.curves = [self.plot.plot(pen=pg.intColor(i, self.num_waves)) for i in range(self.num_waves)]

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(self.update_interval)

        # パフォーマンス最適化
        self.plot.setDownsampling(auto=True, mode='peak')
        self.plot.setClipToView(True)
        self.plot.setAutoVisible(y=True)

    def update(self):
        current_time = time.time() - self.start_time
        
        if current_time > self.scroll_threshold:
            self.plot.setXRange(current_time - self.time_range, current_time, padding=0)

        t = np.linspace(max(0, current_time - self.time_range), current_time, 1000)
        
        for curve, phase in zip(self.curves, self.waves):
            y = np.sin(2 * np.pi * (t - phase) / self.period)
            curve.setData(t, y)

    def run(self):
        if (sys.flags.interactive != 1) or not hasattr(sys, 'ps1'):
            self.app.exec_()

if __name__ == "__main__":
    plotter = SineWavePlotter()
    plotter.run()
