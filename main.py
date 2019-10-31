# ------------------ PySide2 - Qt Designer - Matplotlib ------------------
import threading
import time
from PySide2.QtWidgets import *
from PySide2.QtUiTools import QUiLoader
from PySide2.QtCore import QFile
from PySide2.QtGui import QPixmap, QGuiApplication
from qtutils import inmain  # Used for callback on main thread for plotting

from matplotlib.backends.backend_qt5agg import (
    FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT as NavigationToolbar)

from matplotlib.figure import Figure

import numpy as np
import scipy.fftpack
import serial
import serial.tools.list_ports

# Sampling frequency
fs = 500
# Time between samples
ts = 1 / fs
# Time sampled = 1 second. I'm taking 500 samples at 500 Hz
# Generate array with the times where samples will be taken


# ------------------ MplWidget ------------------
class MplWidget(QWidget):
    """This class defines the properties of the matplotlib canvas and creates objects to plot."""
    # TODO: Move to module
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        # Declare an object that will be used to plot
        self.canvas = FigureCanvas(Figure(tight_layout=True, facecolor='#f0f0f0'))
        self.toolbar = NavigationToolbar(self.canvas, self)
        # Create a layout for embedding toolbar at plot bottom
        vertical_layout = QVBoxLayout()
        vertical_layout.setMargin(1)
        vertical_layout.setSpacing(0)
        # Add plot to layout
        vertical_layout.addWidget(self.canvas)
        # Add toolbar referencing to the plot to the layout
        # vertical_layout.addWidget(NavigationToolbar(self.canvas, self))
        vertical_layout.addWidget(self.toolbar)

        # For 2 vertical subplots, 1 horizontal, add subplot 1 and 2
        self.canvas.ax1 = self.canvas.figure.add_subplot(211)
        self.canvas.ax2 = self.canvas.figure.add_subplot(212)
        # Graph properties
        # Add axis title
        self.canvas.ax1.set_title('Señal')
        self.canvas.ax1.set_xlim(0, 1)
        self.canvas.ax1.set_ylim(-30000, 30000)
        self.canvas.ax1.set_xlabel('Tiempo [s]')
        self.canvas.ax1.set_ylabel('Aceleración [mm/s^2]')
        self.canvas.ax2.set_title("Transformada de Fourier")
        self.canvas.ax1.ticklabel_format(axis='y', style='sci', scilimits=(-2, 2))
        self.canvas.ax2.set_xlim(1, 249)
        self.canvas.ax2.set_ylim(0, 10)  # 512)
        self.canvas.ax2.set_xlabel('Frecuencia [Hz]')
        self.canvas.ax2.set_ylabel('Velocidad [mm/s] RMS')

        t = np.linspace(0, 1, fs)  # los*fs = number of seconds x samples per second
        xf = np.linspace(0.0, fs / 2.0 - 1, int(fs / 2))
        # fs/2 appears twice because the second is number of samples but I'm sampling for one second

        self.canvas.line1, = self.canvas.ax1.plot(t, np.zeros(500, dtype=np.int8), label='Señal', linewidth=0.5)
        self.canvas.line2, = self.canvas.ax2.plot(xf, np.zeros(250, dtype=np.int8), 'co-', markersize=2, label='FFT',
                                                  linewidth=0.5)
        self.canvas.ax1.legend(loc='upper right', framealpha=0.3)
        self.canvas.ax2.legend(loc='upper right', framealpha=0.3)
        # self.canvas.ax3 = self.canvas.figure.add_axes()
        # self.canvas.ax3 = self.canvas.ax2.twinx()
        # Check https://stackoverflow.com/questions/21583965/matplotlib-cursor-value-with-two-axes
        # Set Widget as the layout composed of plot an toolbar
        self.setLayout(vertical_layout)

    # ------------------ MainWidget ------------------


class MainWidget(QWidget):
    """This widget is imported as the main container for the app.
    It loads a designer file and defines the behavior for most components.

    It uses MplWidget class to define how file-loaded WaveWidget behaves.

    """

    # TODO: Refactor
    def __init__(self):
        QWidget.__init__(self)

        # Set filename to read
        designer_file = QFile("ui.ui")
        # Open UI file
        designer_file.open(QFile.ReadOnly)

        # Call file loader
        loader = QUiLoader()
        # Pass MplWidget to loader so it knows what to do when it reads it from UI file
        loader.registerCustomWidget(MplWidget)
        # Load file
        self.ui = loader.load(designer_file, self)

        # Don't need the file anymore, close it.
        designer_file.close()

        # Assign radial button ids
        self.ui.sensor.setId(self.ui.rad_0, 0)
        self.ui.sensor.setId(self.ui.rad_1, 1)
        self.ui.sensor.setId(self.ui.rad_2, 2)
        self.ui.sensor.setId(self.ui.rad_3, 3)

        # Assigns action to button
        self.ui.sensor.buttonClicked.connect(self.get_sensor)
        self.ui.btn_serial_refresh.clicked.connect(self.get_serial_ports)
        self.ui.btn_serial_connect.clicked.connect(self.connect_to_serial)
        self.ui.s_f.valueChanged.connect(self.fft_at_f)
        self.ui.btn_mp.clicked.connect(self.calc_mp)
        self.ui.btn_vt.clicked.connect(self.calc_vt_mc_a0)
        self.ui.btn_ma.clicked.connect(self.calc_ma_mb)
        self.ui.btn_limpiar.clicked.connect(self.clear_v_fields)
        self.ui.btn_v0.clicked.connect(lambda: self.copy_val_at_f(self.ui.le_v0))
        self.ui.btn_v1.clicked.connect(lambda: self.copy_val_at_f(self.ui.le_v1))
        self.ui.btn_v2.clicked.connect(lambda: self.copy_val_at_f(self.ui.le_v2))
        self.ui.btn_v3.clicked.connect(lambda: self.copy_val_at_f(self.ui.le_v3))
        # self.ui.btn_rndSignal.clicked.connect(self.update_graph)  # Button removed
        # This function is called in a thread created on the first time you call connect_to_serial

        # Set process
        self.freq = 16
        self.stop_event = threading.Event()
        self.reading = threading.Event()
        self.sensor = 0
        self.method = threading.Thread(target=self.update_graph, name='Data update', args=(), daemon=True)
        self.serial = serial.Serial(port=None, baudrate=115200, timeout=1.5)

        # Changes the UI title. Cannot be done from designer as we are adding a widget,not defining mainwindow.
        self.setWindowTitle("Método de las cuatro corridas sin fase")
        # self.setWindowIcon(QIcon('icon.png'))
        self.setWindowIcon(QPixmap.fromImage('icon.png'))

        # Create a grid layout
        # grid_layout = QGridLayout()
        # Adds what was loaded from file to layout
        # grid_layout.addWidget(self.ui)
        # Sets window layout to created grid layout
        # self.setLayout(grid_layout)
        self.setLayout(self.ui.layout())
        # self.get_serial_ports()

    def update_graph(self):
        """This method is run on a thread.
        It uses a 'stop_event' event to define if it will capture data or not.
        If not capturing, it sleeps to reduce CPU usage.
        When capturing, it sends a request to DAQ (Arduino) with the sensor it wants to read,
        which it gets from the QButtonGroup 'sensor'.
        After receiving the sensor it wants to read, the DAQ performs measurements at 500Hz and returns a string.
        The communication is secured by a 'reading' event so the serial port is not closed during comunication.
        The thread waits for this string and performs decoding and processing to convert it into a numpy array,
        which eases other processing, like removing the bites offset and transforming them into real units.
        After, it performs fft, and then integrates the fft dividing by each corresponding frequency.
        Finally, it sets data for the plot axes, and sends plotting signal to main thread."""
        while True:
            while not self.stop_event.is_set():
                try:
                    inmain(self.ui.status_lbl.setText, 'Leyendo datos @ {} del sensor {}'.format(
                        self.serial.port, self.sensor)
                           )
                    self.reading.set()
                    if not self.stop_event.is_set():
                        self.serial.write('{}'.format(self.sensor).encode('ASCII'))
                        time.sleep(0.1)
                        signal = self.serial.readline()
                    else:
                        continue
                    self.reading.clear()
                    # print(signal)  # Make a logging
                    signal = signal.decode('ASCII')
                    signal = signal.split(',')
                    if len(signal) != 500:
                        continue
                except UnicodeDecodeError as e:
                    print(e)
                    continue
                except TypeError as e:
                    print('Se ha interrumpido la comunicación.\n{}: {}'.format(
                        type(e), e))
                    continue
                except AttributeError as e:
                    print('Se ha interrumpido la comunicación.\n{}: {}'.format(
                        type(e), e))
                    continue
                except serial.SerialException as e:
                    if self.stop_event.is_set():
                        print('Se ha interrumpido la comunicación.\n{}'.format(e))
                    else:
                        self.ui.status_lbl.setText('Ha desconectado el DAQ. Conéctelo y reinicie la toma de datos.')
                        self.serial.close()
                        self.stop_event.set()
                    continue
                # except Exception as e:
                #     print(e, type(e))
                #     continue
                try:
                    signal_p = list(map(int, signal))
                    signal_p = 3 * 1.68 * 9810 * (np.array(signal_p)-512) / 512  # 9810 y no 9.81 porque es mm/s2
                    # print(signal_p)  # Signal debug
                    yf = scipy.fftpack.fft(signal_p)
                    xf = np.linspace(0.0, fs / 2.0 - 1, fs / 2)
                    xf[0] = 2 / (np.sqrt(2) * 2 * np.pi)
                    yf = (2.0 / fs * np.abs(yf[:fs // 2]))
                    yfv = yf / (2 * np.pi * xf)
                    yfv = yfv / np.sqrt(2)
                    self.ui.le_f.setText(f'{yfv[self.freq]}')
                    self.ui.le_f.setText(f'{yf[self.freq]}')
                    # Clear previous data sets
                    # self.ui.WaveWidget.canvas.ax3.clear()
                    # Add new data
                except RuntimeError:
                    continue
                except Exception as e:
                    print('{}: {}.\n\n{}\n'.format(type(e), e, signal))
                    # Printing both legends
                    # leg = lns2 + lns3
                    # labs = [l.get_label() for l in leg]
                    # self.ui.WaveWidget.canvas.ax2.legend(leg, labs, loc='upper right')
                    # Update graph with new data
                self.ui.WaveWidget.canvas.line1.set_ydata(signal_p)
                self.ui.WaveWidget.canvas.line2.set_ydata(yfv)  # 2.0 / fs * np.abs(yf[:fs // 2]))

                # self.ui.WaveWidget.canvas.ax1.plot(t, signal_p)
                # self.ui.WaveWidget.canvas.ax2.plot(xf, 2.0/fs * np.abs(yf[:fs//2]),  'co-', alpha=0.5,  # fs = n_samples
                #                                    label='Amplitude')

                inmain(self.ui.WaveWidget.canvas.draw)  # Moving to event callback method
                inmain(self.ui.WaveWidget.canvas.flush_events)
            time.sleep(0.2)

    def get_sensor(self):
        """Changes sensor variable to perform request to DAQ using radial buttons."""
        self.sensor = self.ui.sensor.checkedId()

    def fft_at_f(self):
        self.freq = self.ui.s_f.value()
        self.ui.freq_box.setTitle('Valor @f: {}'.format(self.freq))
        y_data = self.ui.WaveWidget.canvas.line2.get_ydata()
        self.ui.le_f.setText('{}'.format(y_data[self.freq]))

    def get_serial_ports(self):
        """Executed at app start and when update button is pressed.
        It gets all available serial ports and list them in the combo box."""
        self.ui.status_lbl.setText('Actualizando puertos disponibles...')
        QGuiApplication.processEvents()
        self.ui.cmb_serial_list.clear()
        for item in list(serial.tools.list_ports.comports()):
            self.ui.cmb_serial_list.addItem(item.description, item.device)
        self.ui.status_lbl.setText('Puertos disponibles actualizados.')

    def connect_to_serial(self):
        """Run through connect button.
        When first run, starts the update_graph thread.
        After, it changes the stop_event state and close/opens serial ports.
        Before closing serial ports, it waits for the thread to finish reading."""
        if self.method.is_alive():
            if self.stop_event.is_set():
                self.serial.port = self.ui.cmb_serial_list.currentData()
                self.ui.status_lbl.setText('Leyendo datos @ {} del sensor {}'.format(self.serial.port, self.sensor))
                self.serial.open()
                self.stop_event.clear()
            else:
                self.ui.status_lbl.setText('Deteniendo comunicación...')
                QGuiApplication.processEvents()
                self.stop_event.set()
                while self.reading.is_set():
                    time.sleep(0.2)
                self.serial.close()
                self.ui.status_lbl.setText('Comunicación detenida.')

        else:
            self.serial.close()
            self.serial.port = self.ui.cmb_serial_list.currentData()
            self.ui.status_lbl.setText('Leyendo datos @ {}'.format(self.serial.port))
            self.serial.open()
            self.method.start()

    def copy_val_at_f(self, le):
        le.setText(self.ui.le_f.text())

    def calc_mp(self):
        try:
            pr = float(self.ui.le_pr.text())
            vr = float(self.ui.le_vr.text())
            r = float(self.ui.le_r.text())
            if pr < 0: raise ValueError("Pr debe ser un decimal positivo.")
            if vr < 0: raise ValueError("Vr debe ser un decimal positivo.")
            if r < 0: raise ValueError("r debe ser un decimal positivo.")
            mp = 1000 * (0.1 * pr * 9.81) / (((2 * np.pi * vr) ** 2) * r)
            self.ui.le_mp.setText(str(mp))
            self.ui.status_lbl.setText('Se ha calculado Mp.')
        except ValueError:
            self.ui.status_lbl.setText('Debe llenar los campos Pr, Vr, y r con decimales positivos.')

    def calc_vt_mc_a0(self):
        try:
            v0 = float(self.ui.le_v0.text())
            v1 = float(self.ui.le_v1.text())
            v2 = float(self.ui.le_v2.text())
            mp = float(self.ui.le_mp.text())
            if v0 < 0: raise ValueError("V0 debe ser un decimal positivo.")
            if v1 < 0: raise ValueError("V1 debe ser un decimal positivo.")
            if v2 < 0: raise ValueError("V2 debe ser un decimal positivo.")
            if mp < 0: raise ValueError("Mp debe ser un decimal positivo.")
            vt = (abs((v1 ** 2) + (v2 ** 2) - 2 * (v0 ** 2)) / 2) ** 0.5
            mc = mp * (v0 / vt)
            val = ((v2 ** 2) - (v1 ** 2)) / (4 * vt * v0)
            if val < -1 or val > 1: raise ArithmeticError
            a0 = np.rad2deg(np.arccos(val))
            self.ui.le_vt.setText(str(vt))
            self.ui.le_mc.setText(str(mc))
            self.ui.le_a0.setText(str(a0))
            self.ui.status_lbl.setText('Se ha calculado Vt, Mc, y α0.')
        except ValueError:
            self.ui.status_lbl.setText('Debe llenar los campos V0, V1, V2, y Mp con decimales positivos.')
        except ArithmeticError:
            self.ui.status_lbl.setText(f'Se está intentando calcular el arcocoseno del valor {val}. '
                                       f'Revise las mediciones nuevamente.')
        except TypeError:
            self.ui.status_lbl.setText(f'Se ha obtenido un valor complejo para Vt {vt}. '
                                       f'Revise las mediciones nuevamente.')

    def clear_v_fields(self):
        self.ui.le_v0.setText("")
        self.ui.le_v1.setText("")
        self.ui.le_v2.setText("")
        self.ui.le_v3.setText("")
        self.ui.le_vt.setText("")
        self.ui.le_mc.setText("")
        self.ui.le_a0.setText("")
        self.ui.le_a.setText("")
        self.ui.le_b.setText("")
        self.ui.le_ma.setText("")
        self.ui.le_mb.setText("")

    def calc_ma_mb(self):
        try:
            mc = float(self.ui.le_mc.text())
            a = np.deg2rad(float(self.ui.le_a.text()))
            b = np.deg2rad(float(self.ui.le_b.text()))
            if mc < 0: raise ValueError("Mc debe ser un decimal positivo.")
            if a < 0: raise ValueError("α debe ser un decimal positivo.")
            if b < 0: raise ValueError("β debe ser un decimal positivo.")
            ma = mc * (np.sin(b)/np.sin(a+b))
            mb = mc * (np.sin(a)/np.sin(a+b))
            self.ui.le_ma.setText(str(ma))
            self.ui.le_mb.setText(str(mb))
        except ValueError:
            self.ui.status_lbl.setText('Debe llenar los campos Mc, α, y β con decimales positivos.')


if __name__ == '__main__':
    app = QApplication([])
    window = MainWidget()
    window.get_serial_ports()
    window.show()
    app.exec_()
