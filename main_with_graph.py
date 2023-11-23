import serial
import struct
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Configuración del puerto serial
#puerto = '/dev/ttyUSB0'  EN CASO DE CORRER CON LINUX
puerto = 'COM5'
baudios = 9600

# Inicializa la lista de datos y tiempos
datos_adc = []
tiempos = []

try:
    ser = serial.Serial(puerto, baudios, timeout=1)
    ser.flush()
except Exception as e:
    print(f"Error al abrir el puerto serial: {e}")
    exit()

# Función para actualizar el gráfico y escribir en el archivo de registro
def actualizar(frame):
    if ser.in_waiting >= 2:
        try:
            data = ser.read(2)
            valor_adc = struct.unpack('<H', data)[0]
            grados = valor_adc * 330 / 4096

            # Obtener la marca de tiempo actual
            marca_tiempo = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

            print(f"{marca_tiempo} - Temperatura: {grados}°C")

            datos_adc.append(grados)
            tiempos.append(time.time())

            # Actualizar gráfico
            plt.cla()
            plt.plot(tiempos, datos_adc)
            plt.xlabel('Tiempo (s)')
            plt.ylabel('Temperatura (°C)')
            plt.title('Temperatura en Tiempo Real')
            plt.grid(True)  # Añadir cuadrícula al gráfico

            # Escribir en el archivo de registro
            with open("data.log", "a") as file:
                file.write(f"{marca_tiempo} - Temperatura: {grados}°C\n")

        except Exception as e:
            print(f"Error al leer los datos: {e}")

# Configuración del gráfico
plt.ion()
ani = FuncAnimation(plt.gcf(), actualizar, interval=100, cache_frame_data=False)

# Mostrar gráfico
plt.show()

# Mantener el script en ejecución
while True:
    plt.pause(0.5)
