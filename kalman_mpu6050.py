from math import *
import random
import matplotlib.pyplot as plt

amostras = 250
dt = 0.02

class MPU6050_simulation:
    def __init__(self, accel_noise=0.03, gyro_noise=0.5):
        self.accel_offset = (0.02, -0.015, 0.03)         # Aceleração (g)
        self.gyro_offset = (-1.2, 0.8, -0.3)             # Giroscópio (°/s)
        
        self.accel_noise = accel_noise
        self.gyro_noise = gyro_noise

        self.amostragem = []
        self.theta_real_list = []
        self.ax_list, self.ay_list, self.az_list = [], [], []
        self.gx_list, self.gy_list, self.gz_list = [], [], []
    
    def add_noise(self, values, std_noise):
        return values + random.gauss(0, std_noise)
    
    def get_data(self, t):
        theta_rad = 0.2 * sin(2 * t)
        omega_deg = degrees(0.4 * cos(2 * t))

        # Acelerômetro:
        ax = self.add_noise(sin(theta_rad) + self.accel_offset[0], self.accel_noise)
        ay = self.add_noise(0.0 + self.accel_offset[1], self.accel_noise)
        az = self.add_noise(cos(theta_rad) + self.accel_offset[2], self.accel_noise)

        # Giroscópio:
        gx = self.add_noise(0.0 + self.gyro_offset[0], self.gyro_noise)
        gy = self.add_noise(omega_deg + self.gyro_offset[1], self.gyro_noise)
        gz = self.add_noise(0.0 + self.gyro_offset[2], self.gyro_noise)

        # Tempo decorrido e referência real
        self.amostragem.append(t)
        self.theta_real_list.append(degrees(theta_rad))

        # Armazenamento das leituras
        self.ax_list.append(ax)
        self.ay_list.append(ay)
        self.az_list.append(az)
        self.gx_list.append(gx)
        self.gy_list.append(gy)
        self.gz_list.append(gz)

        return {
            'accel': (ax, ay, az),
            'gyro':  (gx, gy, gz)
        }


class Filter_Kalman1D:
    def __init__(self, theta_Q=0.001, bias_Q=0.003, measure_noise=0.03):
        #Covariância do Ruído do Processo
        self.theta_Q = theta_Q
        self.bias_Q = bias_Q

        #Covariância do Ruído da Medição
        self.measure_noise = measure_noise

        self.angle = 0.0
        self.bias = 0.0

        #Covariância do Erro de Estimação
        self.P = [[0.0, 0.0], 
                  [0.0, 0.0]]

    def update(self, new_theta_accel, new_omega_gyro, dt):
        # Etapa 1 - Predição:
        rate = new_omega_gyro - self.bias
        self.angle += dt * rate

        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.theta_Q)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.bias_Q * dt

        # Etapa 2 - Atualização:
        z = new_theta_accel - self.angle
        S = self.P[0][0] + self.measure_noise

        k0 = self.P[0][0] / S
        k1 = self.P[1][0] / S

        self.angle += k0 * z
        self.bias += k1 * z

        mem_P00 = self.P[0][0]
        mem_P01 = self.P[0][1]

        self.P[0][0] -= k0 * mem_P00
        self.P[0][1] -= k0 * mem_P01
        self.P[1][0] -= k1 * mem_P00
        self.P[1][1] -= k1 * mem_P01

        return self.angle


def graphics(sim_obj, accel_angles, kalman_angles, err_gross, err_kalman):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    # 1. Acelerômetro
    ax1.plot(sim_obj.amostragem, sim_obj.ax_list, label='Ax (g)')
    ax1.plot(sim_obj.amostragem, sim_obj.ay_list, label='Ay (g)')
    ax1.plot(sim_obj.amostragem, sim_obj.az_list, label='Az (g)')
    ax1.set_title('Simulação MPU6050 e Fusão Sensorial via Filtro de Kalman 1D')
    ax1.set_ylabel('Aceleração (g)')
    ax1.legend(loc='upper right')
    ax1.grid(True)

    # 2. Giroscópio
    ax2.plot(sim_obj.amostragem, sim_obj.gx_list, label='Gx (°/s)')
    ax2.plot(sim_obj.amostragem, sim_obj.gy_list, label='Gy (°/s)')
    ax2.plot(sim_obj.amostragem, sim_obj.gz_list, label='Gz (°/s)')
    ax2.set_ylabel('Vel. Angular (°/s)')
    ax2.legend(loc='upper right')
    ax2.grid(True)

    # 3. Estimação de Ângulo (Filtro de Kalman)
    ax3.plot(sim_obj.amostragem, accel_angles, label=f'Acelerômetro Bruto (RMSE: {err_gross:.2f}°)', alpha=0.5, color='gray')
    ax3.plot(sim_obj.amostragem, sim_obj.theta_real_list, label='Ângulo Real (Ground Truth)', linestyle='--', color='black', linewidth=2)
    ax3.plot(sim_obj.amostragem, kalman_angles, label=f'Filtro de Kalman (RMSE: {err_kalman:.2f}°)', color='blue', linewidth=2)
    ax3.set_xlabel('Tempo (s)')
    ax3.set_ylabel('Ângulo (°)')
    ax3.legend(loc='upper right')
    ax3.grid(True)

    plt.tight_layout()
    plt.savefig('resultado_completo_kalman_mpu6050.png', dpi=150)
    plt.show()

if __name__ == "__main__":
    output = MPU6050_simulation()
    kalman_filter = Filter_Kalman1D()
    accel_angle_list, angle_filtered_list = [], []

    for k in range(amostras):
        t = k * dt
        data = output.get_data(t)

        ax, ay, az = data['accel']
        gx, gy, gz = data['gyro']

        accel_angle = degrees(atan2(ax, az))
        accel_angle_list.append(accel_angle)

        angle_filtered = kalman_filter.update(accel_angle, gy, dt)
        angle_filtered_list.append(angle_filtered)

    gross_error = sqrt(sum((a - r)**2 for a, r in zip(accel_angle_list, output.theta_real_list)) / amostras)
    kalman_error = sqrt(sum((kf - r)**2 for kf, r in zip(angle_filtered_list, output.theta_real_list)) / amostras)

    graphics(output, accel_angle_list, angle_filtered_list, gross_error, kalman_error)
