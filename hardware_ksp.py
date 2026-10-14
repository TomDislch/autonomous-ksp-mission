import numpy as np
import krpc

class Hardware:
    def __init__(self):
        print("Conectando ao Kerbal Space Program...")
        self.conexao = krpc.connect(name='HAL - KSP')
        self.nave = self.conexao.space_center.active_vessel
        
        # Parâmetros Físicos Iniciais
        self.g = self.nave.orbit.body.surface_gravity
        caixa = self.nave.bounding_box(self.nave.reference_frame)
        self.margem_seguranca = abs(caixa[0][1]) + 0.1 # +10cm de folga da perna
        
        # Referenciais
        self.ref_planeta = self.nave.orbit.body.reference_frame
        self.ref_superficie = self.nave.surface_reference_frame
        
        # Sensores (Streams de Telemetria)
        self.telemetria_planeta = self.nave.flight(self.ref_planeta)
        self.telemetria_superficie = self.nave.flight(self.ref_superficie)
        
        # Atuadores (Piloto Automático)
        self.nave.auto_pilot.reference_frame = self.ref_superficie
        self.nave.auto_pilot.engaged = True
        self.nave.auto_pilot.target_roll = 0.0

        #Dados para orbita
        self.tempo_ate_apogeu=self.nave.orbit.time_to_apoapsis
        self.altura_apogeu= self.nave.orbit.apoapsis_altitude

        #Dados sobre mun
        mun = self.conexao.space_center.bodies['Mun']
        raio_mun=mun.orbit.semi_major_axis
        periodo_mun=mun.orbit.period

        print(f"Hardware inicializado! Nave: {self.nave.name} | Margem: {self.margem_seguranca:.2f}m")

    def ler_sensores(self):
        #Dados da nave
        massa = self.nave.mass
        thrust_max = self.nave.available_thrust
        alt_radar = self.telemetria_superficie.surface_altitude
        vel = self.telemetria_planeta.velocity
        vel_local = self.conexao.space_center.transform_direction(vel, self.ref_planeta, self.ref_superficie)
        pitch_real = self.telemetria_superficie.pitch
        
        #Dados para orbita
        tempo_ate_apogeu=self.nave.orbit.time_to_apoapsis
        altura_apogeu= self.nave.orbit.apoapsis_altitude
        raio_apogeu=self.nave.orbit.apoapsis
        alt_perigeu=self.nave.orbit.periapsis_altitude        
        raio_planeta=self.nave.orbit.body.equatorial_radius

        #Dados astronomicos
        kerbin = self.conexao.space_center.bodies['Kerbin']
        ref_frame = kerbin.non_rotating_reference_frame
        mun = self.conexao.space_center.bodies['Mun']
        pos_nave = self.nave.position(ref_frame)
        pos_mun = mun.position(ref_frame)

        angulo_nave = np.atan2(pos_nave[2], pos_nave[0])
        angulo_mun = np.atan2(pos_mun[2], pos_mun[0])
        #a variável fase_atual devolve o angulo RELATIVO entre a nave e mun
        fase_atual_rad = (angulo_mun - angulo_nave) % (2 * np.pi)
        return {
            "massa": massa,
            "empuxo": thrust_max,
            "alt_radar": alt_radar,
            "v_cima": vel_local[0],
            "v_norte": vel_local[1],
            "v_leste": vel_local[2],
            "t_apogeu": tempo_ate_apogeu,
            "alt_apogeu": altura_apogeu,
            "raio_apogeu": raio_apogeu,            
            "pitch_real": pitch_real,
            "alt_perigeu": alt_perigeu,
            "raio_planeta": raio_planeta,
            "fase_mun_rad": fase_atual_rad
        }
    def aplicar_comando(self, comando_A, massa_atual, thrust_max):
        Forca = np.linalg.norm(comando_A) * massa_atual

        porcentagem = np.clip(Forca / thrust_max, 0.0, 1.0)
        self.nave.control.throttle = porcentagem

        if Forca > 1.0:
            # O MPC devolve [Leste, Norte, Cima], o KSP espera (Cima, Norte, Leste)
            u_leste = comando_A[0]
            u_norte = comando_A[1]
            u_cima = comando_A[2]
            self.nave.auto_pilot.target_direction = (u_cima, u_norte, u_leste)
        else:
            self.nave.auto_pilot.target_direction = (1.0, 0.0, 0.0)

    def aplicar_comando_orbital(self, pitch, yaw, acelerador):
        # Controle do motor
        self.nave.control.throttle = acelerador
        
        # Controle de direção por graus 
        self.nave.auto_pilot.target_pitch_and_heading(pitch, yaw)
        self.nave.auto_pilot.target_roll = 0.0

    def tempo_ignicao(self):
        massa = self.nave.mass
        thrust_max = self.nave.available_thrust
        mi_kerbin=self.nave.orbit.body.gravitational_parameter
        raio_apogeu=self.nave.orbit.apoapsis
        ut_atual = self.conexao.space_center.ut
        tempo_ate_apogeu=self.nave.orbit.time_to_apoapsis


        velocidade_topo=self.nave.orbit.orbital_speed_at(ut_atual + tempo_ate_apogeu)
        velocidade_alvo=np.sqrt(mi_kerbin/raio_apogeu)
        delta_v=velocidade_alvo-velocidade_topo
        if thrust_max==0:
            return 0.0
        a=thrust_max/massa
        tempo_queima=delta_v/a
        tempo_ignicao=tempo_queima/2
        return(tempo_ignicao)

    def obter_dados_astronomicos(self):
        """Busca constantes físicas dos astros apenas uma vez"""
        mun = self.conexao.space_center.bodies['Mun']
        kerbin = self.conexao.space_center.bodies['Kerbin']
        
        return {
            "mi_kerbin": kerbin.gravitational_parameter,
            "raio_mun": mun.orbit.semi_major_axis,
            "periodo_mun": mun.orbit.period
        }