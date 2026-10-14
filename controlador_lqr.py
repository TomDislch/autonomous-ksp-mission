import numpy as np
import cvxpy as cp

class ControladorLQR:
    def __init__(self, dt, g, margem_seguranca, N):
        self.dt = dt
        self.g = g
        self.margem_seguranca = margem_seguranca
        self.N = N

        # Dinâmica do Sistema (Matriz A)
        self.A_mpc = np.eye(6)
        self.A_mpc[0, 3] = self.dt
        self.A_mpc[1, 4] = self.dt
        self.A_mpc[2, 5] = self.dt

        # Dinâmica dos Atuadores (Matriz B)
        self.B_mpc = np.zeros((6, 3))
        self.B_mpc[3, 0] = self.dt
        self.B_mpc[4, 1] = self.dt
        self.B_mpc[5, 2] = self.dt
        self.B_mpc[0, 0] = 0.5 * (self.dt**2) 
        self.B_mpc[1, 1] = 0.5 * (self.dt**2)
        self.B_mpc[2, 2] = 0.5 * (self.dt**2)

        # Distúrbios Constantes (Vetor G)
        self.G_mpc = np.array([
            0.0, 
            0.0, 
            -0.5 * self.g * (self.dt**2), 
            0.0, 
            0.0, 
            -self.g * self.dt
        ])

        # Parâmetros e Variáveis do Otimizador
        self.estado_inicial_mpc = cp.Parameter(6)
        self.limite_aceleracao = cp.Parameter(nonneg=True)

        X_otim = cp.Variable((6, self.N+1))
        self.U = cp.Variable((3, self.N)) 

        restricoes = [X_otim[:, 0] == self.estado_inicial_mpc]
        
        Q_pesos = np.array([10.0, 10.0, 300.0, 10.0, 10.0, 800.0]) 
        Q_matrix = np.diag(Q_pesos)

        R_pesos = np.array([2.0, 2.0, 2.0])
        R_matrix = np.diag(R_pesos)

        Q_terminal = Q_matrix * 100

        custo = 0
        tan_max = np.tan(np.radians(7.0)) 

        for k in range(self.N):
            custo += cp.quad_form(X_otim[:, k], Q_matrix) + cp.quad_form(self.U[:, k], R_matrix)
            restricoes.append(X_otim[:, k+1] == self.A_mpc @ X_otim[:, k] + self.B_mpc @ self.U[:, k] + self.G_mpc)
            
            restricoes.append(self.U[2, k] >= 0)                      
            restricoes.append(cp.norm(self.U[:, k], 2) <= self.limite_aceleracao)      
            restricoes.append(cp.norm(self.U[:2, k], 2) <= tan_max * self.U[2, k]) 

        custo += cp.quad_form(X_otim[:, self.N], Q_terminal)

        self.problema_mpc = cp.Problem(cp.Minimize(custo), restricoes)
        print("Cérebro compilado com sucesso!") 

    def calcula_acao(self, altitude_real, vel_x, vel_y, vel_z, a_max):
        # Restaurado para a altitude real
        altitude_virtual = altitude_real - self.margem_seguranca
        
        self.estado_inicial_mpc.value = np.array([0.0, 0.0, altitude_virtual, vel_x, vel_y, vel_z])
        self.limite_aceleracao.value = a_max

        try:
            # O warm_start voltou! O cérebro agora tem "memória" do frame anterior (acaba com o Bang-Bang)
            self.problema_mpc.solve(warm_start=True)
        except Exception as e:
            pass

        if self.U[:, 0].value is None:
            if vel_z < -5.0:
                return np.array([0.0, 0.0, a_max])
            return np.array([0.0, 0.0, 0.0])
        
        retorno = self.U[:, 0].value
        retorno[2] = max(0.0, retorno[2])
        return retorno