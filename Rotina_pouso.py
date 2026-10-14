from controlador_lqr import ControladorLQR
import time

def executar_pouso(nave):
    print("Iniciando rotina de pouso...")
    cerebro = ControladorLQR(dt=0.2, g=nave.g, margem_seguranca=nave.margem_seguranca, N=50)
    modo_pouso_ativado = False
    
        
    while True:
        # A. LER SENSORES
        massa, thrust_max, alt_radar, v_cima, v_norte, v_leste = nave.ler_sensores()
        
        # B. CALCULAR LIMITES FÍSICOS
        a_max = thrust_max / massa
        aceleracao_liquida = a_max - nave.g
        
        # C. CALCULAR PONTO DE IGNICAO
        if aceleracao_liquida > 0.1:
            dist_frenagem = (v_cima**2) / (2 * aceleracao_liquida)
        else:
            dist_frenagem = 99999
            
        ponto_ignicao = dist_frenagem + 300.0 
        
        # ==========================================
        # D. A MÁQUINA DE ESTADOS COM TRAVA (LATCH)
        # ==========================================
        
        # Se cruzou a linha de perigo, puxa a alavanca de trava para sempre
        if alt_radar <= ponto_ignicao:
            modo_pouso_ativado = True
        
        if v_cima > 0.5 and not modo_pouso_ativado:
            nave.aplicar_comando([0.0, 0.0, 0.0], massa, thrust_max)
            print(f"Subindo (Balística) | Alt: {alt_radar:5.1f}m | Vz: {v_cima:5.1f}m/s")
            
        elif not modo_pouso_ativado:
            nave.aplicar_comando([0.0, 0.0, 0.0], massa, thrust_max)
            print(f"Queda Livre         | Alt: {alt_radar:5.1f}m | Vz: {v_cima:5.1f}m/s | Ignição em: {ponto_ignicao:5.1f}m")
            
        else:
            # Pouso Travado! Nunca mais volta para cima.
            comando_A = cerebro.calcula_acao(alt_radar, v_leste, v_norte, v_cima, a_max)
            nave.aplicar_comando(comando_A, massa, thrust_max)
            print(f"Pouso (LQR 3D)      | Alt: {alt_radar:5.1f}m | Vz: {v_cima:5.1f}m/s")

        # Se estiver a meio metro do chão e com velocidade de quique aceitável, corta tudo!
        if alt_radar < (nave.margem_seguranca + 0.5) and abs(v_cima) < 2.0:
            print("POUSO CONFIRMADO! Desligando sistemas.")
            nave.aplicar_comando([0.0, 0.0, 0.0], massa, thrust_max)
            nave.nave.auto_pilot.engaged = False 
            break    

        # Espera 0.02s para fechar o ciclo de 50 Hertz do KSP
        time.sleep(0.02)