import time
import math

def executar_lancamento(nave):
    alvo_apogeu=45.0
    dt_controle=0.2

    
    nave.nave.control.activate_next_stage()

    while True:
        
        sensores = nave.ler_sensores()
        
        thrust_max = sensores["empuxo"]
        alt_radar  = sensores["alt_radar"]
        v_cima     = sensores["v_cima"]
        alt_apogeu = sensores["alt_apogeu"]
        pitch_real = sensores["pitch_real"]
        v_leste    = sensores["v_leste"]
        
        if alt_radar<12000 and v_cima>280:
            acelerador=0.6
        else:
            acelerador=1

        #MAQUINA DE ESTADOS

        theta=math.degrees(math.atan2(v_cima,v_leste))

        #Fase 4: Apogeu no espaço
        if alt_apogeu> 75000:
            print("Apogeu no espaço! Cortando motores.")
            nave.aplicar_comando_orbital(0.0,90.0,0.0)
            break

        #Fase 1: Subida vertical
        elif v_cima<100:
            nave.aplicar_comando_orbital(90.0,90.0,acelerador)

        #Fase 2: Pitch ideal
        elif theta>85:
            nave.aplicar_comando_orbital(85,90.0,acelerador)
            print(f"[KICK] FOrçando 85° | Pitch REAL: {pitch_real:4.1f}°")

        #Fase 3
        else:
            nave.aplicar_comando_orbital(theta,90.0,acelerador)    
            print(f"[GRAVITY TURN] Seguindo Prograde: {theta:4.1f}° | Real: {pitch_real:4.1f}°")
        
        #Auto-Staging
        if thrust_max==0:
            nave.nave.control.activate_next_stage()
            time.sleep(1)
        time.sleep(dt_controle)

            

