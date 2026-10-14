import time

def colocar_em_orbita(nave):
    print("\n--- INICIANDO ROTINA DE CIRCULARIZAÇÃO ---")

    nave.aplicar_comando_orbital(pitch=0.0, yaw=90.0, acelerador=0.0)
    gatilho_ignicao=nave.tempo_ignicao()
    fase_queima=False
    while True:
        sensores=nave.ler_sensores()
        t_apogeu=sensores["t_apogeu"]
        alt_perigeu=sensores["alt_perigeu"]

        print(f"[DEBUG ÓRBITA] T. Apogeu: {t_apogeu:5.1f}s | Perigeu Atual: {alt_perigeu/1000:6.1f} km")

        if alt_perigeu>70000:
            nave.aplicar_comando_orbital(pitch=0.0, yaw=90.0, acelerador=0.0)
            print(f"MECO! Órbita circularizada com Perigeu de {alt_perigeu/1000:.1f} km.")
            break

        if t_apogeu<=gatilho_ignicao and not fase_queima:
            fase_queima=True
        elif not fase_queima:
            print(f"[COASTING] Esperando chegar no topo... T. Apogeu: {t_apogeu:.1f}s")
        if fase_queima:
            nave.aplicar_comando_orbital(pitch=0.0, yaw=90.0, acelerador=1.0)
        else:
            nave.aplicar_comando_orbital(pitch=0.0, yaw=90.0, acelerador=0.0)
            
        time.sleep(0.5)
