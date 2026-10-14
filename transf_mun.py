import time
import numpy as np
def ir_ate_mun(nave):
    fase_queima=False
    queimou=False

    sensores=nave.ler_sensores()
    astronomia=nave.obter_dados_astronomicos()

    fase_atual_radianos=sensores["fase_atual_radianos"]
    raio_apogeu=sensores["raio_apogeu"]
    mi_kerbin=astronomia["mi_kerbin"]
    raio_mun=astronomia["raio_mun"]
    periodo_mun=astronomia["periodo_mun"]

    eixo_semimaior=(raio_apogeu+raio_mun)/2
    tempo_voo=np.pi *np.sqrt((eixo_semimaior**3)/mi_kerbin)
    vel_angular_mun = (2 * np.pi) / periodo_mun
    angulo_percorrido_mun = vel_angular_mun * tempo_voo

    fase_alvo_radianos = np.pi - angulo_percorrido_mun
    fase_alvo_graus = np.degrees(fase_alvo_radianos)

    nave.aplicar_comando_orbital(0.0, 90.0, 0.0)
    print("\n[ESPERA] Procurando janela de transferência lunar...")

    while True:
        sensores=nave.ler_sensores()
        fase_atual_radianos=sensores["fase_atual_radianos"]
        raio_apogeu=sensores["raio_apogeu"]

        print(f"Fase Atual: {np.degrees(fase_atual_radianos):.1f}º | Alvo: {fase_alvo_graus:.1f}º")
        erro_fase = abs(fase_atual_radianos - fase_alvo_radianos)

        if erro_fase <= 0.02 and not fase_queima:
            print("\n[IGNIÇÃO] Janela atingida! Iniciando Injeção Trans-Munar (TMI)!")
            fase_queima=True

        if fase_queima and not queimou:
            nave.aplicar_comando_orbital(0.0,90,1.0)
            queimou=True

        if raio_apogeu>=11400000:
            nave.aplicar_comando_orbital(0.0,0.0,0.0)
            print("MECO! Transferência concluída. Bem-vindo ao espaço profundo.")
            break

        time.sleep(0.5)
