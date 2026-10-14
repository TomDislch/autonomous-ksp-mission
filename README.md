#  KSP Autonomous Flight Controller
Autonomous flight controller and orbital mechanics calculator for KSP using Python and kRPC.


##  Module Architecture

The system is modularized to separate the hardware abstraction from the control mathematics:

- **`hardware_ksp.py`**: Hardware Abstraction Layer (HAL). Manages telemetry, converts coordinates to non-rotating reference frames, and extracts astronomical constants.

- **`lancamento.py`**: Atmospheric ascent control executing an optimized Gravity Turn.

- **`circularizacao.py`**: State machine that calculates the exact ignition time (based on thrust, mass, and $\Delta V$) to circularize the orbit at apoapsis.

- **`transf_mun.py`**: Translunar injection algorithm. Monitors phase angles and calculates the Time of Flight ($ToF$) for an autonomous Hohmann Transfer.

- **`Rotina_pouso.py`**: Control algorithm for descent and braking burn.

- **`controlador_lqr.py`**: Implementation of a Linear Quadratic Regulator (LQR) for active stabilization.

- **`main_missao.py`**: Mission orchestrator that manages the continuous flow of mission states.


##  How to Run
1. Start Kerbal Space Program and enable the kRPC server.

2. Ensure you have the required libraries installed: `pip install krpc numpy`
3. Run the main module in the terminal:
```bash
python3 main_missao.py
