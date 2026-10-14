from hardware_ksp import Hardware
from lancamento import executar_lancamento
from circularizacao import colocar_em_orbita
from transf_mun import ir_ate_mun
minha_nave=Hardware()
executar_lancamento(minha_nave)
colocar_em_orbita(minha_nave)
ir_ate_mun(minha_nave)
