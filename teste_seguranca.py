"""Cenarios dirigidos: sequencia de estados da viagem ao Mercado e freio de emergencia."""
import dog_ia_v2 as d


def viagem_mercado():
    sim = d.Simulacao(d.Config(prob_infrator=0.0), 1)
    sim.iniciar_viagem([1])
    while not sim.viagem_concluida and sim.t < 120:
        sim.passo(d.DT_FIXO)
    print("== Viagem ao Mercado (sem infratores) ==")
    for t, estado, motivo in sim.robo.historico:
        print(f"{t:6.2f}s  {estado:<17} {motivo}")
    print(f"tempo total: {sim.t_viagem:.2f}s")


def infrator_na_travessia(dist_x):
    sim = d.Simulacao(d.Config(prob_infrator=0.0), 1)
    sim.iniciar_viagem([1])
    while sim.robo.estado != d.ATRAVESSANDO and sim.t < 60:
        sim.passo(d.DT_FIXO)
    while sim.robo.y < d.RUA_Y + 20:
        sim.passo(d.DT_FIXO)
    v = sim.trafego.spawn(0, infrator=True, forcar_x=d.FAIXA_X - dist_x)
    v.vmax = v.v = 170.0
    estados = []
    for _ in range(300):
        sim.passo(d.DT_FIXO)
        if not estados or estados[-1] != sim.robo.estado:
            estados.append(sim.robo.estado)
    sim.fechar_episodios()
    print(f"infrator a {dist_x}px da faixa: estados={estados} freios={sim.m.freios} "
          f"colisoes={sim.m.colisoes} quase={sim.m.quase_colisoes}")


if __name__ == "__main__":
    viagem_mercado()
    print("\n== Infrator surgindo com o robo na pista ==")
    for dx in (400, 250, 150, 90):
        infrator_na_travessia(dx)
