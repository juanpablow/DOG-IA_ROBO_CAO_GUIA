# DOG-IA V2 — Cão-guia robótico autônomo (AC-4)

Simulador urbano em Python/pygame (A\* + C-Space, LiDAR com ruído e filtro de mediana, semáforo com All-Red,
tráfego com infratores, tutor acoplado, áudio estéreo e benchmark headless).

## Execução
```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python dog_ia_v2.py                          # interface gráfica
.venv/bin/python dog_ia_v2.py --benchmark 20 --seed 1  # headless
.venv/bin/python teste_seguranca.py                    # cenários dirigidos
```
Controles: `1-4` destinos (Mercado, Farmácia, Padaria, Casa), clique esq. novo destino, clique dir. obstáculo,
`V` veículo infrator, `G` C-Space e rota, `M` áudio, `ESPAÇO` manual/IA, `R` reiniciar.

## Conteúdo
- `dog_ia_v1.py` — código base (V1); `roteiro_aula_AC4-FINAL.md` — roteiro da aula.
- `dog_ia_v2.py` — V2 (Marcos 1 a 4).
- `respostas_AC4.md` — respostas às 5 questões técnicas.
- `resultados/` — saídas dos benchmarks e dos cenários dirigidos.

Observação: usa `pygame-ce`, pois o `pygame` 2.6.1 não traz os módulos `font` e `mixer` no Python 3.14.
