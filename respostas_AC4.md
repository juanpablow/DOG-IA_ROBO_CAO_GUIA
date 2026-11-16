# AC-4 — Respostas às 5 questões técnicas (DOG-IA V2)

Todos os números abaixo foram obtidos executando o código desta pasta (`dog_ia_v2.py`, `teste_seguranca.py`).
Os resultados brutos estão em `resultados/`.

## 1. Arquitetura híbrida (global vs. local)

**Limitação resolvida.** A navegação puramente reativa só enxerga o que está dentro do alcance do LiDAR (165 px) e decide
olhando para o alvo. Isso causa **mínimos locais**: diante de um obstáculo em "U" ou de uma parede entre o robô e o alvo,
virar para o lado "mais livre" e depois voltar a apontar para o alvo gera oscilação ou travamento. Também não existe
noção de custo: o robô não prefere o piso tátil nem sabe que a faixa de pedestres é o único ponto de travessia. O A\* em
grade conhece o mapa inteiro, acha a rota ótima (com custo 0,45 no piso tátil) e passa pela faixa.

**Por que o global sozinho não basta.** O mapa do A\* é uma foto estática do que se sabia no planejamento. Num ambiente urbano
há coisas que ele não vê ou que mudam mais rápido que o replanejamento: veículos, infratores, o semáforo, obstáculos novos
(clique direito), erro de odometria e a tensão do arnês. Por isso o A\* dá a rota e a camada local (LiDAR com filtro,
barreira de meio-fio, máquina de estados do semáforo e freio por zona varrida) garante a segurança no instante.
Se a camada local trava mais de 3,5 s em desvio, ela devolve o controle ao planejador (replanejamento com bloqueio temporário).

## 2. Máquina de estados (viagem 1 — Mercado, sem infratores, `teste_seguranca.py`)

| Tempo | Estado | Condição que causou a transição |
|---|---|---|
| 0,00 s | PLANEJANDO | Destino escolhido; A\* calcula a rota até o Mercado |
| 0,02 s | NAVEGANDO | Rota pronta e via livre: segue o A\* pelo piso tátil |
| 3,53 s | AGUARDANDO_SINAL | Chegou a ≤ 34 px da pista e o semáforo ainda estava VERDE (para carros) |
| 13,12 s | ATRAVESSANDO | Fase PEDESTRE, tempo restante > limite (2,44 s), faixa livre de veículos e tutor junto ao robô |
| 14,78 s | NAVEGANDO | Robô e tutor fora da pista (autorização encerrada) |
| 15,05 s | CHEGOU | Distância ao destino ≤ 22 px |

Nesta viagem não houve DESVIANDO nem FREIO_EMERGENCIA: a rota inflada (C-space) já mantém folga dos obstáculos.
DESVIANDO entra quando o LiDAR vê obstáculo a < 44 px à frente (ou < 26 px de lado); FREIO_EMERGENCIA, quando a zona varrida de um veículo intercepta robô/tutor.

**Diferença AGUARDANDO_SINAL × FREIO_EMERGENCIA.**
- AGUARDANDO_SINAL é uma **parada planejada e regrada**: o robô está na calçada, antes do meio-fio, esperando a fase
  PEDESTRE com tempo suficiente, a faixa livre e o tutor perto. É previsível e sai sozinho quando as condições são atendidas.
- FREIO_EMERGENCIA é uma **parada reativa de segurança**, com prioridade sobre qualquer outro estado (inclusive no meio da
  travessia). Dispara pela previsão de colisão (retângulo do veículo projetado em Δt = 1,0 s intercepta robô ou tutor),
  mantém o freio por 0,4 s depois do risco passar, e conta na métrica de freios.

Teste dirigido (infrator a 170 px/s surgindo com o robô na pista, `resultados/teste_seguranca.txt`): a 400 e 250 px da faixa o
robô conclui a travessia sem freio; a 150 e 90 px ele aciona FREIO_EMERGENCIA, retoma e termina, com 0 colisões.

## 3. Percepção e ruído

**3.1 Mediana × média.** Uma falha do tipo "leitura = alcance máximo" é um *outlier* de valor extremo (165 px contra, por
exemplo, 40 px reais). A média de 3 leituras (40, 165, 40) vira 81 px: o obstáculo "se afasta" 41 px e a leitura continua
errada. A mediana (40) ignora o outlier por completo, desde que ele não ocupe a maioria da janela. Com dropout de 2%,
dois erros na mesma janela do mesmo raio têm chance de cerca de 3 × 0,02² ≈ 0,12%.

**3.2 Custo em atraso.** Com janela de 3, um obstáculo novo só vira a mediana na **2ª varredura** em que aparece (2 de 3).
Em relação à leitura crua, o atraso é de 1 varredura = 1/60 s ≈ **0,017 s** (o obstáculo é confirmado 2/60 ≈ 0,033 s após surgir).

**3.3 Com 15 varreduras.** A mediana só muda quando 8 de 15 leituras concordam, ou seja, 7 varreduras a mais que a leitura
crua: 7/60 ≈ **0,12 s** (8/60 ≈ 0,13 s desde o surgimento). Nesse intervalo o robô anda ≈ 18 px (156 px/s) e um carro a
170 px/s anda ≈ 20 px, quase 40 px de aproximação sem reação. O filtro também deixa o mapa "arrastado" para obstáculos
móveis. O ganho de robustez seria marginal (a janela de 3 já elimina falhas isoladas).
Empiricamente, no benchmark `--janela 15` deu exatamente os mesmos resultados de `--janela 3` (seeds 1 e 7), porque nesse
cenário o desvio reativo quase não é acionado; o custo teórico de 0,12 s, portanto, não aparece nas métricas agregadas.

## 4. Segurança temporal na travessia

**4.1 Limite.** (170 + 24 + 42) / 156 × 1,35 + 0,4 = 236 / 156 × 1,35 + 0,4 = 1,5128 × 1,35 + 0,4 = 2,0423 + 0,4 ≈ **2,44 s**.
(O robô só inicia a travessia se restarem mais de 2,44 s de fase PEDESTRE; o tempo nominal de travessia é 1,51 s,
e o fator 1,35 e os 0,4 s de reserva cobrem lentidão por tensão do arnês e latência.)

**4.2 Fases "todos vermelhos" de 1,5 s.** São o intervalo de limpeza da interseção. Elas eliminam o risco do **conflito entre
quem ainda está terminando o cruzamento e quem recebe o verde**: veículos que entraram no amarelo (ou infratores) ainda estão
dentro/ao redor da faixa quando o sinal vermelho começa, e um pedestre lento ou o tutor que não limpou a pista antes do
fim do tempo seguem cruzando. Sem o intervalo, o verde dos carros abriria sobre uma pista ainda ocupada (e o verde de pedestre
sobre carros ainda em movimento). A faixa tem 95 px + carro de 46 px ≈ 141 px; a 170 px/s isso leva ≈ 0,83 s, menos que 1,5 s.

## 5. Avaliação experimental

Comandos: `python dog_ia_v2.py --benchmark 20 --seed 1` e `--seed 7` (viagens ida ao Mercado/Farmácia/Padaria em rodízio,
saída da Casa, partindo de um instante aleatório do ciclo do semáforo).

**5.1 Resultados**

| | Seed 1 | Seed 7 |
|---|---|---|
| Taxa de sucesso | 100% (20/20) | 100% (20/20) |
| Tempo médio (viagem) | 17,06 s (6,77 a 23,18 s) | 11,81 s (5,47 a 20,33 s) |
| Colisões | 0 | 0 |
| Quase-colisões (< 25 px de veículo) | 0 | 0 |
| Freios de emergência | 1 (viagem 16) | 0 |
| Replanejamentos | 0 | 0 |

**5.2 Seed fixa e diferença entre seeds.** A seed inicializa o gerador pseudoaleatório (`random.Random(seed)`) usado para
tráfego, infratores, ruído/dropout do LiDAR e instante de chegada ao ciclo do semáforo. A mesma seed produz exatamente a
mesma sequência de números, logo a mesma simulação: rodando `--seed 1` duas vezes os JSONs saíram idênticos (verificado com `cmp`).
Seeds diferentes produzem sequências diferentes: outro tráfego, outro ruído e, principalmente, outro ponto do ciclo do
semáforo em que o robô chega ao meio-fio. O tempo médio é dominado pela espera pela fase PEDESTRE (até 14,5 s),
por isso as médias diferem (17,06 s contra 11,81 s) mesmo com 100% de sucesso.

**5.3 Melhoria proposta: aumentar a margem de segurança dos obstáculos inflados (solo 30 → 38 px, suspensos 38 → 46 px).**
Esperado: melhorar a folga ao tutor/cabeça (menos quase-colisões com obstáculos e menos desvios reativos); piorar o
tempo e a distância percorrida (rota mais longa) e a chance de A\* não achar rota em corredores estreitos (o corredor sob a
banca tem cerca de 48 px).
Medido (`--infl-solo 38 --infl-susp 46`): seed 1 → 12,12 s, 0 colisões, 0 freios; seed 7 → 13,46 s, 0 colisões, 0 freios.
**Conclusão honesta:** com colisões e quase-colisões já em 0 nos dois casos base, não há ganho de segurança mensurável neste
cenário, e a variação de tempo (menor na seed 1, maior na seed 7) vem sobretudo do alinhamento com o semáforo (o tempo muda quando a rota muda
o instante de chegada ao meio-fio), não de efeito direto da margem. Para provar a hipótese seria preciso um cenário mais
denso em obstáculos e mais viagens por seed.

## Observações e limitações

- Cenário V1 ajustado: poste movido para (470, 110), toldo para (470, 590) e dois prédios adicionados, porque na V1 o toldo
  inflado (38 px) bloqueava o destino e o piso tátil inferior.
- A passagem da faixa no planejador foi restrita à região central (x 615 a 640) para o robô e o tutor não passarem rente aos carros parados.
- Veículos têm frenagem automática por pedestre à frente (com tempo de reação de 0,4 s); infratores só ignoram o semáforo.
- A etapa bônus (vídeo, LinkedIn, e-mail ao professor) depende do grupo e não foi feita aqui.
