"""DOG-IA V2 - simulador urbano de cao-guia autonomo (AC-4).

Uso:
    python dog_ia_v2.py                         # interface grafica
    python dog_ia_v2.py --benchmark 20 --seed 1 # execucao headless
"""
import argparse
import heapq
import json
import math
import os
import random
import sys
from array import array
from collections import deque
from dataclasses import asdict, dataclass

if "--benchmark" in sys.argv:
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame  # noqa: E402

# ==================== GEOMETRIA E CONSTANTES ====================
LARGURA, ALTURA = 1050, 680
FPS = 60
AREA_W = LARGURA - 260
RUA_Y, RUA_H = 250, 170
RUA_FIM = RUA_Y + RUA_H
FAIXA_X, FAIXA_W = 580, 95
CELULA = 10
COLS, LINHAS = AREA_W // CELULA, ALTURA // CELULA

VEL_ROBO = 156.0
VEL_TUTOR = 150.0
OMEGA = 3.5
GUIA = 42
FOLGA_TENSAO = 10
TENSAO_MAX = 30
RAIO_PLAN = 20
RAIO_COL = 17
RAIO_TUTOR = 12
FOLGA_TRAVESSIA = 24
MARGEM_BARREIRA = 24
ESPERA_DIST = 34
DT_FIXO = 1.0 / 60.0

VEL_VEICULO_MAX = (140.0, 175.0)
ACEL = 120.0
DESACEL = 260.0
COMP_VEI, LARG_VEI = 46, 24
LINHA_RETENCAO = 16
FAIXAS_Y = (RUA_Y + 43, RUA_FIM - 43)

BRANCO = (245, 245, 245)
PRETO = (20, 22, 28)
ASFALTO = (42, 45, 52)
CALCADA = (130, 134, 142)
MEIO_FIO = (175, 180, 188)
PISO_TATIL = (235, 190, 20)
GRAMA = (46, 115, 65)
CORPO_ROBO = (30, 136, 229)
DETALHE_ROBO = (15, 76, 129)
CABECA_ROBO = (230, 236, 245)
ARNES_GUIA = (255, 215, 0)
VERDE = (46, 204, 113)
LARANJA = (255, 140, 0)
VERMELHO = (225, 55, 55)
AMARELO_COR = (245, 205, 40)
ROXO = (155, 89, 182)

TATIL_RECTS = [
    pygame.Rect(40, 120, 595, 14),
    pygame.Rect(621, 120, 14, 130),
    pygame.Rect(621, RUA_FIM, 14, 130),
    pygame.Rect(150, 536, 485, 14),
]
PAREDES = [
    pygame.Rect(0, 0, AREA_W, 15),
    pygame.Rect(0, ALTURA - 15, AREA_W, 15),
    pygame.Rect(0, 0, 15, ALTURA),
    pygame.Rect(AREA_W - 15, 0, 15, ALTURA),
]
CASA = (95, 127)
DESTINOS = {
    1: ("Mercado", (628, 543)),
    2: ("Farmácia", (230, 543)),
    3: ("Padaria", (730, 543)),
    4: ("Casa", CASA),
}

# Estados da maquina de estados do robo
OCIOSO = "OCIOSO"
PLANEJANDO = "PLANEJANDO"
NAVEGANDO = "NAVEGANDO"
DESVIANDO = "DESVIANDO"
AGUARDANDO = "AGUARDANDO_SINAL"
ATRAVESSANDO = "ATRAVESSANDO"
FREIO = "FREIO_EMERGENCIA"
CHEGOU = "CHEGOU"
SEM_ROTA = "SEM_ROTA"


@dataclass
class Config:
    infl_solo: int = 30
    infl_edif: int = 22
    infl_susp: int = 38
    custo_tatil: float = 0.45
    janela_mediana: int = 3
    ruido_sigma: float = 1.5
    dropout: float = 0.02
    horizonte_s: float = 1.0
    prob_infrator: float = 0.12
    timeout_viagem: float = 120.0


def limite_travessia(vel=VEL_ROBO):
    return (RUA_H + FOLGA_TRAVESSIA + GUIA) / vel * 1.35 + 0.4


def dist_ponto_rect(px, py, r):
    dx = max(r.left - px, 0, px - r.right)
    dy = max(r.top - py, 0, py - r.bottom)
    return math.hypot(dx, dy)


def normalizar_angulo(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rotacionar_ponto(cx, cy, px, py, angulo):
    s, c = math.sin(angulo), math.cos(angulo)
    dx, dy = px - cx, py - cy
    return (int(cx + dx * c - dy * s), int(cy + dx * s + dy * c))


def criar_cenario():
    return [
        {"rect": pygame.Rect(230, 85, 45, 75), "tipo": "solo", "nome": "Banca / Hidrante"},
        {"rect": pygame.Rect(470, 110, 32, 32), "tipo": "solo", "nome": "Poste de Luz"},
        {"rect": pygame.Rect(380, 460, 40, 40), "tipo": "solo", "nome": "Lixeira Urbana"},
        {"rect": pygame.Rect(350, 75, 85, 65), "tipo": "suspenso", "nome": "Copa de Árvore"},
        {"rect": pygame.Rect(470, 590, 90, 45), "tipo": "suspenso", "nome": "Toldo de Loja"},
        {"rect": pygame.Rect(690, 60, 85, 95), "tipo": "edificacao", "nome": "Prédio"},
        {"rect": pygame.Rect(40, 585, 110, 55), "tipo": "edificacao", "nome": "Prédio"},
    ]


# ==================== PLANEJAMENTO GLOBAL (A* + C-SPACE) ====================
class GradeOcupacao:
    """Grade de ocupacao com espaco de configuracao (obstaculos inflados)."""

    def __init__(self, obstaculos, cfg, bloqueios=()):
        self.bloq = [[False] * COLS for _ in range(LINHAS)]
        self.custo = [[1.0] * COLS for _ in range(LINHAS)]
        self.cfg = cfg
        infl = {
            "solo": cfg.infl_solo,
            "edificacao": cfg.infl_edif,
            "parede": cfg.infl_edif,
            "suspenso": cfg.infl_susp,
        }
        fontes = [(o["rect"], infl.get(o["tipo"], cfg.infl_solo)) for o in obstaculos]
        fontes += [(p, infl["parede"]) for p in PAREDES]
        for r in range(LINHAS):
            for c in range(COLS):
                cx, cy = c * CELULA + CELULA // 2, r * CELULA + CELULA // 2
                if any(dist_ponto_rect(cx, cy, rect) <= i for rect, i in fontes):
                    self.bloq[r][c] = True
                    continue
                if any(math.hypot(cx - bx, cy - by) <= br for bx, by, br in bloqueios):
                    self.bloq[r][c] = True
                    continue
                if RUA_Y - 12 < cy < RUA_FIM + 12 and not (FAIXA_X + 35 <= cx <= FAIXA_X + 60):
                    self.bloq[r][c] = True
                    continue
                if any(t.collidepoint(cx, cy) for t in TATIL_RECTS):
                    self.custo[r][c] = cfg.custo_tatil

    def celula(self, x, y):
        return (min(max(int(x // CELULA), 0), COLS - 1), min(max(int(y // CELULA), 0), LINHAS - 1))

    def livre(self, c, r):
        return 0 <= c < COLS and 0 <= r < LINHAS and not self.bloq[r][c]

    def mais_proxima_livre(self, c, r):
        if self.livre(c, r):
            return (c, r)
        for raio in range(1, 8):
            melhor = None
            for dc in range(-raio, raio + 1):
                for dr in range(-raio, raio + 1):
                    if max(abs(dc), abs(dr)) != raio or not self.livre(c + dc, r + dr):
                        continue
                    d = math.hypot(dc, dr)
                    if melhor is None or d < melhor[0]:
                        melhor = (d, (c + dc, r + dr))
            if melhor:
                return melhor[1]
        return None

    def planejar(self, origem, destino):
        ini = self.mais_proxima_livre(*self.celula(*origem))
        fim = self.mais_proxima_livre(*self.celula(*destino))
        if ini is None or fim is None:
            return None
        custo_min = min(self.cfg.custo_tatil, 1.0)

        def h(c, r):
            dx, dy = abs(c - fim[0]), abs(r - fim[1])
            return custo_min * ((dx + dy) + (math.sqrt(2) - 2) * min(dx, dy))

        abertos = [(h(*ini), 0, ini)]
        g = {ini: 0.0}
        pai = {}
        fechados = set()
        cont = 0
        while abertos:
            _, _, atual = heapq.heappop(abertos)
            if atual in fechados:
                continue
            if atual == fim:
                break
            fechados.add(atual)
            c, r = atual
            for dc in (-1, 0, 1):
                for dr in (-1, 0, 1):
                    if dc == 0 and dr == 0:
                        continue
                    nc, nr = c + dc, r + dr
                    if not self.livre(nc, nr):
                        continue
                    if dc and dr and not (self.livre(c + dc, r) and self.livre(c, r + dr)):
                        continue
                    passo = (math.sqrt(2) if dc and dr else 1.0) * self.custo[nr][nc]
                    ng = g[atual] + passo
                    if ng < g.get((nc, nr), 1e18):
                        g[(nc, nr)] = ng
                        pai[(nc, nr)] = atual
                        cont += 1
                        heapq.heappush(abertos, (ng + h(nc, nr), cont, (nc, nr)))
        if fim not in pai and fim != ini:
            return None
        caminho = [fim]
        while caminho[-1] != ini:
            caminho.append(pai[caminho[-1]])
        caminho.reverse()
        pts = [(c * CELULA + CELULA // 2, r * CELULA + CELULA // 2) for c, r in caminho]
        pts[-1] = (destino[0], destino[1])
        return pts


# ==================== SEMAFORO E TRAFEGO ====================
class Semaforo:
    FASES = (
        ("VERDE", 9.0),
        ("AMARELO", 2.5),
        ("TODOS_VERM_1", 1.5),
        ("PEDESTRE", 9.0),
        ("TODOS_VERM_2", 1.5),
    )

    def __init__(self):
        self.indice = 0
        self.t = 0.0

    @property
    def fase(self):
        return self.FASES[self.indice][0]

    def tempo_restante(self):
        return self.FASES[self.indice][1] - self.t

    def atualizar(self, dt):
        self.t += dt
        while self.t >= self.FASES[self.indice][1]:
            self.t -= self.FASES[self.indice][1]
            self.indice = (self.indice + 1) % len(self.FASES)


class Veiculo:
    def __init__(self, faixa, x, vmax, infrator):
        self.faixa = faixa
        self.dir = 1 if faixa == 0 else -1
        self.x = x
        self.y = FAIXAS_Y[faixa]
        self.vmax = vmax
        self.v = 0.0
        self.infrator = infrator

    @property
    def progresso(self):
        return self.dir * self.x

    @property
    def rect(self):
        return pygame.Rect(int(self.x - COMP_VEI / 2), int(self.y - LARG_VEI / 2), COMP_VEI, LARG_VEI)

    def rect_varrido(self, horizonte):
        x2 = self.x + self.dir * self.v * horizonte
        x_min, x_max = min(self.x, x2) - COMP_VEI / 2, max(self.x, x2) + COMP_VEI / 2
        return pygame.Rect(int(x_min), int(self.y - LARG_VEI / 2), int(x_max - x_min), LARG_VEI)

    def dist_linha_retencao(self):
        if self.dir == 1:
            return (FAIXA_X - LINHA_RETENCAO) - (self.x + COMP_VEI / 2)
        return (self.x - COMP_VEI / 2) - (FAIXA_X + FAIXA_W + LINHA_RETENCAO)

    def atualizar(self, dt, lider, semaforo, pedestres):
        v_des = self.vmax
        if lider is not None:
            gap = lider.progresso - self.progresso - COMP_VEI
            d_seg = 14 + 0.45 * self.v
            if gap < d_seg + 40:
                v_des = min(v_des, max(0.0, lider.v + (gap - d_seg) * 1.5))
            if gap < 4:
                v_des = 0.0
        ds = self.dist_linha_retencao()
        if not self.infrator and semaforo.fase != "VERDE" and ds > -2:
            nao_para = semaforo.fase == "AMARELO" and ds > 0 and self.v ** 2 / (2 * ds) > DESACEL
            if semaforo.fase == "AMARELO" and ds <= 0:
                nao_para = True
            if not nao_para:
                v_des = min(v_des, math.sqrt(max(0.0, 2 * 0.7 * DESACEL * max(ds - 1, 0))))
        freio = DESACEL
        d_parada = self.v ** 2 / (2 * DESACEL) + self.v * 0.4 + 18
        zona = pygame.Rect(0, int(self.y - LARG_VEI / 2), int(d_parada), LARG_VEI)
        zona.left = int(self.x + COMP_VEI / 2) if self.dir == 1 else int(self.x - COMP_VEI / 2 - d_parada)
        if any(zona.colliderect(p) for p in pedestres):
            v_des = 0.0
            freio = DESACEL * 1.5
        a = max(-freio, min(ACEL, (v_des - self.v) / dt))
        self.v = max(0.0, self.v + a * dt)
        self.x += self.dir * self.v * dt


class Trafego:
    def __init__(self, rng, cfg):
        self.rng = rng
        self.cfg = cfg
        self.veiculos = []
        self.prox_spawn = [rng.uniform(0.2, 1.5), rng.uniform(0.5, 2.0)]

    def spawn(self, faixa, infrator=None, forcar_x=None):
        x0 = -COMP_VEI if faixa == 0 else AREA_W + COMP_VEI
        if forcar_x is not None:
            x0 = forcar_x
        mesma = [v for v in self.veiculos if v.faixa == faixa]
        dir_ = 1 if faixa == 0 else -1
        if forcar_x is None and mesma and min(dir_ * (v.x - x0) for v in mesma) < 90:
            return None
        if infrator is None:
            infrator = self.rng.random() < self.cfg.prob_infrator
        v = Veiculo(faixa, x0, self.rng.uniform(*VEL_VEICULO_MAX), infrator)
        v.v = v.vmax * 0.8 if (infrator or forcar_x is not None) else min(v.vmax, 120.0)
        self.veiculos.append(v)
        return v

    def atualizar(self, dt, semaforo, pedestres):
        for faixa in (0, 1):
            self.prox_spawn[faixa] -= dt
            if self.prox_spawn[faixa] <= 0:
                if self.spawn(faixa) is not None:
                    self.prox_spawn[faixa] = self.rng.uniform(1.0, 3.2)
                else:
                    self.prox_spawn[faixa] = 0.3
        for faixa in (0, 1):
            fila = sorted((v for v in self.veiculos if v.faixa == faixa), key=lambda v: -v.progresso)
            for i, v in enumerate(fila):
                v.atualizar(dt, fila[i - 1] if i > 0 else None, semaforo, pedestres)
        self.veiculos = [v for v in self.veiculos if -2 * COMP_VEI < v.x < AREA_W + 2 * COMP_VEI]


# ==================== SENSORIAMENTO ====================
class Lidar:
    def __init__(self, cfg, rng):
        self.cfg = cfg
        self.rng = rng
        self.alcance = 165
        self.num_raios = 19
        self.fov = math.radians(150)
        self.historico = [deque(maxlen=cfg.janela_mediana) for _ in range(self.num_raios)]
        self.leituras = []

    def varrer(self, x, y, angulo, alvos):
        cands = [(r, t) for r, t in alvos if dist_ponto_rect(x, y, r) <= self.alcance]
        inicio = angulo - self.fov / 2
        passo = self.fov / (self.num_raios - 1)
        ox, oy = int(x), int(y)
        self.leituras = []
        for i in range(self.num_raios):
            ang = inicio + i * passo
            fx = int(x + math.cos(ang) * self.alcance)
            fy = int(y + math.sin(ang) * self.alcance)
            dist, tipo = float(self.alcance), None
            for rect, t in cands:
                seg = rect.clipline(ox, oy, fx, fy)
                if not seg:
                    continue
                d = min(math.hypot(px - x, py - y) for px, py in seg)
                if d < dist:
                    dist, tipo = d, t
            if tipo is not None:
                dist = min(float(self.alcance), max(0.0, dist + self.rng.gauss(0.0, self.cfg.ruido_sigma)))
            if self.rng.random() < self.cfg.dropout:
                dist, tipo = float(self.alcance), None
            self.historico[i].append((dist, tipo or ""))
            mediana = sorted(self.historico[i])[len(self.historico[i]) // 2]
            self.leituras.append({"angulo": ang, "distancia": mediana[0], "tipo": mediana[1] or None})


class AudioNulo:
    def __init__(self):
        self.eventos = 0

    def tocar(self, pan, nivel):
        self.eventos += 1


class AudioEstereo(AudioNulo):
    """Alertas sinteticos com panorama estereo (pan -1 = esquerda, +1 = direita)."""

    def __init__(self):
        super().__init__()
        self.ativo = False
        self.ultimo = {}
        try:
            pygame.mixer.init(frequency=22050, size=-16, channels=2)
            self.sons = {"aviso": self._beep(660), "perigo": self._beep(990, 0.18), "freio": self._beep(440, 0.3)}
            self.ativo = True
        except (pygame.error, NotImplementedError, ImportError):
            self.sons = {}

    @staticmethod
    def _beep(freq, dur=0.12, vol=0.45, sr=22050):
        n = int(sr * dur)
        buf = array("h")
        for i in range(n):
            env = min(1.0, i / 200, (n - i) / 400)
            s = int(32767 * vol * env * math.sin(2 * math.pi * freq * i / sr))
            buf.append(s)
            buf.append(s)
        return pygame.mixer.Sound(buffer=buf.tobytes())

    def tocar(self, pan, nivel):
        super().tocar(pan, nivel)
        agora = pygame.time.get_ticks()
        if not self.ativo or agora - self.ultimo.get(nivel, -9999) < 450:
            return
        self.ultimo[nivel] = agora
        canal = self.sons[nivel].play()
        if canal:
            canal.set_volume(1.0 - max(pan, 0.0), 1.0 + min(pan, 0.0))


# ==================== TUTOR E ROBO ====================
class Tutor:
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.tensao = 0.0

    @property
    def caixa(self):
        return pygame.Rect(int(self.x - RAIO_TUTOR), int(self.y - RAIO_TUTOR), 2 * RAIO_TUTOR, 2 * RAIO_TUTOR)

    def atualizar(self, dt, robo):
        dx, dy = robo.x - self.x, robo.y - self.y
        d = math.hypot(dx, dy)
        if d > GUIA:
            passo = min(VEL_TUTOR * dt, d - GUIA)
            self.x += dx / d * passo
            self.y += dy / d * passo
        self.tensao = max(0.0, math.hypot(robo.x - self.x, robo.y - self.y) - GUIA)

    def fator_velocidade(self):
        if self.tensao <= FOLGA_TENSAO:
            return 1.0
        return max(0.1, 1.0 - (self.tensao - FOLGA_TENSAO) / TENSAO_MAX)


class RoboCaoGuia:
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.angulo = 0.0
        self.v = 0.0
        self.estado = OCIOSO
        self.motivo = "Aguardando destino."
        self.historico = []
        self.caminho = []
        self.idx = 0
        self.autorizada = False
        self.dir_travessia = 1
        self.freio_hold = 0.0
        self.t_desvio = 0.0
        self.cor_alerta = VERDE
        self.status = "Caminho livre."
        self.passo_animacao = 0.0
        self.alerta_vermelho_ativo = False
        self.t_total = 0.0

    @property
    def caixa(self):
        return pygame.Rect(int(self.x - RAIO_PLAN), int(self.y - RAIO_PLAN), 2 * RAIO_PLAN, 2 * RAIO_PLAN)

    def mudar(self, estado, motivo):
        if estado != self.estado:
            self.historico.append((round(self.t_total, 2), estado, motivo))
            self.estado = estado
        self.motivo = motivo

    def em_rua(self, y=None):
        y = self.y if y is None else y
        return RUA_Y < y < RUA_FIM

    def aplicar_barreira(self, y_antes, travessia_manual=False):
        if self.autorizada or travessia_manual:
            if RUA_Y < self.y < RUA_FIM:
                self.x = min(max(self.x, FAIXA_X + 14), FAIXA_X + FAIXA_W - 14)
            return
        if RUA_Y - MARGEM_BARREIRA < self.y < RUA_FIM + MARGEM_BARREIRA:
            self.y = RUA_Y - MARGEM_BARREIRA if y_antes <= RUA_Y else RUA_FIM + MARGEM_BARREIRA

    def ponto_lookahead(self, dist=30):
        if not self.caminho:
            return None
        n = len(self.caminho)
        while self.idx + 1 < n and (
            math.hypot(self.x - self.caminho[self.idx + 1][0], self.y - self.caminho[self.idx + 1][1])
            < math.hypot(self.x - self.caminho[self.idx][0], self.y - self.caminho[self.idx][1])
        ):
            self.idx += 1
        for j in range(self.idx, n):
            px, py = self.caminho[j]
            if math.hypot(px - self.x, py - self.y) >= dist:
                return (px, py)
        return self.caminho[-1]

    def precisa_esperar_sinal(self):
        if self.em_rua():
            return False
        for j in range(self.idx, len(self.caminho)):
            px, py = self.caminho[j]
            if RUA_Y <= py <= RUA_FIM:
                return math.hypot(px - self.x, py - self.y) <= ESPERA_DIST
        return False

    def motivo_bloqueio(self, sim):
        sem = sim.semaforo
        if sem.fase != "PEDESTRE":
            return f"Sinal fechado para pedestre ({sem.fase})"
        if sem.tempo_restante() <= limite_travessia():
            return "Tempo restante insuficiente para travessia"
        if not sim.travessia_livre(limite_travessia()):
            return "Veiculo em aproximacao na faixa"
        if math.hypot(self.x - sim.tutor.x, self.y - sim.tutor.y) > GUIA + FOLGA_TENSAO:
            return "Aguardando o tutor se aproximar"
        return None

    def mover(self, dt, v, sim):
        self.v = v
        y_antes = self.y
        self.x += math.cos(self.angulo) * v * dt
        self.y += math.sin(self.angulo) * v * dt
        self.aplicar_barreira(y_antes, sim.travessia_manual())
        self.passo_animacao += v * dt * 0.15

    def virar_para(self, alvo, dt):
        ang_alvo = math.atan2(alvo[1] - self.y, alvo[0] - self.x)
        diff = normalizar_angulo(ang_alvo - self.angulo)
        self.angulo += max(-OMEGA * dt, min(OMEGA * dt, diff))
        return diff

    def atualizar(self, dt, sim):
        self.t_total += dt
        if self.estado in (OCIOSO, CHEGOU, SEM_ROTA):
            self.v = 0.0
            self.cor_alerta = VERDE
            return
        if sim.risco_colisao():
            self.freio_hold = 0.4
        if self.freio_hold > 0:
            self.freio_hold -= dt
            self.mudar(FREIO, "Zona varrida de veiculo intercepta robo/tutor")
            self.v = 0.0
            self.cor_alerta = VERMELHO
            self.status = "FREIO DE EMERGENCIA! Veiculo em rota de colisao."
            return
        if self.estado == FREIO:
            self.mudar(ATRAVESSANDO if self.autorizada else NAVEGANDO, "Risco de colisao encerrado")

        fator = sim.tutor.fator_velocidade()
        alvo = self.ponto_lookahead()
        if alvo is None:
            self.v = 0.0
            return

        if self.autorizada:
            self.mudar(ATRAVESSANDO, "Sinal de pedestre aberto e tempo suficiente")
            self.status = "Atravessando na faixa."
            self.cor_alerta = AMARELO_COR
            self.virar_para(alvo, dt)
            self.mover(dt, VEL_ROBO * fator, sim)
            sim.sentir_alertas(self)
            fora = self.y > RUA_FIM + 6 if self.dir_travessia == 1 else self.y < RUA_Y - 6
            tutor_fora = (
                sim.tutor.y > RUA_FIM + RAIO_TUTOR if self.dir_travessia == 1 else sim.tutor.y < RUA_Y - RAIO_TUTOR
            )
            if fora and tutor_fora:
                self.autorizada = False
                self.mudar(NAVEGANDO, "Robo e tutor fora da pista")
            return

        if math.hypot(self.x - sim.destino[0], self.y - sim.destino[1]) <= 22:
            self.mudar(CHEGOU, "Distancia ao destino <= 22 px")
            self.status = "DESTINO ALCANCADO COM SEGURANCA!"
            self.cor_alerta = VERDE
            self.v = 0.0
            sim.ao_chegar()
            return

        if self.precisa_esperar_sinal():
            bloqueio = self.motivo_bloqueio(sim)
            if bloqueio is None:
                self.autorizada = True
                self.dir_travessia = 1 if self.y < RUA_Y else -1
                self.mudar(ATRAVESSANDO, "Sinal de pedestre aberto e tempo suficiente")
                return
            self.mudar(AGUARDANDO, bloqueio)
            self.status = f"Aguardando sinal: {bloqueio}"
            self.cor_alerta = LARANJA
            self.v = 0.0
            return

        self.navegar(dt, sim, alvo, fator)

    def navegar(self, dt, sim, alvo, fator):
        leit = sim.lidar.leituras
        alc = sim.lidar.alcance

        def efetiva(r):
            return alc if r["tipo"] == "veiculo" else r["distancia"]

        esq = min(efetiva(r) for r in leit[:6])
        cen = min(efetiva(r) for r in leit[6:13])
        dire = min(efetiva(r) for r in leit[13:])
        sim.sentir_alertas(self)
        if cen < 44:
            suspenso = any(r["tipo"] == "suspenso" and r["distancia"] < 100 for r in leit[6:13])
            self.status = "ALERTA CABEÇA: obstáculo alto!" if suspenso else "Obstáculo à frente: contornando."
            self.angulo += (-OMEGA if esq > dire else OMEGA) * dt * 0.8
            v = VEL_ROBO * 0.4
            self.mudar(DESVIANDO, "Obstaculo frontal a menos de 44 px")
        elif esq < 26:
            self.status = "Obstáculo à esquerda. Deslocando à direita."
            self.angulo += OMEGA * dt * 0.5
            v = VEL_ROBO * 0.8
            self.mudar(DESVIANDO, "Obstaculo lateral esquerdo")
        elif dire < 26:
            self.status = "Obstáculo à direita. Deslocando à esquerda."
            self.angulo -= OMEGA * dt * 0.5
            v = VEL_ROBO * 0.8
            self.mudar(DESVIANDO, "Obstaculo lateral direito")
        else:
            diff = self.virar_para(alvo, dt)
            v = VEL_ROBO * (max(0.35, math.cos(diff)) if abs(diff) > 0.6 else 1.0)
            self.status = "Caminho livre. Seguindo rota planejada."
            self.mudar(NAVEGANDO, "Via livre: seguindo A*")
        self.t_desvio = self.t_desvio + dt if self.estado == DESVIANDO else max(0.0, self.t_desvio - dt)
        if self.t_desvio > 3.5:
            self.t_desvio = 0.0
            sim.replanejar_por_travamento()
            return
        self.mover(dt, v * fator, sim)


# ==================== SIMULACAO ====================
@dataclass
class Metricas:
    distancia: float = 0.0
    alertas_vermelhos: int = 0
    freios: int = 0
    viagens: int = 0
    colisoes: int = 0
    quase_colisoes: int = 0
    replanejamentos: int = 0


class Simulacao:
    def __init__(self, cfg=None, seed=1, audio=None):
        self.cfg = cfg or Config()
        self.rng = random.Random(seed)
        self.t = 0.0
        self.semaforo = Semaforo()
        self.trafego = Trafego(self.rng, self.cfg)
        self.obstaculos = criar_cenario()
        self.bloqueios = []
        self.audio = audio or AudioNulo()
        self.m = Metricas()
        self.lidar = Lidar(self.cfg, self.rng)
        self.robo = RoboCaoGuia(*CASA)
        self.tutor = Tutor(CASA[0] - GUIA, CASA[1])
        self.destino = CASA
        self.nome_destino = "-"
        self.fila = []
        self.t_viagem = 0.0
        self.viagem_concluida = False
        self.contatos = {}
        self.grade = None
        self.reconstruir_grade()

    # ---- planejamento ----
    def reconstruir_grade(self):
        self.bloqueios = [b for b in self.bloqueios if b[3] > self.t]
        self.grade = GradeOcupacao(self.obstaculos, self.cfg, [(x, y, r) for x, y, r, _ in self.bloqueios])

    def planejar(self):
        caminho = self.grade.planejar((self.robo.x, self.robo.y), self.destino)
        if caminho is None and self.bloqueios:
            self.bloqueios = []
            self.reconstruir_grade()
            caminho = self.grade.planejar((self.robo.x, self.robo.y), self.destino)
        self.robo.caminho = caminho or []
        self.robo.idx = 0
        if caminho:
            self.robo.mudar(PLANEJANDO, f"Rota A* calculada ate {self.nome_destino}")
        else:
            self.robo.mudar(SEM_ROTA, "A* nao encontrou rota")

    def definir_destino(self, nome, pos, fila=()):
        self.nome_destino, self.destino = nome, pos
        self.fila = list(fila)
        self.viagem_concluida = False
        self.t_viagem = 0.0
        self.robo.autorizada = False
        self.planejar()

    def iniciar_viagem(self, ids):
        nome, pos = DESTINOS[ids[0]]
        self.definir_destino(nome, pos, ids[1:])

    def ao_chegar(self):
        if self.fila:
            prox = self.fila.pop(0)
            nome, pos = DESTINOS[prox]
            fila = list(self.fila)
            self.definir_destino(nome, pos, fila)
            return
        self.m.viagens += 1
        self.viagem_concluida = True

    def replanejar_por_travamento(self):
        r = self.robo
        leit = self.lidar.leituras
        cand = [l for l in leit[6:13] if l["tipo"] not in (None, "veiculo")]
        if cand:
            l = min(cand, key=lambda l: l["distancia"])
            bx = r.x + math.cos(l["angulo"]) * l["distancia"]
            by = r.y + math.sin(l["angulo"]) * l["distancia"]
            self.bloqueios.append((bx, by, 26, self.t + 12.0))
            self.reconstruir_grade()
        self.m.replanejamentos += 1
        self.planejar()

    def adicionar_obstaculo(self, x, y, tipo):
        self.obstaculos.append({"rect": pygame.Rect(x - 22, y - 22, 44, 44), "tipo": tipo, "nome": "Obstáculo"})
        self.reconstruir_grade()
        if self.robo.estado not in (OCIOSO, CHEGOU):
            self.planejar()

    def reiniciar_robo(self):
        self.robo = RoboCaoGuia(*CASA)
        self.tutor = Tutor(CASA[0] - GUIA, CASA[1])
        self.lidar = Lidar(self.cfg, self.rng)
        self.contatos = {}

    # ---- seguranca ----
    def travessia_manual(self):
        return self.robo.estado == OCIOSO and self.semaforo.fase == "PEDESTRE"

    def pedestres(self):
        return [self.robo.caixa, self.tutor.caixa]

    def risco_colisao(self):
        caixas = self.pedestres()
        for v in self.trafego.veiculos:
            if v.v < 8:
                continue
            zona = v.rect_varrido(self.cfg.horizonte_s).inflate(0, 6)
            if any(zona.colliderect(c) for c in caixas):
                return True
        return False

    def travessia_livre(self, horizonte):
        zona = pygame.Rect(FAIXA_X - 10, RUA_Y, FAIXA_W + 20, RUA_H)
        for v in self.trafego.veiculos:
            if v.v >= 8 and v.rect_varrido(horizonte).colliderect(zona):
                return False
        return True

    def sentir_alertas(self, robo):
        leit = self.lidar.leituras
        if not leit:
            return
        ruim = min(leit[6:13], key=lambda r: r["distancia"])
        lado = min(leit, key=lambda r: r["distancia"])
        if ruim["distancia"] < 44:
            nivel, ref = "perigo", ruim
            robo.cor_alerta = VERMELHO
        elif lado["distancia"] < 26:
            nivel, ref = "aviso", lado
            robo.cor_alerta = LARANJA
        else:
            robo.cor_alerta = VERDE if robo.estado != ATRAVESSANDO else AMARELO_COR
            robo.alerta_vermelho_ativo = False
            return
        pan = max(-1.0, min(1.0, normalizar_angulo(ref["angulo"] - robo.angulo) / (self.lidar.fov / 2)))
        self.audio.tocar(pan, nivel)
        if nivel == "perigo" and not robo.alerta_vermelho_ativo:
            self.m.alertas_vermelhos += 1
        robo.alerta_vermelho_ativo = nivel == "perigo"

    # ---- metricas de contato ----
    def _contato(self, agente, raio, cx, cy, obj_id, rect, limiar):
        d = dist_ponto_rect(cx, cy, rect) - raio
        chave = (agente, obj_id)
        estado = self.contatos.get(chave)
        if d < 0:
            if estado is not True:
                self.m.colisoes += 1
            self.contatos[chave] = True
        elif d < limiar:
            if estado is None:
                self.contatos[chave] = False
        elif chave in self.contatos:
            if self.contatos.pop(chave) is False:
                self.m.quase_colisoes += 1

    def fechar_episodios(self):
        self.m.quase_colisoes += sum(1 for v in self.contatos.values() if v is False)
        self.contatos = {}

    def avaliar_contatos(self):
        r, tu = self.robo, self.tutor
        for i, o in enumerate(self.obstaculos):
            if o["tipo"] != "suspenso":
                self._contato("robo", RAIO_COL, r.x, r.y, ("o", i), o["rect"], 0)
            self._contato("tutor", RAIO_TUTOR, tu.x, tu.y, ("o", i), o["rect"], 0)
        for i, p in enumerate(PAREDES):
            self._contato("robo", RAIO_COL, r.x, r.y, ("p", i), p, 0)
        for v in self.trafego.veiculos:
            self._contato("robo", RAIO_COL, r.x, r.y, ("v", id(v)), v.rect, 25)
            self._contato("tutor", RAIO_TUTOR, tu.x, tu.y, ("v", id(v)), v.rect, 25)
        vivos = {("v", id(v)) for v in self.trafego.veiculos}
        for chave in [k for k in self.contatos if k[1][0] == "v" and k[1] not in vivos]:
            if self.contatos.pop(chave) is False:
                self.m.quase_colisoes += 1

    # ---- passo ----
    def passo(self, dt):
        self.t += dt
        if not self.viagem_concluida and self.robo.estado not in (OCIOSO, CHEGOU):
            self.t_viagem += dt
        self.semaforo.atualizar(dt)
        self.trafego.atualizar(dt, self.semaforo, self.pedestres())
        alvos = [(o["rect"], o["tipo"]) for o in self.obstaculos]
        alvos += [(p, "parede") for p in PAREDES]
        alvos += [(v.rect, "veiculo") for v in self.trafego.veiculos]
        self.lidar.varrer(self.robo.x, self.robo.y, self.robo.angulo, alvos)
        ant = (self.robo.x, self.robo.y)
        estado_ant = self.robo.estado
        self.robo.atualizar(dt, self)
        if self.robo.estado == FREIO and estado_ant != FREIO:
            self.m.freios += 1
            self.audio.tocar(0.0, "freio")
        self.m.distancia += math.hypot(self.robo.x - ant[0], self.robo.y - ant[1])
        self.tutor.atualizar(dt, self.robo)
        self.avaliar_contatos()


# ==================== BENCHMARK ====================
def rodar_benchmark(n, seed, cfg, ida_volta=False, saida=None):
    sim = Simulacao(cfg, seed)
    ciclo = [1, 2, 3]
    resultados = []
    for i in range(n):
        dest = ciclo[i % len(ciclo)]
        sim.reiniciar_robo()
        espera = sim.rng.uniform(0.0, sum(d for _, d in Semaforo.FASES))
        for _ in range(int(espera / DT_FIXO)):
            sim.passo(DT_FIXO)
        ini = asdict(sim.m)
        sim.iniciar_viagem([dest] + ([4] if ida_volta else []))
        t0 = sim.t
        while not sim.viagem_concluida and sim.t - t0 < cfg.timeout_viagem:
            sim.passo(DT_FIXO)
        sucesso = sim.viagem_concluida
        sim.fechar_episodios()
        fim = asdict(sim.m)
        resultados.append({
            "viagem": i + 1,
            "destino": DESTINOS[dest][0],
            "sucesso": sucesso,
            "tempo_s": round(sim.t - t0, 2),
            "colisoes": fim["colisoes"] - ini["colisoes"],
            "quase_colisoes": fim["quase_colisoes"] - ini["quase_colisoes"],
            "freios": fim["freios"] - ini["freios"],
            "alertas_vermelhos": fim["alertas_vermelhos"] - ini["alertas_vermelhos"],
            "replanejamentos": fim["replanejamentos"] - ini["replanejamentos"],
            "distancia_px": round(fim["distancia"] - ini["distancia"], 1),
        })
        sim.m.viagens = sum(1 for r in resultados if r["sucesso"])
    ok = [r for r in resultados if r["sucesso"]]
    resumo = {
        "seed": seed,
        "viagens": n,
        "taxa_sucesso_pct": round(100 * len(ok) / n, 1),
        "tempo_medio_s": round(sum(r["tempo_s"] for r in ok) / len(ok), 2) if ok else None,
        "colisoes": sum(r["colisoes"] for r in resultados),
        "quase_colisoes": sum(r["quase_colisoes"] for r in resultados),
        "freios": sum(r["freios"] for r in resultados),
        "alertas_vermelhos": sum(r["alertas_vermelhos"] for r in resultados),
        "replanejamentos": sum(r["replanejamentos"] for r in resultados),
        "config": asdict(cfg),
    }
    for r in resultados:
        print(
            f"#{r['viagem']:02d} {r['destino']:<9} ok={str(r['sucesso']):<5} t={r['tempo_s']:6.2f}s "
            f"col={r['colisoes']} quase={r['quase_colisoes']} freios={r['freios']} red={r['alertas_vermelhos']}"
        )
    print("\n=== RESUMO ===")
    for k, v in resumo.items():
        if k != "config":
            print(f"{k}: {v}")
    if saida:
        os.makedirs(os.path.dirname(saida) or ".", exist_ok=True)
        with open(saida, "w", encoding="utf-8") as f:
            json.dump({"resumo": resumo, "viagens": resultados}, f, ensure_ascii=False, indent=2)
        print(f"Resultados salvos em {saida}")
    return resumo, resultados


# ==================== RENDERIZACAO ====================
class Visual:
    def __init__(self):
        self.tela = pygame.display.set_mode((LARGURA, ALTURA))
        pygame.display.set_caption("DOG-IA V2 — Simulador Urbano de Cão-Guia Autônomo")
        self.f_titulo = pygame.font.SysFont("arial", 18, bold=True)
        self.f_texto = pygame.font.SysFont("arial", 13)
        self.f_peq = pygame.font.SysFont("arial", 12, bold=True)
        self.overlay_grade = False
        self.cache_grade = None
        self.grade_ref = None

    def _overlay(self, sim):
        if self.grade_ref is not sim.grade:
            surf = pygame.Surface((AREA_W, ALTURA), pygame.SRCALPHA)
            for r in range(LINHAS):
                for c in range(COLS):
                    if sim.grade.bloq[r][c]:
                        surf.fill((225, 55, 55, 70), (c * CELULA, r * CELULA, CELULA, CELULA))
                    elif sim.grade.custo[r][c] < 1.0:
                        surf.fill((235, 190, 20, 60), (c * CELULA, r * CELULA, CELULA, CELULA))
            self.cache_grade, self.grade_ref = surf, sim.grade
        return self.cache_grade

    def ambiente(self, sim):
        t = self.tela
        t.fill(CALCADA)
        pygame.draw.rect(t, GRAMA, (0, 0, AREA_W, 28))
        pygame.draw.rect(t, GRAMA, (0, ALTURA - 28, AREA_W, 28))
        pygame.draw.rect(t, ASFALTO, (0, RUA_Y, AREA_W, RUA_H))
        pygame.draw.line(t, MEIO_FIO, (0, RUA_Y), (AREA_W, RUA_Y), 4)
        pygame.draw.line(t, MEIO_FIO, (0, RUA_FIM), (AREA_W, RUA_FIM), 4)
        for x in range(15, AREA_W - 20, 45):
            if not (FAIXA_X - 10 <= x <= FAIXA_X + FAIXA_W + 5):
                pygame.draw.line(t, BRANCO, (x, RUA_Y + RUA_H // 2), (x + 22, RUA_Y + RUA_H // 2), 3)
        for y in range(RUA_Y + 12, RUA_FIM - 12, 22):
            pygame.draw.rect(t, BRANCO, (FAIXA_X, y, FAIXA_W, 12))
        for rect in TATIL_RECTS:
            pygame.draw.rect(t, PISO_TATIL, rect)
        fase = sim.semaforo.fase
        cor_v = {"VERDE": VERDE, "AMARELO": AMARELO_COR}.get(fase, VERMELHO)
        for (x, y) in ((FAIXA_X - LINHA_RETENCAO, RUA_Y + 10), (FAIXA_X + FAIXA_W + LINHA_RETENCAO, RUA_FIM - 10)):
            pygame.draw.line(t, BRANCO, (x, y - 8 if y < 300 else y - 40), (x, y + 40 if y < 300 else y + 8), 2)
            pygame.draw.circle(t, cor_v, (x, y + (25 if y < 300 else -25)), 7)
        cor_p = VERDE if fase == "PEDESTRE" else VERMELHO
        for (x, y) in ((FAIXA_X + FAIXA_W + 14, RUA_Y - 14), (FAIXA_X - 14, RUA_FIM + 14)):
            pygame.draw.rect(t, PRETO, (x - 8, y - 8, 16, 16), border_radius=3)
            pygame.draw.circle(t, cor_p, (x, y), 5)

    def veiculos(self, sim):
        for v in sim.trafego.veiculos:
            cor = (200, 70, 70) if v.infrator else (90, 150, 220) if v.dir == 1 else (120, 190, 120)
            pygame.draw.rect(self.tela, cor, v.rect, border_radius=5)
            pygame.draw.rect(self.tela, BRANCO, v.rect, 2, border_radius=5)
            if v.infrator:
                lbl = self.f_peq.render("!", True, BRANCO)
                self.tela.blit(lbl, (v.rect.centerx - 3, v.rect.centery - 7))

    def obstaculos(self, sim):
        for o in sim.obstaculos:
            rect = o["rect"]
            if o["tipo"] == "suspenso":
                pygame.draw.rect(self.tela, (90, 95, 102), rect.move(5, 5), border_radius=12)
                pygame.draw.rect(self.tela, ROXO, rect, border_radius=12)
                rot = "ALTO"
            elif o["tipo"] == "edificacao":
                pygame.draw.rect(self.tela, (96, 88, 82), rect)
                rot = "PRÉDIO"
            else:
                pygame.draw.rect(self.tela, (70, 74, 82), rect, border_radius=5)
                rot = "SOLO"
            pygame.draw.rect(self.tela, BRANCO, rect, 2, border_radius=5)
            lbl = self.f_peq.render(rot, True, BRANCO)
            self.tela.blit(lbl, (rect.centerx - lbl.get_width() // 2, rect.centery - 7))

    def robo(self, sim):
        r, t = sim.robo, self.tela
        for l in sim.lidar.leituras:
            fx, fy = r.x + math.cos(l["angulo"]) * l["distancia"], r.y + math.sin(l["angulo"]) * l["distancia"]
            cor = VERDE if l["distancia"] >= sim.lidar.alcance - 5 else VERMELHO
            pygame.draw.line(t, cor, (r.x, r.y), (fx, fy), 1)
            if l["tipo"]:
                pygame.draw.circle(t, BRANCO, (int(fx), int(fy)), 3)
        tu = sim.tutor
        pygame.draw.line(t, ARNES_GUIA, (r.x, r.y), (tu.x, tu.y), 4)
        pygame.draw.circle(t, (235, 235, 240), (int(tu.x), int(tu.y)), RAIO_TUTOR)
        pygame.draw.circle(t, PRETO, (int(tu.x), int(tu.y)), RAIO_TUTOR, 2)
        balanco = math.sin(r.passo_animacao) * 5
        for ox, oy in [(10 + balanco, -14), (10 - balanco, 14), (-12 - balanco, -14), (-12 + balanco, 14)]:
            px, py = rotacionar_ponto(r.x, r.y, r.x + ox, r.y + oy, r.angulo)
            pygame.draw.circle(t, DETALHE_ROBO, (px, py), 5)
        corpo = [rotacionar_ponto(r.x, r.y, r.x + dx, r.y + dy, r.angulo)
                 for dx, dy in ((16, -11), (16, 11), (-18, 11), (-18, -11))]
        pygame.draw.polygon(t, CORPO_ROBO, corpo)
        pygame.draw.polygon(t, BRANCO, corpo, 2)
        cx, cy = rotacionar_ponto(r.x, r.y, r.x + 21, r.y, r.angulo)
        pygame.draw.circle(t, CABECA_ROBO, (cx, cy), 9)
        pygame.draw.circle(t, DETALHE_ROBO, (cx, cy), 9, 2)
        lx, ly = rotacionar_ponto(r.x, r.y, r.x + 3, r.y, r.angulo)
        pygame.draw.circle(t, LARANJA, (lx, ly), 5)

    def painel(self, sim, auto, audio_on):
        t, r = self.tela, sim.robo
        px = AREA_W
        pygame.draw.rect(t, PRETO, (px, 0, LARGURA - px, ALTURA))
        pygame.draw.line(t, BRANCO, (px, 0), (px, ALTURA), 2)
        y = 12
        t.blit(self.f_titulo.render("TELEMETRIA DOG-IA V2", True, CORPO_ROBO), (px + 12, y))
        y += 30
        sem = sim.semaforo
        linhas = [
            f"Estado: {r.estado}",
            f"Destino: {sim.nome_destino}",
            f"Semáforo: {sem.fase} ({sem.tempo_restante():.1f}s)",
            f"Limite travessia: {limite_travessia():.2f}s",
            f"Tensão do arnês: {sim.tutor.tensao:4.1f} px",
            f"Velocidade: {r.v:5.1f} px/s",
            f"Tempo viagem: {sim.t_viagem:5.1f} s",
            f"Distância: {sim.m.distancia:6.0f} px",
            f"Alertas vermelhos: {sim.m.alertas_vermelhos}",
            f"Freios de emergência: {sim.m.freios}",
            f"Colisões: {sim.m.colisoes}  Quase: {sim.m.quase_colisoes}",
            f"Replanejamentos: {sim.m.replanejamentos}",
            f"Viagens concluídas: {sim.m.viagens}",
            f"Modo: {'AUTÔNOMO' if auto else 'MANUAL'}  Áudio: {'on' if audio_on else 'off'}",
        ]
        for ln in linhas:
            t.blit(self.f_texto.render(ln, True, BRANCO), (px + 12, y))
            y += 21
        y += 6
        pygame.draw.rect(t, r.cor_alerta, (px + 12, y, 236, 70), border_radius=8)
        palavras, atual, yy = r.status.split(" "), "", y + 8
        for p in palavras:
            if len(atual + p) < 30:
                atual += p + " "
            else:
                t.blit(self.f_texto.render(atual, True, PRETO), (px + 20, yy))
                yy += 17
                atual = p + " "
        t.blit(self.f_texto.render(atual, True, PRETO), (px + 20, yy))
        y += 82
        for ln in [
            "CONTROLES",
            "1 Mercado 2 Farmácia 3 Padaria 4 Casa",
            "Clique esq: novo destino",
            "Clique dir: criar obstáculo",
            "V: gerar veículo infrator",
            "G: C-Space / rota   M: áudio",
            "ESPAÇO: manual/IA   R: reiniciar",
        ]:
            t.blit(self.f_texto.render(ln, True, BRANCO), (px + 12, y))
            y += 19

    def desenhar(self, sim, auto, audio_on):
        self.ambiente(sim)
        if self.overlay_grade:
            self.tela.blit(self._overlay(sim), (0, 0))
            if len(sim.robo.caminho) > 1:
                pygame.draw.lines(self.tela, (0, 200, 255), False, sim.robo.caminho, 2)
        pygame.draw.circle(self.tela, VERDE, (int(sim.destino[0]), int(sim.destino[1])), 20, 3)
        pygame.draw.circle(self.tela, VERDE, (int(sim.destino[0]), int(sim.destino[1])), 7)
        self.obstaculos(sim)
        self.veiculos(sim)
        self.robo(sim)
        self.painel(sim, auto, audio_on)
        pygame.display.flip()


def executar_gui(cfg, seed):
    pygame.init()
    audio = AudioEstereo()
    sim = Simulacao(cfg, seed, audio)
    visual = Visual()
    clock = pygame.time.Clock()
    auto = True
    sim.iniciar_viagem([1])
    rodando = True
    while rodando:
        dt = min(clock.tick(FPS) / 1000.0, 0.05)
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                rodando = False
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.pos[0] < AREA_W:
                if ev.button == 1:
                    sim.definir_destino("Ponto livre", ev.pos)
                elif ev.button == 3:
                    sim.adicionar_obstaculo(ev.pos[0], ev.pos[1], random.choice(["solo", "suspenso"]))
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4):
                    sim.iniciar_viagem([ev.key - pygame.K_0])
                elif ev.key == pygame.K_SPACE:
                    auto = not auto
                    if not auto:
                        sim.robo.mudar(OCIOSO, "Modo manual")
                    else:
                        sim.planejar()
                elif ev.key == pygame.K_r:
                    sim.reiniciar_robo()
                    sim.obstaculos = criar_cenario()
                    sim.reconstruir_grade()
                    sim.iniciar_viagem([1])
                elif ev.key == pygame.K_g:
                    visual.overlay_grade = not visual.overlay_grade
                elif ev.key == pygame.K_m:
                    audio.ativo = not audio.ativo and bool(audio.sons)
                elif ev.key == pygame.K_v:
                    sim.trafego.spawn(random.choice((0, 1)), infrator=True)
        if not auto:
            teclas = pygame.key.get_pressed()
            r = sim.robo
            if teclas[pygame.K_LEFT]:
                r.angulo -= 2.8 * dt
            if teclas[pygame.K_RIGHT]:
                r.angulo += 2.8 * dt
            if teclas[pygame.K_UP]:
                y_antes = r.y
                r.x += math.cos(r.angulo) * VEL_ROBO * dt
                r.y += math.sin(r.angulo) * VEL_ROBO * dt
                r.aplicar_barreira(y_antes, sim.travessia_manual())
        sim.passo(dt)
        visual.desenhar(sim, auto, audio.ativo)
    pygame.quit()


def main():
    ap = argparse.ArgumentParser(description="DOG-IA V2")
    ap.add_argument("--benchmark", type=int, metavar="N", help="executa N viagens sem interface")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--ida-volta", action="store_true", help="cada viagem inclui o retorno para casa")
    ap.add_argument("--saida", help="arquivo JSON com os resultados do benchmark")
    ap.add_argument("--janela", type=int, default=3, help="janela do filtro de mediana do LiDAR")
    ap.add_argument("--infl-solo", type=int, default=30)
    ap.add_argument("--infl-edif", type=int, default=22)
    ap.add_argument("--infl-susp", type=int, default=38)
    ap.add_argument("--custo-tatil", type=float, default=0.45)
    ap.add_argument("--horizonte", type=float, default=1.0)
    ap.add_argument("--prob-infrator", type=float, default=0.12)
    a = ap.parse_args()
    cfg = Config(
        infl_solo=a.infl_solo, infl_edif=a.infl_edif, infl_susp=a.infl_susp, custo_tatil=a.custo_tatil,
        janela_mediana=a.janela, horizonte_s=a.horizonte, prob_infrator=a.prob_infrator,
    )
    if a.benchmark:
        rodar_benchmark(a.benchmark, a.seed, cfg, a.ida_volta, a.saida)
    else:
        executar_gui(cfg, a.seed)


if __name__ == "__main__":
    main()
