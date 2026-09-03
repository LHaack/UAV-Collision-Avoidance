# Algoritmo de Collision Avoidance baseline (VFH+ vanilla)

Este documento descreve o algoritmo usado como **baseline de comparação** contra o
nosso collision avoidance, ativado pela flag `--px4-standard` do
`run_collision_test.sh`.

---

## 1. Qual algoritmo e por quê

**VFH+ "de livro"** (Vector Field Histogram+, Ulrich & Borenstein, 1998) — uma
implementação **vanilla/textbook**, sem nenhuma das melhorias do nosso algoritmo.

### Por que VFH+ (e não o "Collision Prevention" do PX4)
Investigamos o que o PX4 oferece de fábrica:
- O único método **embarcado no firmware** é o **Collision Prevention**, que apenas
  **freia e para** diante do obstáculo (não contorna) e, além disso, só roda no
  modo Position manual — não em offboard. Não serve como baseline de um algoritmo
  que *desvia*.
- O método "oficial" do PX4 que de fato **contorna** obstáculos é o **3DVFH\*** do
  projeto **PX4-Avoidance** — mas roda num *companion computer*, é **ROS1 +
  Gazebo Classic**, está **depreciado**, e só funciona em modo Mission. Inviável
  no nosso stack (ROS2 + gz novo + offboard + PX4 1.14).

O **3DVFH\* é, no fundo, um VFH** com lookahead. Então o baseline justo e factível
é reimplementar o **VFH+ clássico** dentro do nosso próprio nó ROS2 — mesmo stack,
mesma entrada (LiDAR) e saída (velocity setpoint) que o nosso algoritmo.

> **Importante:** o nosso `avoid_obstacles_3d` **também é um VFH+**, porém
> *turbinado* (repulsão de emergência, wall-sliding, 3D, tuning próprio). Este
> baseline é o VFH+ **puro/de livro**, de propósito sem essas melhorias — para
> medir exatamente **o que elas agregam**.

---

## 2. Como o VFH+ funciona

VFH+ é um método **reativo** baseado num **histograma polar** das direções ao redor
do drone. Por ciclo de controle:

### Passo 1 — Histograma polar primário
- Divide o entorno em **72 setores** de **5°**.
- Cada ponto do LiDAR "vota" no seu setor com magnitude
  `m = a − b·d` (mais perto = maior).
- **Alargamento (enlargement):** cada obstáculo a distância `d` é inflado pelo
  **raio do drone + margem**, espalhando seu voto por `±γ` setores, com
  `γ = asin(raio / d)`. Isso permite tratar o drone como um ponto.

### Passo 2 — Histograma binário (com histerese)
- Setor vira **bloqueado** se a densidade passa de `τ_high`; volta a **livre** só
  abaixo de `τ_low`. A histerese (dois limiares) evita "piscar" entre livre/bloqueado.

### Passo 3 — Candidatos a partir das aberturas
- Identifica **aberturas** (sequências contíguas de setores livres).
- **Abertura larga** (> `s_max` setores): gera candidatos perto das **duas bordas**
  e, se o alvo cair dentro dela, na **direção do alvo**.
- **Abertura estreita:** gera o candidato no **centro** da abertura.

### Passo 4 — Função de custo
Cada candidato `c` recebe:
```
g(c) = μ1·Δ(c, alvo) + μ2·Δ(c, heading_atual) + μ3·Δ(c, direção_anterior)
```
onde `Δ` é a diferença angular (em setores). Escolhe o candidato de **menor custo**:
- `μ1` puxa para o alvo,
- `μ2` evita curvas bruscas,
- `μ3` dá histerese de direção (não oscilar entre ciclos).

### Passo 5 — Velocidade
Vai na direção escolhida, com módulo **reduzido** quando há obstáculo próximo à
frente (e desacelerando ao chegar perto do alvo).

### Limitação proposital (é o ponto da comparação)
Sendo **reativo e sem lookahead**, o VFH+ vanilla pode **travar em mínimos locais**
(ex.: obstáculo côncavo / beco em "U"): se não há setor livre, ele **para**. É
justamente onde as melhorias do nosso algoritmo (wall-sliding, repulsão, 3D) fazem
diferença.

---

## 3. Parâmetros (constantes em `avoid_obstacles_vfh`, `velocity_control.py`)

| Parâmetro | Valor | O que faz |
|-----------|-------|-----------|
| `N` | 72 | número de setores (5° cada) |
| `DETECT_DIST` | 5.0 m | alcance do LiDAR considerado |
| `ROBOT_RADIUS` | 0.6 m | raio do drone + margem (alargamento γ) |
| `A_CONST`, `B_CONST` | 5.0, 1.0 | magnitude do voto `m = A − B·d` |
| `TAU_HIGH`, `TAU_LOW` | 3.0, 1.5 | limiares do binário (histerese) |
| `S_MAX` | 16 | largura (setores) que separa abertura "larga" de "estreita" |
| `MU1, MU2, MU3` | 5, 2, 2 | pesos do custo: alvo, heading, direção anterior |
| `SAFE_DIST` | 2.5 m | abaixo disso reduz a velocidade |

---

## 4. Implementação detalhada (passo a passo do código)

Tudo vive no método **`avoid_obstacles_vfh(self, goal_vel_n, goal_vel_e)`** em
`velocity_control.py`. Ele recebe a velocidade ideal (linha reta ao alvo) e devolve
`(vel_n, vel_e, 0.0)` — a velocidade corrigida no **mundo NED**. O terceiro valor é
sempre `0.0` (o VFH vanilla é 2D; o controle de altitude Z é externo).

### 4.0. Entrada e frames
```python
pts = self.lidar_points_3d                  # nuvem JÁ filtrada, no frame do CORPO
xb, yb, zb = pts[:,0], pts[:,1], pts[:,2]
d2d = np.sqrt(xb**2 + yb**2)
valid = (d2d >= 0.3) & (d2d <= DETECT_DIST) & (np.abs(zb) < 2.0)
```
- `self.lidar_points_3d` são pontos no **frame do corpo** do drone (x=frente, y=esquerda).
- Filtra: descarta o que está colado (`< 0.3 m`, ruído/corpo), além do alcance
  (`> 5 m`) ou fora da faixa vertical relevante (`|z| ≥ 2 m`).
- Se não sobrar ponto válido → retorna a velocidade do alvo **sem alterar** (caminho livre).

**Conversão body → NED** (idêntica à do nosso `avoid_obstacles_3d`, p/ consistência):
```python
cos_y, sin_y = cos(self.true_yaw), sin(self.true_yaw)
on = xv*cos_y - yv*sin_y      # componente Norte do obstáculo
oe = xv*sin_y + yv*cos_y      # componente Leste
beta = arctan2(oe, on)        # ângulo do obstáculo no mundo (N=0, E=+90°)
```
Trabalhar em NED (e não no corpo) alinha os obstáculos, o alvo e a saída no mesmo
referencial — simplifica o custo e o setpoint final.

**Direção do alvo** (também NED):
```python
gn = finish_mission_position['N'] - odometry.position[0]
ge = finish_mission_position['E'] - odometry.position[1]
goal_angle = atan2(ge, gn)
```

### 4.1. Histograma polar primário (vetorizado)
```python
mags    = max(A_CONST - B_CONST*dv, 0)        # voto: perto=alto, zera em DETECT_DIST
gamma   = arcsin(min(ROBOT_RADIUS/dv, 1.0))   # meio-alargamento (rad)
centers = (beta / SECTOR) → setor de cada ponto
spreads = ceil(gamma / SECTOR)                # alargamento em nº de setores
for off in range(-max_sp, max_sp+1):
    sel = abs(off) <= spreads
    np.add.at(hist, (centers[sel]+off) % N, mags[sel])
```
- **Magnitude `m = A − B·d`**: obstáculo mais perto vota mais forte. `A = DETECT_DIST`
  garante voto 0 na borda do alcance.
- **Alargamento `γ = asin(r/d)`**: cada obstáculo é "engordado" pelo raio do drone
  (`ROBOT_RADIUS`), espalhando o voto por `±spreads` setores. Quanto mais perto, mais
  largo (um obstáculo a 0,6 m bloqueia quase 180°). É isso que permite tratar o drone
  como um ponto.
- **Vetorização:** em vez de um loop por ponto (lento a 50 Hz com milhares de pontos),
  iteramos sobre os **offsets de setor** (no máx. ~±18) e usamos `np.add.at` para somar
  de uma vez em todos os pontos cujo `spread` alcança aquele offset. `% N` faz o
  *wraparound* circular (setor 71 → 0).

### 4.2. Histograma binário com histerese
```python
prev_bin = getattr(self, '_vfh_binary', zeros(N, bool))
blocked  = where(hist > TAU_HIGH, True, where(hist < TAU_LOW, False, prev_bin))
self._vfh_binary = blocked
```
- Dois limiares (`TAU_HIGH=3`, `TAU_LOW=1.5`): setor só **bloqueia** acima de `TAU_HIGH`
  e só **libera** abaixo de `TAU_LOW`; na zona cinzenta mantém o estado anterior
  (`prev_bin`, guardado em `self._vfh_binary`). Essa **histerese** evita o setor
  "piscar" entre livre/bloqueado de um ciclo pro outro.

### 4.3. Decisão de direção
```python
free = ~blocked
target_sector = setor de goal_angle
if not any(free):            return 0,0,0          # tudo bloqueado → PARA
if free[target_sector]:      chosen = target_sector # alvo livre → vai reto
else:                        chosen = <busca por aberturas>  # ver 4.4
```
Dois atalhos antes da busca: se **nada** está livre, o drone **para** (limitação
proposital do VFH puro — é onde ele trava em becos); se a direção do **alvo já está
livre**, vai reto sem custo.

### 4.4. Candidatos a partir das aberturas
Percorre os setores agrupando **aberturas** (runs contíguos de `free`, com wraparound):
```python
kr = borda direita da abertura;  kl = borda esquerda;  run = largura
if run > S_MAX:        # abertura LARGA: vai perto das bordas (e do alvo, se couber)
    candidates += [(kr + S_MAX//2), (kl - S_MAX//2)]
    if free[target_sector]: candidates += [target_sector]
else:                  # abertura ESTREITA: mira o centro
    candidates += [(kr + run//2)]
```
- **Abertura larga** (`> S_MAX=16` setores ≈ 80°): não vale ir pro meio (desperdício);
  geram-se candidatos a meio-`S_MAX` de cada borda — "rente" ao obstáculo, como o VFH+
  faz pra passar perto sem colar.
- **Abertura estreita:** o único candidato sensato é o **centro** do vão.

### 4.5. Função de custo e escolha
```python
def ang_diff(a,b): return min(|a-b|%N, N-|a-b|%N)     # diferença circular
def cost(c): MU1*ang_diff(c, target) + MU2*ang_diff(c, heading) + MU3*ang_diff(c, prev)
chosen = min(candidates, key=cost)
self._vfh_prev_sector = chosen
```
- **`MU1` (alvo, peso 5):** puxa pra direção do destino — domina.
- **`MU2` (heading, peso 2):** penaliza virar muito em relação ao yaw atual (suavidade).
- **`MU3` (direção anterior, peso 2):** penaliza mudar da escolha do ciclo passado
  (`self._vfh_prev_sector`) — **histerese de direção**, evita oscilar entre dois vales.
- `ang_diff` é circular (a distância entre setor 1 e 71 é 2, não 70).

### 4.6. Velocidade de saída
```python
cn, ce = cos(chosen_angle), sin(chosen_angle)          # versor NED da direção escolhida
proj = on*cn + oe*ce                                    # projeção dos obstáculos à frente
min_ahead = min(proj[proj>0]) or DETECT_DIST
speed = MAX_SPEED * clip(min_ahead / SAFE_DIST, 0.2, 1.0)
if goal_dist < 3.0: speed = min(speed, MAX_SPEED*max(goal_dist/3, 0.15))  # frear no alvo
return speed*cn, speed*ce, 0.0
```
- Projeta os obstáculos na **direção escolhida** (`proj`) e pega o mais próximo à frente
  (`min_ahead`); a velocidade é **reduzida proporcionalmente** quando há obstáculo dentro
  de `SAFE_DIST` (2,5 m), com piso de 20% do `MAX_SPEED`.
- Perto do alvo (`< 3 m`) desacelera pra não passar do ponto.
- Saída final = versor da direção escolhida × velocidade, no **mundo NED**.

### 4.7. Estado entre ciclos
O método guarda dois atributos no nó (ambos com `getattr`/default no 1º ciclo):
- `self._vfh_binary` — histograma binário anterior (histerese do passo 4.2).
- `self._vfh_prev_sector` — direção escolhida no ciclo anterior (histerese do passo 4.5).

---

## 5. Como está integrado (isolado do nosso algoritmo)

Tudo gira em torno da flag `--px4-standard`, e o nosso algoritmo **não é tocado**:

```
--px4-standard
   → run_collision_test.sh exporta USE_VFH=1
   → velocity_control.py: self.use_vfh = True
   → no loop de controle, o desvio chama avoid_obstacles_vfh (VFH+ vanilla)
     em vez de avoid_obstacles_3d (o nosso)
```

- **Sem a flag** (default): `USE_VFH` não setado → `use_vfh = False` →
  roda `avoid_obstacles_3d`, **idêntico ao de sempre**.
- **Mesma entrada/saída** nos dois: nuvem `self.lidar_points_3d` → velocity setpoint.
  Não há mudança de firmware nem de modo offboard.

Arquivo único: `UAV-Collision-Avoidance/src/src_codes/px4_offboard/velocity_control.py`
(método `avoid_obstacles_vfh`).

---

## 6. Como rodar

```bash
# Nosso algoritmo (VFH+ 3D turbinado):
./run_collision_test.sh test.txtmap

# Baseline VFH+ vanilla:
./run_collision_test.sh test.txtmap --px4-standard
```

Para comparar de forma justa, rode o **mesmo mapa** (com paredes `|` e/ou inimigos)
nos dois modos e compare os logs em `logs/`. Esperado: o baseline contorna
obstáculos simples, mas **pode travar** em geometrias côncavas onde o nosso
algoritmo escorrega/sobe e continua.

---

## 7. Referências

- I. Ulrich, J. Borenstein, *"VFH+: Reliable Obstacle Avoidance for Fast Mobile
  Robots"*, ICRA 1998.
- J. Borenstein, Y. Koren, *"The Vector Field Histogram — Fast Obstacle Avoidance
  for Mobile Robots"*, IEEE T-RA, 1991.
- PX4-Avoidance (`local_planner`, 3DVFH*) — a variante 3D com lookahead que
  inspirou este baseline: https://github.com/PX4/PX4-Avoidance
