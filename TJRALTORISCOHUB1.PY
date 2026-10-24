# ============================================================
# HUB 1 - PRINCIPAL | RESGATE DE ALTO RISCO | PYBRICKS 4.x
# A/B = esteiras esquerda/direita
# C/E/D/F = externo esquerdo / central esq / central dir / externo dir
# Hub 2: direito INICIAR, esquerdo PARAR, sem reiniciar programas.
# 1000/1100/1110 => esquerda; 0001/0011/0111 => direita
# 0000 e 1111 => reto; todas as outras combinacoes => PD
# 90 graus: avanca 140 graus de motor, acha preto, ajusta 35.
# Lata: recua, gira 180, aproxima DE RE (garra atras),
#       pede fechamento ao Hub 2, gira de volta 180, segue PD.
# ============================================================
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Color, Stop, Axis
from pybricks.messaging import BLERadio
from pybricks.tools import StopWatch, wait

# ---------- PD / ESTEIRAS ----------
VELOCIDADE_BASE = 450
VELOCIDADE_GAP = 400
VELOCIDADE_MAX = 1100
LIMITE_RE = -460
MIN_BASE = 190
KP_X100 = 240
KD_X100 = 135
FORCA_ESTEIRA_X100 = 320
FILTRO_D = 0.5
ZONA_MORTA = 5
DERIVADA_MAX = 45
CORRECAO_PD_MAX = 300
CORRECAO_MAX = 700
FREIO_EM_CURVA = 50
CICLO_MS = 10

# ---------- CALIBRACAO / CURVAS ----------
PRETO_MAX = 40
BRANCO_MIN = 41
CONTRASTE_MINIMO = 10
AVANCO_90 = 140
VELOCIDADE_AVANCO_90 = 320
VELOCIDADE_DETECTAR_CURVA = 220
VELOCIDADE_GIRO_90 = 270
AJUSTE_90 = 35
VELOCIDADE_AJUSTE_90 = 140
CONFIRMACAO_CURVA = 3
CONFIRMACAO_BRANCO = 2
CONFIRMACAO_PRETO = 2
MIN_GIRO_90_MS = 170
MAX_AVANCO_90_MS = 1800
MAX_GIRO_90_MS = 2600
MAX_AJUSTE_90_MS = 1200
REARME_90_MS = 100
REARME_90_LEITURAS = 3
VELOCIDADE_POS_CURVA = 320
TEMPO_POS_CURVA_MS = 250

# ---------- BASE DE RESGATE (AJUSTAR NA PISTA) ----------
# Graus abaixo sao graus de MOTOR, nao distancia em cm.
RECUO_ANTES_180 = 260
APROXIMAR_LATA_DE_RE = 290
VELOCIDADE_RECUO_RESGATE = 170
VELOCIDADE_GIRO_180 = 230
VELOCIDADE_GIRO_180_FINO = 100
GRAUS_GIRO_180 = 180  # angulo medido pela IMU, do robo
TOLERANCIA_GIRO = 3
TEMPO_MAX_GIRO_180_MS = 10500
MINIMO_DETECTAR_SENTIDO = 3  # graus reais lidos; independente do sinal da IMU
PROGRESSO_MINIMO_GIRO = 2
TEMPO_SEM_PROGRESSO_GIRO_MS = 900
MAX_RECUO_ANGULAR_GIRO = 8
MAX_SALTO_IMU_GIRO = 45  # entre dois ciclos; nao e limite do giro completo
MAX_EXCESSO_GIRO = 10  # nao aceitar sucesso se o robo passar muito do alvo
TOLERANCIA_FINAL_GIRO = 6
ESTABILIZAR_GIRO_MS = 180
INTERVALO_LOG_GIRO_MS = 500
TEMPO_MAX_MOVIMENTO_MS = 4000
TEMPO_MAX_GARRA_MS = 10000  # maior que os 8000 ms locais da garra no Hub 2
LADO_GIRO_RESGATE = 1   # 1=direita, -1=esquerda
# Reduz antes da deteccao de lata em 8cm para dar tempo de frear.
DISTANCIA_REDUZIR = 140  # mm; nao muda limite de deteccao do Hub 2
VELOCIDADE_PERTO_LATA = 210
# Apos confirmar a garra, nao reidentifica a lata ate nova calibracao.

# ---------- BLUETOOTH ----------
CH_TX, CH_RX = 40, 41
ID_H1, ID_H2 = 101, 202
INTERVALO_BLE_MS, TIMEOUT_BLE_MS = 100, 1000
CMD_PARAR, CMD_CALIBRAR, CMD_INICIAR = 0, 1, 2
AGUARDANDO, CALIBRANDO, ERRO_CAL, PRONTO = 0, 1, 2, 3
CORRENDO, PARADO, FALHA, SEM_SINAL = 4, 5, 6, 7
LIVRE, LATA, OBSTACULO, INDEFINIDO = 0, 1, 2, 3
GARRA_NADA, GARRA_FECHAR = 0, 1
GARRA_OCIOSA, GARRA_MOVENDO, GARRA_OK, GARRA_ERRO = 0, 1, 2, 3

# ---------- ESTADOS DE NAVEGACAO ----------
PD, AVANCANDO_90, GIRANDO_90, AJUSTANDO_90 = 0, 1, 2, 3
RES_RECUANDO, RES_GIRANDO_IDA, RES_APROXIMANDO = 4, 5, 6
RES_FECHANDO, RES_GIRANDO_VOLTA = 7, 8
ESQ, DIR = -1, 1
RETO, SEGUIR_PD = 0, 2
ACOES = (RETO, DIR, SEGUIR_PD, DIR,
         SEGUIR_PD, SEGUIR_PD, SEGUIR_PD, DIR,
         ESQ, SEGUIR_PD, SEGUIR_PD, SEGUIR_PD,
         ESQ, SEGUIR_PD, ESQ, RETO)

# Botoes para cima; frente do hub voltada para tras dos sensores de linha.
hub = PrimeHub(top_side=Axis.Z, front_side=-Axis.X)
esq = dir = None
sensor_f = sensor_d = sensor_e = sensor_c = None
radio = BLERadio(broadcast_channel=CH_TX, observe_channels=[CH_RX])
clock = StopWatch()


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def inicializar_dispositivos(recriar=False):
    global esq, dir, sensor_f, sensor_d, sensor_e, sensor_c
    if recriar:
        # Reabrir os dispositivos somente apos novo comando do Hub 2.
        for motor in (esq, dir):
            if motor is not None:
                try:
                    motor.close()
                except Exception:
                    pass
        esq = dir = None
        sensor_f = sensor_d = sensor_e = sensor_c = None
    if esq is None:
        esq = Motor(Port.A, Direction.CLOCKWISE)
    if dir is None:
        dir = Motor(Port.B, Direction.COUNTERCLOCKWISE)
    if sensor_f is None:
        sensor_f = ColorSensor(Port.C)  # externo esquerdo
    if sensor_d is None:
        sensor_d = ColorSensor(Port.E)  # central esquerdo
    if sensor_e is None:
        sensor_e = ColorSensor(Port.D)  # central direito
    if sensor_c is None:
        sensor_c = ColorSensor(Port.F)  # externo direito


def drive(l, r):
    esq.run(clamp(l, LIMITE_RE, VELOCIDADE_MAX))
    dir.run(clamp(r, LIMITE_RE, VELOCIDADE_MAX))


def brake():
    erro = None
    for motor in (esq, dir):
        if motor is not None:
            try:
                motor.brake()
            except Exception as falha:
                erro = falha
    if erro is not None:
        raise erro


def brake_safe():
    try:
        brake()
    except Exception:
        pass


def is_black(x):
    return 0 <= x <= PRETO_MAX


def is_white(x):
    return BRANCO_MIN <= x <= 100


def read_line():
    return (sensor_f.reflection(), sensor_d.reflection(),
            sensor_e.reflection(), sensor_c.reflection())


def classify_line(c, e, d, f):
    mask = (8 * int(is_black(c)) + 4 * int(is_black(e)) +
            2 * int(is_black(d)) + int(is_black(f)))
    return ACOES[mask], mask


def calibrate():
    l = r = 0
    for _ in range(10):
        l += sensor_d.reflection()
        r += sensor_e.reflection()
        wait(12)
    l /= 10
    r /= 10
    print('CAL E/D:', l, r)
    if not ((is_black(l) and is_white(r)) or
            (is_white(l) and is_black(r))):
        return None
    if abs(l-r) < CONTRASTE_MINIMO:
        return None
    return min(l, r), max(l, r)


def normalize(v, low, high):
    return clamp((v-low)*100/max(1, high-low), 0, 100)


def pd(erro, anterior, filtrada, dt):
    if abs(erro) < ZONA_MORTA:
        erro = 0
    delta = 0 if anterior is None else clamp(
        (erro-anterior)*CICLO_MS/max(1, dt), -DERIVADA_MAX, DERIVADA_MAX)
    filtrada = FILTRO_D*filtrada + (1-FILTRO_D)*delta
    termo = clamp((KP_X100*erro + KD_X100*filtrada)/100,
                  -CORRECAO_PD_MAX, CORRECAO_PD_MAX)
    return clamp(termo*FORCA_ESTEIRA_X100/100,
                 -CORRECAO_MAX, CORRECAO_MAX), erro, filtrada


def both_angle(angle, speed):
    brake()
    esq.run_angle(speed, angle, then=Stop.BRAKE, wait=False)
    dir.run_angle(speed, angle, then=Stop.BRAKE, wait=False)


def start_turn(side, speed):
    # IMU: heading positivo = sentido horario/direita.
    drive(side * speed, -side * speed)


class GiroResgate:
    # Controla somente os 180 do resgate, sem bloquear BLE ou botoes.
    # O sinal e aprendido no giro real; nao depende do lado dos motores.
    def iniciar(self, side, now):
        self.side = side
        self.heading_start = hub.imu.heading()
        self.delta_anterior = 0
        self.sentido = 0
        self.progresso = 0
        self.marca_progresso = 0
        self.inicio = self.ultimo_progresso = now
        self.ultimo_log = now
        self.parou_em = None
        self.erro = ''
        print('GIRO 180: INICIO, HEADING', self.heading_start,
              'LADO MOTOR', side)
        start_turn(side, VELOCIDADE_GIRO_180)

    def falhar(self, motivo):
        brake()
        self.erro = motivo
        print('GIRO 180: FALHA', motivo, 'PROGRESSO', self.progresso)
        return -1

    def atualizar(self, now):
        # Retorna 0 durante o giro, 1 ao concluir e -1 se falhar.
        if not hub.imu.ready():
            return self.falhar('IMU PERDEU A REFERENCIA')
        delta = hub.imu.heading() - self.heading_start
        if delta != delta or abs(delta-self.delta_anterior) > MAX_SALTO_IMU_GIRO:
            return self.falhar('SALTO NA LEITURA DA IMU')
        self.delta_anterior = delta
        if not self.sentido and abs(delta) >= MINIMO_DETECTAR_SENTIDO:
            self.sentido = 1 if delta > 0 else -1
            print('GIRO 180: SENTIDO IMU', self.sentido)
        self.progresso = self.sentido*delta

        if self.progresso < self.marca_progresso-MAX_RECUO_ANGULAR_GIRO:
            return self.falhar('IMU INVERTEU DURANTE O GIRO')
        if self.progresso > GRAUS_GIRO_180+MAX_EXCESSO_GIRO:
            return self.falhar('GIRO PASSOU DO ALVO')
        if self.progresso >= self.marca_progresso+PROGRESSO_MINIMO_GIRO:
            self.marca_progresso = self.progresso
            self.ultimo_progresso = now
        if now-self.inicio > TEMPO_MAX_GIRO_180_MS:
            return self.falhar('TEMPO MAXIMO DO GIRO')

        if self.parou_em is not None:
            # Mantem os encoders na posicao da parada, reduzindo o embalo.
            if abs(self.progresso-self.progresso_parado) > 1:
                self.progresso_parado = self.progresso
                self.parou_em = now
            if now-self.parou_em >= ESTABILIZAR_GIRO_MS:
                if abs(GRAUS_GIRO_180-self.progresso) > TOLERANCIA_FINAL_GIRO:
                    return self.falhar('GIRO NAO ESTABILIZOU NO ALVO')
                print('GIRO 180: CONCLUIDO, ANGULO', self.progresso)
                return 1
            return 0

        if now-self.ultimo_progresso >= TEMPO_SEM_PROGRESSO_GIRO_MS:
            return self.falhar('IMU SEM PROGRESSO; CONFIRA MONTAGEM DO HUB')
        if self.progresso >= GRAUS_GIRO_180-TOLERANCIA_GIRO:
            esq.hold()
            dir.hold()
            self.parou_em = now
            self.progresso_parado = self.progresso
            return 0

        restante = GRAUS_GIRO_180-self.progresso
        speed = clamp(restante*4, VELOCIDADE_GIRO_180_FINO,
                      VELOCIDADE_GIRO_180)
        start_turn(self.side, speed)
        if now-self.ultimo_log >= INTERVALO_LOG_GIRO_MS:
            print('GIRO 180: ANGULO', self.progresso, 'FALTAM', restante)
            self.ultimo_log = now
        return 0


def finish_90(side):
    brake()
    esq.run_angle(VELOCIDADE_AJUSTE_90, side*AJUSTE_90,
                  then=Stop.BRAKE, wait=False)
    dir.run_angle(VELOCIDADE_AJUSTE_90, -side*AJUSTE_90,
                  then=Stop.BRAKE, wait=False)


def alarm(texto, codigo='X'):
    brake()
    hub.light.on(Color.RED)
    hub.display.char(codigo)
    print('PARADO -', texto, '- DIREITO PARA RETOMAR')


def principal():
    state = AGUARDANDO
    phase = PD
    calibrated = False
    low, high = 0, 100
    token_seen = -1
    token_rx = -1
    last_seq = -1
    last_rx = -10000
    seq_tx = 0
    last_tx = -INTERVALO_BLE_MS
    object_type = LIVRE
    dist1, dist2 = 2000, 2000
    grip_cmd = GARRA_NADA
    grip_event = 0
    grip_ack = -1
    grip_status = GARRA_OCIOSA
    lata_capturada = False
    t_phase = 0
    giro_resgate = GiroResgate()
    t_pd = 0
    error_last = None
    d_filter = 0
    side_candidate, candidate_count = 0, 0
    curve_locked, unlock_count = False, 0
    curve_side, t_curve_end = 0, -10000
    saw_white, nwhite, nblack = False, 0, 0
    gap_heading, on_gap = None, False
    object_printed = False
    precisa_inicializar = True
    precisa_recriar = False
    ultimo_erro = None

    def preparar_dispositivos():
        nonlocal precisa_inicializar, precisa_recriar
        precisa_inicializar = False
        inicializar_dispositivos(precisa_recriar)
        precisa_recriar = False

    def tratar_erro(erro, origem='EXECUCAO'):
        nonlocal state, phase, grip_cmd, calibrated, error_last, d_filter
        nonlocal side_candidate, candidate_count, on_gap, gap_heading
        nonlocal precisa_recriar, ultimo_erro
        if state == CALIBRANDO:
            calibrated = False
        state = FALHA if calibrated else ERRO_CAL
        phase = PD
        grip_cmd = GARRA_NADA
        error_last, d_filter = None, 0
        side_candidate, candidate_count = 0, 0
        on_gap, gap_heading = False, None
        if isinstance(erro, OSError) and origem == 'EXECUCAO':
            precisa_recriar = True
        brake_safe()
        mensagem = origem+': '+repr(erro)
        if mensagem != ultimo_erro:
            try:
                print('HUB 1 PARADO POR ERRO:', mensagem,
                      '- CORRIJA E APERTE DIREITO NO HUB 2')
            except Exception:
                pass
            ultimo_erro = mensagem
        for operacao, valor in ((hub.light.on, Color.RED),
                                (hub.display.char, 'E')):
            try:
                operacao(valor)
            except Exception:
                pass

    brake_safe()
    try:
        hub.speaker.volume(85)
        hub.light.on(Color.BLUE)
        hub.display.char('A')
        print('HUB 1: ESPERANDO CALIBRACAO PELO HUB 2')
    except Exception as erro:
        tratar_erro(erro)

    while True:
        cycle_start = clock.time()
        try:
            origem_erro = 'BLE_RX'
            pkt = radio.observe(CH_RX)
            origem_erro = 'EXECUCAO'
            if (isinstance(pkt, tuple) and len(pkt) == 6
                    and pkt[0] == ID_H2):
                token_rx = pkt[1]
                seq = pkt[2]
                packed_object = pkt[3]
                if seq != last_seq:
                    last_seq = seq
                    last_rx = cycle_start
                object_type = packed_object % 8
                grip_status = (packed_object // 8) % 4
                grip_ack = packed_object // 32
                dist1, dist2 = pkt[4], pkt[5]
                if token_rx != token_seen:
                    token_seen = token_rx
                    command = token_rx % 4
                    if command == CMD_CALIBRAR:
                        calibrated = False
                    if command in (CMD_CALIBRAR, CMD_INICIAR):
                        preparar_dispositivos()
                        ultimo_erro = None
                    if command == CMD_PARAR:
                        brake()
                        grip_cmd = GARRA_NADA
                        phase = PD
                        state = PARADO
                        error_last, d_filter = None, 0
                        hub.display.char('P')
                        hub.light.on(Color.YELLOW)
                        print('PARADO PELO HUB 2')
                    elif command == CMD_CALIBRAR:
                        brake()
                        grip_cmd = GARRA_NADA
                        calibrated = False
                        lata_capturada = False
                        phase = PD
                        state = CALIBRANDO
                    elif command == CMD_INICIAR:
                        if calibrated:
                            brake()
                            grip_cmd = GARRA_NADA
                            phase = PD
                            state = CORRENDO
                            t_pd = clock.time()
                            error_last, d_filter = None, 0
                            side_candidate, candidate_count = 0, 0
                            curve_locked, unlock_count = False, 0
                            on_gap = False
                            object_printed = False
                            hub.display.char('R')
                            hub.light.on(Color.GREEN)
                            print('DIREITO: COMECOU')
                        else:
                            state = ERRO_CAL
                            print('CALIBRE ANTES DE INICIAR')

            if precisa_inicializar:
                preparar_dispositivos()
            now = clock.time()
            connected = now - last_rx <= TIMEOUT_BLE_MS
            if not connected and state not in (AGUARDANDO, SEM_SINAL):
                brake()
                grip_cmd = GARRA_NADA
                state = SEM_SINAL
                phase = PD
                hub.light.on(Color.YELLOW)
                hub.display.char('A')
                print('BLE PERDIDO - AGUARDANDO NOVO DIREITO')

            if state == CALIBRANDO and connected:
                brake()
                result = calibrate()
                if result is None:
                    state = ERRO_CAL
                    hub.display.char('X')
                    hub.light.on(Color.RED)
                    hub.speaker.beep(450, 150)
                    wait(110)
                    hub.speaker.beep(450, 150)
                    print('ERRO DE CALIBRACAO - DIREITO REPETE')
                else:
                    low, high = result
                    calibrated = True
                    state = PRONTO
                    hub.display.char('O')
                    hub.light.on(Color.GREEN)
                    hub.speaker.beep(1200, 160)
                    print('CALIBRADO - DIREITO INICIA')

            elif state == CORRENDO and connected:
                now = clock.time()
                # Sensor 2 e falhas de leitura tem prioridade em toda fase.
                # Somente a deteccao da lata e ignorada durante a captura.
                rescuing = phase >= RES_RECUANDO
                if object_type in (OBSTACULO, INDEFINIDO):
                    brake()
                    grip_cmd = GARRA_NADA
                    if rescuing:
                        phase = PD
                        state = FALHA
                        alarm('OBSTACULO/LEITURA INVALIDA NO RESGATE; '
                              'REPOSICIONE NA LINHA')
                    elif object_type == INDEFINIDO:
                        phase = PD
                        state = FALHA
                        alarm('LEITURA INVALIDA DOS ULTRAS; CORRIJA ANTES DE RETOMAR')
                    elif phase != PD:
                        phase = PD
                        state = FALHA
                        alarm('OBSTACULO NO MEIO DA CURVA')
                    elif not object_printed:
                        object_printed = True
                        hub.display.char('O')
                        hub.light.on(Color.YELLOW)
                        print('OBJETO ALTO/INDEFINIDO - PARADO')
                    error_last, d_filter = None, 0
                    t_pd = now
                else:
                    if object_printed:
                        object_printed = False
                        error_last, d_filter = None, 0
                        hub.display.char('R')
                        hub.light.on(Color.GREEN)

                    # -- Comeca o resgate apenas quando sensor de baixo
                    # ve lata e o de cima NAO ve objeto.
                    if (phase == PD and object_type == LATA
                            and not lata_capturada):
                        brake()
                        phase = RES_RECUANDO
                        t_phase = now
                        side_candidate, candidate_count = 0, 0
                        on_gap, gap_heading = False, None
                        both_angle(-RECUO_ANTES_180, VELOCIDADE_RECUO_RESGATE)
                        hub.display.char('L')
                        print('LATA: RECUAR, GIRAR, APROXIMAR, FECHAR')

                    # -- Manobra de resgate nao bloqueante.
                    elif phase == RES_RECUANDO:
                        if esq.done() and dir.done():
                            brake()
                            if not hub.imu.ready():
                                state = FALHA
                                phase = PD
                                alarm('IMU NAO PRONTA PARA 180')
                            else:
                                phase = RES_GIRANDO_IDA
                                t_phase = now
                                giro_resgate.iniciar(LADO_GIRO_RESGATE, now)
                        elif now - t_phase > TEMPO_MAX_MOVIMENTO_MS:
                            state = FALHA
                            phase = PD
                            alarm('RECUO DA LATA EXCEDEU TEMPO')

                    elif phase == RES_GIRANDO_IDA or phase == RES_GIRANDO_VOLTA:
                        resultado_giro = giro_resgate.atualizar(now)
                        if resultado_giro == 1:
                            brake()
                            if phase == RES_GIRANDO_IDA:
                                phase = RES_APROXIMANDO
                                t_phase = now
                                both_angle(-APROXIMAR_LATA_DE_RE,
                                           VELOCIDADE_RECUO_RESGATE)
                                print('180 OK: APROXIMANDO GARRA DE RE')
                            else:
                                phase = PD
                                t_curve_end = now
                                curve_locked = True
                                unlock_count = 0
                                side_candidate, candidate_count = 0, 0
                                on_gap, gap_heading = False, None
                                error_last, d_filter = None, 0
                                t_pd = now
                                hub.display.char('R')
                                hub.light.on(Color.GREEN)
                                print('LATA CAPTURADA! PD REATIVADO')
                        elif resultado_giro == -1:
                            grip_cmd = GARRA_NADA
                            state = FALHA
                            phase = PD
                            alarm('GIRO 180: '+giro_resgate.erro, 'G')

                    elif phase == RES_APROXIMANDO:
                        if esq.done() and dir.done():
                            brake()
                            # Diferente do ultimo evento do Hub 2, inclusive
                            # quando apenas este Hub 1 acabou de reiniciar.
                            grip_event = (grip_ack % 31) + 1
                            grip_cmd = GARRA_FECHAR
                            phase = RES_FECHANDO
                            t_phase = now
                            print('COMANDO GARRA: FECHAR AMBOS MOTORES')
                        elif now - t_phase > TEMPO_MAX_MOVIMENTO_MS:
                            state = FALHA
                            phase = PD
                            alarm('APROXIMACAO DE RE EXCEDEU TEMPO')

                    elif phase == RES_FECHANDO:
                        brake()
                        if grip_ack == grip_event and grip_status == GARRA_OK:
                            grip_cmd = GARRA_NADA
                            # Ja fechou: uma parada no giro de volta nao deve
                            # permitir outro resgate da mesma lata.
                            lata_capturada = True
                            if not hub.imu.ready():
                                state = FALHA
                                phase = PD
                                alarm('IMU NAO PRONTA PARA VOLTAR')
                            else:
                                phase = RES_GIRANDO_VOLTA
                                t_phase = now
                                giro_resgate.iniciar(-LADO_GIRO_RESGATE, now)
                                print('GARRA FECHADA: GIRANDO 180 DE VOLTA')
                        elif (grip_ack == grip_event and grip_status == GARRA_ERRO):
                            grip_cmd = GARRA_NADA
                            state = FALHA
                            phase = PD
                            alarm('FALHA NA GARRA')
                        elif now-t_phase > TEMPO_MAX_GARRA_MS:
                            grip_cmd = GARRA_NADA
                            state = FALHA
                            phase = PD
                            alarm('SEM RESPOSTA DA GARRA')

                    # -- Curvas: avanca, busca a linha e conclui o ajuste.
                    elif phase == AVANCANDO_90:
                        if esq.done() and dir.done():
                            brake()
                            phase = GIRANDO_90
                            t_phase = now
                            saw_white, nwhite, nblack = False, 0, 0
                            start_turn(curve_side, VELOCIDADE_GIRO_90)
                        elif now-t_phase > MAX_AVANCO_90_MS:
                            phase = PD
                            state = FALHA
                            alarm('AVANCO 90 EXCEDEU TEMPO')

                    elif phase == GIRANDO_90:
                        central = (sensor_d.reflection() if curve_side == ESQ
                                   else sensor_e.reflection())
                        nwhite = nwhite+1 if is_white(central) else 0
                        if nwhite >= CONFIRMACAO_BRANCO:
                            saw_white = True
                        nblack = (nblack+1 if saw_white and
                                  now-t_phase >= MIN_GIRO_90_MS and is_black(central)
                                  else 0)
                        if nblack >= CONFIRMACAO_PRETO:
                            phase = AJUSTANDO_90
                            t_phase = now
                            finish_90(curve_side)
                        elif now-t_phase > MAX_GIRO_90_MS:
                            phase = PD
                            state = FALHA
                            alarm('LINHA NAO ENCONTRADA NA CURVA')

                    elif phase == AJUSTANDO_90:
                        if esq.done() and dir.done():
                            brake()
                            phase = PD
                            t_curve_end = now
                            curve_locked = True
                            unlock_count = 0
                            error_last, d_filter = None, 0
                            t_pd = now
                        elif now-t_phase > MAX_AJUSTE_90_MS:
                            phase = PD
                            state = FALHA
                            alarm('AJUSTE 35 EXCEDEU TEMPO')

                    elif phase == PD and now-t_pd >= CICLO_MS:
                        dt = now-t_pd
                        t_pd = now
                        c,e,d,f = read_line()
                        action, mask = classify_line(c,e,d,f)
                        if curve_locked:
                            # Rearma ao reencontrar a linha entre os externos.
                            # Branco total (gap) nao comprova fim da curva.
                            unlock_count = (unlock_count+1
                                            if is_white(c) and is_white(f) and
                                            (is_black(e) or is_black(d)) else 0)
                            if (unlock_count >= REARME_90_LEITURAS
                                    and now-t_curve_end >= REARME_90_MS):
                                curve_locked = False
                        candidate = (action if action in (ESQ,DIR)
                                     and not curve_locked else 0)
                        if candidate != 0 and candidate == side_candidate:
                            candidate_count += 1
                        else:
                            side_candidate = candidate
                            candidate_count = 1 if candidate else 0
                        if candidate and candidate_count >= CONFIRMACAO_CURVA:
                            phase = AVANCANDO_90
                            curve_side = candidate
                            t_phase = now
                            error_last, d_filter = None, 0
                            side_candidate, candidate_count = 0, 0
                            on_gap, gap_heading = False, None
                            print('CURVA 90:', 'ESQUERDA' if curve_side == ESQ
                                  else 'DIREITA', 'MASCARA', mask)
                            both_angle(AVANCO_90, VELOCIDADE_AVANCO_90)
                        elif mask == 0:
                            if not on_gap:
                                gap_heading = (hub.imu.heading()
                                               if hub.imu.ready() else None)
                            on_gap = True
                            if gap_heading is not None and hub.imu.ready():
                                correction = clamp((gap_heading-hub.imu.heading())*7,
                                                   -80,80)
                                drive(VELOCIDADE_GAP+correction,
                                      VELOCIDADE_GAP-correction)
                            else:
                                drive(VELOCIDADE_GAP,VELOCIDADE_GAP)
                            error_last, d_filter = None,0
                        elif mask == 15:
                            drive(VELOCIDADE_BASE,VELOCIDADE_BASE)
                            on_gap = False
                            error_last,d_filter=None,0
                        else:
                            on_gap=False
                            ne=normalize(e,low,high)
                            nd=normalize(d,low,high)
                            correction,error_last,d_filter=pd(
                                ne-nd,error_last,d_filter,dt)
                            base = (VELOCIDADE_POS_CURVA if
                                    now-t_curve_end < TEMPO_POS_CURVA_MS
                                    else VELOCIDADE_BASE)
                            if candidate:
                                base=min(base,VELOCIDADE_DETECTAR_CURVA)
                            if (not lata_capturada and
                                    0 < dist1 <= DISTANCIA_REDUZIR):
                                base=min(base,VELOCIDADE_PERTO_LATA)
                            base=clamp(base-abs(correction)*FREIO_EM_CURVA/100,
                                       MIN_BASE,VELOCIDADE_MAX)
                            drive(base+correction,base-correction)

            else:
                brake()

        except Exception as erro:
            tratar_erro(erro, origem_erro)

        # 3. Enviar estado e comando da garra pelo MESMO protocolo
        # de 4 numeros, compactando as informacoes no estado.
        now=clock.time()
        if now-last_tx >= INTERVALO_BLE_MS:
            seq_tx=(seq_tx+1)%100
            packed_state = state + 16*(grip_cmd+4*grip_event)
            try:
                radio.broadcast((ID_H1, token_rx, packed_state, seq_tx))
            except Exception as erro:
                tratar_erro(erro, 'BLE_TX')
            last_tx=now
        wait(max(1,CICLO_MS-(clock.time()-cycle_start)))


try:
    principal()
finally:
    # Apenas parada deliberada/interrupcao do sistema chega aqui.
    brake_safe()
    try:
        radio.broadcast(None)
    except Exception:
        pass
    try:
        hub.light.on(Color.RED)
    except Exception:
        pass
