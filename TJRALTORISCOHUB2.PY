# ============================================================
# HUB 2 - AUXILIAR | RESGATE DE ALTO RISCO | PYBRICKS 4.x
# B: sensorultra 1 / inferior (lata, 8 cm)
# A: sensorultra 2 / superior (obstaculo, 10 cm; prioritario)
# C e E: motores da garra sincronizados por encoder
# ESQUERDA = PARAR | DIREITA = INICIAR/REINICIAR
# Primeiro inicio do programa pede calibracao ao Hub 1.
# ============================================================
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, UltrasonicSensor
from pybricks.parameters import Port, Color, Button, Stop
from pybricks.messaging import BLERadio
from pybricks.tools import StopWatch, wait

# -------- DISTANCIAS EM MILIMETROS --------
LIMITE_LATA = 80
LIMITE_OBSTACULO = 100
CONFIRMACOES = 3
# O sensor retorna 2000 normalmente quando nao recebe eco.
# Zero e a sentinela transmitida quando a leitura falha.
DISTANCIA_SEM_ECO = 2000

# -------- GARRA: AJUSTE PARA SEU MECANISMO --------
# Nao ha homing! Posicione a garra ABERTA antes de iniciar.
ANGULO_FECHAR = 360 * 5         # 5 voltas por motor; ajuste confirmado no robo
VELOCIDADE_GARRA = 300          # graus/s; ajuste confirmado no robo
SINAL_MOTOR_C = 1              # inverter se fechar no sentido errado
SINAL_MOTOR_E = -1             # inverter se fechar no sentido errado
TIMEOUT_GARRA_MS = 8000         # 1800 graus a 300 graus/s: 6 s + margem
# BRAKE para nao insistir contra batente mecanico.

# -------- RADIO --------
CH_TX, CH_RX = 41, 40
ID_H1, ID_H2 = 101, 202
INTERVALO_BLE_MS = 100
TIMEOUT_BLE_MS = 1000
CMD_PARAR, CMD_CALIBRAR, CMD_INICIAR = 0, 1, 2
AGUARDANDO, CALIBRANDO, ERRO_CAL, PRONTO = 0, 1, 2, 3
CORRENDO, PARADO, FALHA, SEM_SINAL = 4, 5, 6, 7
LIVRE, LATA, OBSTACULO, INDEFINIDO = 0, 1, 2, 3
GARRA_NADA, GARRA_FECHAR = 0, 1
GARRA_OCIOSA, GARRA_MOVENDO, GARRA_OK, GARRA_ERRO = 0, 1, 2, 3

hub = PrimeHub()
radio = BLERadio(broadcast_channel=CH_TX, observe_channels=[CH_RX])
clock = StopWatch()
# Dispositivo ausente nao pode encerrar o programa antes do loop.
us_inf = us_sup = garra_c = garra_e = None
ALVO_FECHAR_C = ALVO_FECHAR_E = None


def inicializar_dispositivos():
    global us_inf, us_sup, garra_c, garra_e
    global ALVO_FECHAR_C, ALVO_FECHAR_E
    if us_inf is None:
        us_inf = UltrasonicSensor(Port.B)
    if us_sup is None:
        us_sup = UltrasonicSensor(Port.A)
    if garra_c is None:
        garra_c = Motor(Port.C)
    if garra_e is None:
        garra_e = Motor(Port.E)
    if ALVO_FECHAR_C is None or ALVO_FECHAR_E is None:
        # So salvar quando AMBAS leituras funcionarem. Nunca recalcular
        # os alvos depois de uma captura interrompida com a garra parcial.
        aberto_c, aberto_e = garra_c.angle(), garra_e.angle()
        ALVO_FECHAR_C = aberto_c + SINAL_MOTOR_C*ANGULO_FECHAR
        ALVO_FECHAR_E = aberto_e + SINAL_MOTOR_E*ANGULO_FECHAR


def brake_grip():
    erro = None
    for motor in (garra_c, garra_e):
        if motor is not None:
            try:
                motor.brake()
            except Exception as exc:
                if erro is None:
                    erro = exc
    if erro is not None:
        raise erro


def brake_grip_seguro():
    try:
        brake_grip()
    except Exception:
        pass


def valid_distance(distance):
    return (isinstance(distance, (int, float)) and
            not isinstance(distance, bool) and
            0 < distance <= DISTANCIA_SEM_ECO)


def read_distance(sensor):
    try:
        distance = sensor.distance()
    except OSError:
        return 0
    return distance if valid_distance(distance) else 0


def classify(d1, d2):
    # Sensor 2 tem prioridade mesmo se o sensor de lata falhar.
    if valid_distance(d2) and d2 <= LIMITE_OBSTACULO:
        return OBSTACULO
    if not valid_distance(d1) or not valid_distance(d2):
        return INDEFINIDO
    low = 0 < d1 <= LIMITE_LATA
    if low:
        return LATA
    return LIVRE


def start_close():
    # Alvos absolutos calculados uma vez, a partir da garra aberta.
    brake_grip()
    garra_c.run_target(VELOCIDADE_GARRA, ALVO_FECHAR_C,
                       then=Stop.BRAKE, wait=False)
    garra_e.run_target(VELOCIDADE_GARRA, ALVO_FECHAR_E,
                       then=Stop.BRAKE, wait=False)


def principal():
    contador = 1
    comando = CMD_CALIBRAR
    token = contador*4 + comando
    seq_tx = 0
    last_tx = -INTERVALO_BLE_MS
    estado_principal = -1
    last_h1_seq = -1
    last_rx = -10000
    last_left = False
    last_right = False
    last_screen = ''
    objeto = LIVRE
    candidato = LIVRE
    repeticoes = 0
    d1,d2=2000,2000
    grip_state = GARRA_OCIOSA
    grip_event_last = 0
    t_grip = 0
    ultimo_status = -10000
    falha_local = False
    inicializar_pendente = True
    calibracao_confirmada = False
    connected = False

    def falhar(erro):
        nonlocal falha_local, contador, comando, token, last_tx
        nonlocal grip_state, objeto, candidato, repeticoes, d1, d2
        nonlocal estado_principal, last_screen
        nova_falha = not falha_local
        falha_local = True
        grip_state = GARRA_ERRO
        objeto = candidato = INDEFINIDO
        repeticoes = 0
        d1,d2 = 0,0
        brake_grip_seguro()
        if nova_falha:
            # Um unico novo PARAR por falha; os pacotes seguintes repetem
            # esse token ate uma nova borda do botao DIREITO.
            contador = contador%60 + 1
            comando = CMD_PARAR
            token = contador*4 + comando
            estado_principal = -1
            last_tx = -INTERVALO_BLE_MS
            last_screen = ''
            try:
                print('HUB 2 PARADO -', erro, '- DIREITO PARA RETOMAR')
            except Exception:
                pass

    try:
        hub.display.char('C')
        hub.light.on(Color.YELLOW)
        print('HUB2 ATIVO: ESQUERDO PARA / DIREITO INICIA')
    except Exception as erro:
        falhar(erro)

    while True:
        now = clock.time()
        try:
            # Ler antes de receber pedidos impede iniciar com PARAR pressionado.
            keys = hub.buttons.pressed()
            left = Button.LEFT in keys
            right = Button.RIGHT in keys

            # 1. Receber estado e comandos para a garra.
            pkt = radio.observe(CH_RX)
            if (isinstance(pkt, tuple) and len(pkt) == 4
                    and pkt[0] == ID_H1):
                ack_token = pkt[1]
                packed_state = pkt[2]
                seq_h1 = pkt[3]
                if seq_h1 != last_h1_seq:
                    last_h1_seq = seq_h1
                    last_rx = now
                decoded_state = packed_state % 16
                grip_command = (packed_state//16)%4
                grip_event = packed_state//64
                if ack_token == token:
                    estado_principal = decoded_state
                    if decoded_state in (PRONTO, CORRENDO):
                        calibracao_confirmada = True
                    elif decoded_state in (AGUARDANDO, CALIBRANDO, ERRO_CAL):
                        calibracao_confirmada = False

                # So executar com o token atual, largada local e sem falha.
                # Novo event id evita acionamento repetido pelos broadcasts.
                if (ack_token == token and
                        grip_state == GARRA_MOVENDO and
                        (decoded_state != CORRENDO or
                         grip_command != GARRA_FECHAR)):
                    brake_grip()
                    grip_state = GARRA_ERRO
                    print('GARRA PARADA POR COMANDO DO HUB 1')
                elif (not falha_local and not inicializar_pendente and
                        now-last_rx <= TIMEOUT_BLE_MS and
                        ack_token == token and
                        comando == CMD_INICIAR and not left and
                        decoded_state == CORRENDO and
                        grip_command == GARRA_FECHAR and
                        grip_event != grip_event_last):
                    grip_event_last = grip_event
                    grip_state = GARRA_MOVENDO
                    t_grip = now
                    start_close()
                    print('GARRA: FECHANDO OS DOIS MOTORES')

            connected = now-last_rx <= TIMEOUT_BLE_MS
            if not connected:
                estado_principal = -1
                if (grip_state == GARRA_MOVENDO or
                        (last_rx >= 0 and comando == CMD_INICIAR)):
                    raise OSError('BLE PERDIDO')

            # 2. Botao esquerdo prioritario; direito so envia com link valido.
            click_left = left and not last_left
            click_right = right and not last_right
            last_left = left
            last_right = right
            if click_left:
                if grip_state == GARRA_MOVENDO:
                    brake_grip()
                    grip_state = GARRA_ERRO
                contador = contador%60 + 1
                comando = CMD_PARAR
                token = contador*4 + comando
                estado_principal = -1
                last_tx = -INTERVALO_BLE_MS
                print('ESQUERDO: PARAR')
            elif click_right and not left:
                if not connected or estado_principal in (-1,CALIBRANDO):
                    print('ESPERE CONEXAO/CALIBRACAO')
                elif estado_principal == CORRENDO and not falha_local:
                    print('JA CORRENDO')
                else:
                    brake_grip()
                    if grip_state == GARRA_MOVENDO:
                        grip_state = GARRA_ERRO
                    contador = contador%60 + 1
                    comando = (CMD_CALIBRAR if not calibracao_confirmada or
                               estado_principal in (AGUARDANDO,ERRO_CAL)
                               else CMD_INICIAR)
                    token = contador*4 + comando
                    estado_principal = -1
                    last_tx = -INTERVALO_BLE_MS
                    # So este clique libera nova tentativa. Se o defeito
                    # persistir, o mesmo ciclo volta ao estado de falha.
                    falha_local = False
                    inicializar_pendente = True
                    candidato, repeticoes = LIVRE, 0
                    print('DIREITO: NOVA CALIBRACAO/LARGADA')

            if falha_local:
                brake_grip_seguro()
            else:
                if inicializar_pendente:
                    inicializar_pendente = False
                    inicializar_dispositivos()
                    brake_grip()

                # 3. Lata exige 3 confirmacoes; perigo e saida sao imediatos.
                d1=read_distance(us_inf)
                wait(25)
                d2=read_distance(us_sup)
                if not valid_distance(d1) or not valid_distance(d2):
                    raise OSError('LEITURA INVALIDA DOS ULTRASSONICOS')
                current=classify(d1,d2)
                if objeto == INDEFINIDO:
                    # So retirar a sentinela depois de duas leituras validas;
                    # nao transmitir INICIAR junto do erro do ciclo anterior.
                    objeto = LIVRE
                if current == candidato:
                    repeticoes+=1
                else:
                    candidato=current
                    repeticoes=1
                change_now = (current in (OBSTACULO, INDEFINIDO) or
                              (objeto == LATA and current != LATA) or
                              repeticoes >= CONFIRMACOES)
                if change_now and current!=objeto:
                    objeto=current
                    print('DETECCAO:',objeto,'DIST:',d1,d2)

                # 4. Garra: concluir quando AMBOS motores terminarem.
                now=clock.time()
                if grip_state == GARRA_MOVENDO:
                    if garra_c.stalled() or garra_e.stalled():
                        raise OSError('GARRA TRAVADA')
                    elif garra_c.done() and garra_e.done():
                        brake_grip()
                        grip_state = GARRA_OK
                        print('GARRA FECHADA')
                    elif now-t_grip>TIMEOUT_GARRA_MS:
                        raise OSError('GARRA: TIMEOUT')
        except Exception as erro:
            falhar(erro)

        # Heartbeat fora do bloco de sensores/motores: continua ate com
        # defeito persistente, para o Hub 1 receber PARAR e liberar o botao.
        now=clock.time()
        connected = now-last_rx <= TIMEOUT_BLE_MS
        if now-last_tx>=INTERVALO_BLE_MS:
            seq_tx=(seq_tx+1)%100
            packed_obj = objeto + 8*grip_state + 32*grip_event_last
            last_tx=now
            try:
                radio.broadcast((ID_H2, token, seq_tx, packed_obj, d1, d2))
            except Exception as erro:
                falhar(erro)

        # Feedback tambem e protegido: uma falha aqui nao derruba o BLE.
        try:
            if falha_local:
                screen,color='X',Color.RED
            elif not connected:
                screen,color='A',Color.YELLOW
            elif grip_state == GARRA_MOVENDO:
                screen,color='G',Color.YELLOW
            elif estado_principal == PRONTO:
                screen,color='O',Color.GREEN
            elif estado_principal == CALIBRANDO:
                screen,color='C',Color.BLUE
            elif estado_principal in (FALHA,ERRO_CAL):
                screen,color='X',Color.RED
            elif estado_principal == CORRENDO:
                screen,color='R',Color.GREEN
            elif estado_principal == PARADO:
                screen,color='P',Color.YELLOW
            else:
                screen,color='A',Color.YELLOW
            if screen!=last_screen:
                hub.display.char(screen)
                hub.light.on(color)
                last_screen=screen
            if now-ultimo_status>2500:
                print('BLE',connected,'OBJ',objeto,'GARRA',grip_state,
                      'H1',estado_principal)
                ultimo_status=now
        except Exception as erro:
            falhar(erro)
        wait(15)


try:
    principal()
finally:
    # A parada manual pelo botao central continua funcionando normalmente.
    brake_grip_seguro()
    try:
        radio.broadcast(None)
    except Exception:
        pass
    try:
        hub.light.on(Color.RED)
    except Exception:
        pass
