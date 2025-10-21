import socket, struct, time, math, os

CMD_HOST  = str(os.getenv("CMD_HOST", "127.0.0.1"))
CMD_PORT  = int(os.getenv("CMD_PORT", "5555"))
TEL_HOST  = str(os.getenv("TEL_HOST", "0.0.0.0"))
TEL_PORT  = int(os.getenv("TEL_PORT", "5600"))
print(CMD_HOST, CMD_PORT)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

tele = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tele.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
tele.bind((TEL_HOST, TEL_PORT))
tele.listen(1)
print("waiting to establish telemetry channel")
teleConn, _ = tele.accept()
print("established telemetry channel")

maxspeed = 1.0

def send_cmd(forwardSpeed, angularSpeed):
    packet = struct.pack("<2f", forwardSpeed, angularSpeed)
    sock.sendto(packet, (CMD_HOST, CMD_PORT))

def tele_receive(size):
    t0 = time.time()
    left = size
    buf = b''
    while len(buf) < size:
        b = teleConn.recv(size - len(buf))
        buf += b
        if time.time() - t0 > 1:
            print('telemetry timeout, perhaps finished')
            exit(0)
    return buf

def read_tele():
    global x, y, th, lidar, fl, fr, alpha
    bytes = tele_receive(48)
    hdr, x, y, th, vx, vy, vth, wx, wy, wz, n = struct.unpack('<xxxx4s9fI', bytes)
    bytes = tele_receive(n * 4)
    lraw = list(struct.unpack('<%sf' % n, bytes))
    lidar = []
    n //= 4
    alpha = math.pi / 2 / n
    for i in range(n):
        lidar.append(sum(lraw[i*4:i*4+4])/4)
    fl = lidar[n//2]
    fr = lidar[n//2-1]

def print_tele():
    print('coords: %.3f %.3f %.3f %.1f %.1f %.1f %.1f' % ((fl + fr) / 2, lidar[-1], lidar[0], lt_wall_angle(), rt_wall_angle(), lt_wall_dist(), rt_wall_dist()))


def drive_until(v, da, cond):
    global lastX, lastY, lastTh
    t0 = time.time()
    send_cmd(v, da)
    while True:
        read_tele()
        dt = time.time() - t0
        if eval(cond):
            print_tele()
            break
    lastX, lastY, lastTh = x, y, th

def curve(v, da, cond):
    print('curve', v, da)
    drive_until(v, da, cond)

def forward(cond):
    print('forward')
    drive_until(1, 0, cond)

def backward(cond):
    print('backward')
    drive_until(-maxspeed, 0, cond)

def backleft(turn, cond):
    print('back-left', turn)
    drive_until(-maxspeed, -turn, cond)

def backright(turn, cond):
    print('back-right', turn)
    drive_until(-maxspeed, turn, cond)

def dist():
    return math.hypot(x - lastX, y - lastY)

def dth():
    d = th - lastTh
    if d < -math.pi:
        d += 2 * math.pi
    elif d >= math.pi:
        d -= 2 * math.pi
    return d

def rt_wall_angle():
    if lidar[1] == lidar[0]:
        return 90
    d = alpha * (lidar[0] + lidar[1]) / 2
    return math.atan(d / (lidar[1] - lidar[0])) / math.pi * 180

def lt_wall_angle():
    if lidar[-2] == lidar[-1]:
        return 90
    d = alpha * (lidar[-1] + lidar[-2]) / 2
    return math.atan(d / (lidar[-2] - lidar[-1])) / math.pi * 180

def lt_wall_dist():
    return math.sin(lt_wall_angle() / 180 * math.pi) * lidar[-1]

def rt_wall_dist():
    return math.sin(rt_wall_angle() / 180 * math.pi) * lidar[0]

backleft(0.12, 'x < -0.7')
backleft(0.2, 'rt_wall_angle() < 0 and rt_wall_angle() > -70')
backward('lidar[-1] < 1')
backleft(2, 'rt_wall_angle() > 62')
backward('dt > 5 and lidar[0] < 0.7')
curve(-0.5, 1.5, 'dth() > math.pi * 0.15')
backward('dt > 3 and fl > 4')
backward('dist() > 0.25')
backleft(2, 'lt_wall_angle() < 30')
backward('lt_wall_dist() < 0.5')
backright(1, 'lt_wall_angle() > 40')
backright(0.5, 'lt_wall_angle() > 43')
curve(-0.7, 0, 'fl < 1')
backleft(0.8, 'dth() < -math.pi / 6')
backright(0.5, 'dt > 5 and rt_wall_angle() < 40')
backward('lidar[-20] > 3.48')
print([lidar[i] for i in range(0, 90, 5)])
curve(-0.4, 2, 'dt > 3.5')
backward('dt > 7')
print("control sequence completed")
