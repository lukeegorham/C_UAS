import socket

def Main():
    host = '192.168.1.50'
    port = 46555
    s = socket.socket()
    s.bind((host, port))

    s.listen(1)
    c, addr = s.accept()
    while True:
        data = c.recv(1024)
        if not data:
            break
        data = str(data).upper()
        c.send(data)
    c.close()
if __name__ == '__main__':
    Main()