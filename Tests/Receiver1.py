import socket


def Main():
    hostname = socket.gethostname()
    port = 46555

    s = socket.socket()
    s.bind((hostname, port))
    s.listen(1)
    c, addr = s.accept()

    while True:
        data = c.recv(1024)
        if not data:
            break
        print(str(data, 'utf8'))
        c.send(data)
    c.close()


if __name__ == '__main__':
    Main()
