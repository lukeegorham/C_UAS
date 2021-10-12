import socket

def Main():
    host = '192.168.1.131'  # Address of receiver
    port = 46555
    s = socket.socket()
    s.connect((host, port))
    message = input("->")
    while message != 'q':
        s.send(bytes(message, 'utf8'))
        data = s.recv(32)
        message = input("->")
    s.close()

if __name__ == '__main__':
    Main()