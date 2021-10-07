import socket

def Main():
    host = '192.168.1.50' #The host on your client needs to be the external-facing IP address of your router. Obtain it from here https://www.whatismyip.com/
    port = 46555
    s = socket.socket()
    s.connect((host,port))
    message = input("->")
    while message != 'q':
        s.send(bytes(message, 'utf8'))
        data = s.recv(1024)
        message = input("->")
    s.close()

if __name__ == '__main__':
    Main()