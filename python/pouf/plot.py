import matplotlib.pyplot as plt


def draw( data, figure = None, log = None ):
    if figure:
        plt.figure( figure )

    plt.clf()
        
    for label in data:
        (x, y) = data[label]
        plt.plot(x, y, label = label)

    if log:
        plt.yscale('log')
    plt.legend()
    
    plt.draw()




plt.ion()
plt.show()



import socket
import pickle

from StringIO import StringIO

class Client:

    def __init__(self):
        self.ip = "127.0.0.1"
        self.port = 5005
    
    def __enter__(self):
        self.sock = socket.socket(socket.AF_INET, # Internet
                                  socket.SOCK_STREAM)
        self.sock.connect( (self.ip, self.port) )
        return self


    def __exit__(self, type, value, traceback):
        self.sock.close()
    
    def send(self, data ) :
        self.sock.sendall( pickle.dumps(data) )
        

class Server:
    
    def __init__(self):
        self.ip = "127.0.0.1"
        self.port = 5005


    def accept(self):
        self.conn, addr = self.sock.accept()
        self.fp = self.conn.makefile()

    def __enter__(self):
        
        self.sock = socket.socket(socket.AF_INET, # Internet
                                  socket.SOCK_STREAM) 
        self.sock.bind((self.ip, self.port))
        self.sock.listen(1)

        
        self.accept()
        self.sock.setblocking(False)
        
        return self
    

    def __exit__(self, type, value, traceback):
        self.conn.close()

        
    def recv(self):
        while True:
            try:
                return pickle.Unpickler(self.fp).load()
            except EOFError:
                self.accept()
        

    
