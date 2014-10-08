import subprocess
import sys
import os
import select
import code 


import signal

class Console(code.InteractiveConsole):

    def start(self):
        # flush previous crap
        while self.ready():
            self.recv()

        # ready
        self.send('>>> ')

    # send prompt to indicate we are ready
    def send(self, data):
        prompt.write(data + '\n')
        prompt.flush()

    # receive command line
    def recv(self):
        res = cmd.readline()
        if res: return res.rstrip('\n')
        return res

    # is there any available command ?
    def ready(self):
        read, _, _ = select.select([ cmd ], [], [], 0)
        return read

    # execute next command, blocks on console input
    def next(self):
        line = self.recv()
        data = '>>> '
        
        if self.push( line ):
            data = '... '
            
        self.send( data )
        
    # convenience
    def poll(self):
        if self.ready(): self.next()


class History:
    def __init__(self, filename):
        self.filename = os.path.expanduser( filename )

    def __enter__(self):
        try:
            readline.read_history_file(self.filename)
            print 'loaded console history from', self.filename
        except IOError:
                pass
        return self

    def __exit__(self, type, value, traceback):
        readline.write_history_file( self.filename )
    

if __name__ == '__main__':
    import readline
    import sys
    import os
    import signal

    fd_in = int(sys.argv[1])
    file_in = os.fdopen( fd_in )

    fd_out = int(sys.argv[2])
    file_out = os.fdopen( fd_out, 'w' )

    def send(data):
        file_out.write(data + '\n')
        file_out.flush()

    def recv():
        return file_in.readline().rstrip('\n')
    
    try:
        with History( "~/.sofa-history" ):
            print 'console started'
            while True:
                send( raw_input( recv() ) )

    except KeyboardInterrupt:
        print 'console exited (SIGINT)'
        sys.exit(0)
        
    except EOFError:
        print 'console exited (EOF)'
        send( 'import sys; sys.exit(0)' )
        
else:

    from PySide import QtCore
    import signal
    import os
    
    # communication pipes
    prompt = os.pipe() 
    cmd = os.pipe()

    copy = os.dup(sys.stdin.fileno())

    # subprocess with in/out fd, and forwarding stdin
    sub = subprocess.Popen(['python', __file__, str(prompt[0]), str(cmd[1])],
                           stdin = copy)

    
    # open files
    prompt = os.fdopen(prompt[1], 'w')
    cmd = os.fdopen(cmd[0], 'r')

    # so here is what happens: when closing the main window, the
    # python interpreter is shut down abruptly, and Py_Finalize is not
    # called, so readline can not restore the terminal to a nice state
    # before exiting. if we try to terminate python from c++ static
    # destructors, some objects allocated by pyside still live on and
    # this results in a segfault. so a solution is to trap gui
    # termination using pyside, send a SIGINT to the child process and
    # wait for its termination

    # send SIGINT to child so that readline do not bork terminal
    def handler():
        sub.send_signal(signal.SIGINT)
        sub.wait()

    # TODO is there a way to do that cleanly ?
    app = QtCore.QCoreApplication.instance()
    app.aboutToQuit.connect( handler )
    
