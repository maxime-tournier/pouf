
import subprocess
import sys
import os
import select
import code 

class Console(code.InteractiveConsole):

    def start(self):
        # communication pipes
        prompt = os.pipe() 
        cmd = os.pipe()

        # subprocess with in/out fd, and forwarding stdin
        sub = subprocess.Popen(['python', __file__, str(prompt[0]), str(cmd[1])],
                               stdin = sys.stdin)

        # open files
        self.prompt = os.fdopen(prompt[1], 'w')
        self.cmd = os.fdopen(cmd[0], 'r')

        # ready
        self.send('>>> ')

    # send prompt to indicate we are ready
    def send(self, data):
        self.prompt.write(data + '\n')
        self.prompt.flush()

    # receive command line
    def recv(self):
        res = self.cmd.readline()
        if res: return res.rstrip('\n')
        return res

    # is there any available command ?
    def ready(self):
        read, _, _ = select.select([ self.cmd ], [], [], 0)
        return read

    # execute next command, blocks on console input
    def next(self):
        line = self.recv()
        prompt = '>>> '
        
        if self.push( line ):
            prompt = '... '
            
        self.send( prompt )
        
    # convenience
    def poll(self):
        if self.ready(): self.next()


if __name__ == '__main__':
    import readline
    import sys
    import os

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
        print 'console started'
        while True:
            send( raw_input( recv() ) )
            
    except EOFError:
        print 'console exited (EOF)'
        send( 'import sys; sys.exit()' )
        
