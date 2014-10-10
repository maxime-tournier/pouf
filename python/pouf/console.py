from select import select
import os
import sys
import signal

if __name__ == '__main__':
    import readline

    # prompt input stream
    fd_in = int(sys.argv[1])    
    file_in = os.fdopen( fd_in )

    # cmd output stream
    fd_out = int(sys.argv[2])
    file_out = os.fdopen( fd_out, 'w' )

    # some helpers
    def send(data):
        file_out.write(data + '\n')
        file_out.flush()

    def recv():
        while True:
            res = file_in.readline().rstrip('\n')
            read, _, _ = select([ file_in ], [], [], 0)
            if not read: return res


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


    # main loop
    try:
        with History( "~/.sofa-history" ):
            print 'console started'
            while True:
                send( raw_input( recv() ) )

    except KeyboardInterrupt:
        print 'console exited (SIGINT)'
    except EOFError:
        print 'console exited (EOF)'

    finally:
        # sigint to parent 
        os.kill(os.getppid(), signal.SIGINT)
        
else:

    import subprocess
    import code
    import atexit


    class Console(code.InteractiveConsole):

        def __init__(self, locals = None):
            code.InteractiveConsole.__init__(self, locals)

        # execute next command, blocks on console input
        def next(self):
            line = recv()
            data = '>>> '

            if self.push( line ):
                data = '... '

            send( data )

        # convenience
        def poll(self):
            if ready(): self.next()


    # send prompt to indicate we are ready
    def send(data):
        prompt_out.write(data + '\n')
        prompt_out.flush()
            
    # receive command line
    def recv():
        res = cmd_in.readline()
        if res: return res.rstrip('\n')
        return res

    # is there any available command ?
    def ready():
        read, _, _ = select([ cmd_in ], [], [], 0)
        return read


    # communication pipes
    prompt = os.pipe() 
    cmd = os.pipe()

    # subprocess with in/out fd, and forwarding stdin
    sub = subprocess.Popen(['python', __file__,
                            str(prompt[0]), str(cmd[1])],
                           stdin = sys.stdin)

    
    # open the tubes !
    prompt_out = os.fdopen(prompt[1], 'w')
    cmd_in = os.fdopen(cmd[0], 'r')

    send('>>> ')

    
    # send SIGINT to child so that readline does not bork terminal
    def handler():
        sub.send_signal(signal.SIGINT)
        sub.wait()

    atexit.register( handler )
    
