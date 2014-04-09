import os


def path():
    pwd = os.path.dirname( os.path.abspath(__file__) )
    res = '/'.join( pwd.split('/')[:-2] )
    return res


