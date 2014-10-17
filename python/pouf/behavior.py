
release = 'release'
available = 'available'


def adapt(response, request):
    if not response:
        return {resource: available for resource in request}
    return response


def parallel(*args):
    args = [ x for x in args]
    
    # merge requests by priority in lhs
    def merge(x, y):
        if not y: return

        for k in y:
            yk = y[k]
            xk = x[k] if k in x else None

            if not xk or xk < yk: x[k] = yk


    # initial response is None
    resp = {gen: None for gen in args }
    
    # nobody holds anything
    holder = {}

    while args:
        req = {}
        request = {}

        current = None
        to_remove = []
        
        for gen in args:
            current = gen
            # print 'current', gen
            try:
                req[gen] = gen.send( resp[gen] ) or {}
                merge(request, req[gen] )
            except StopIteration:
                to_remove.append( current )
                # print current, 'stopped'

        for x in to_remove: args.remove(x)

        if not args: raise StopIteration
    
        # transmit request upwards and wait for response
        response = yield request
        response = adapt(response, request)
        
        holder = {}
        
        # update who holds what: permission was granted and resource
        # still requested
        for gen in req:
            for resource in req[gen]:
                if resp[gen] and resource in resp[gen] and resp[gen][resource]:
                    holder[resource] = gen
        
        # build per generator response
        resp = {}

        for gen in args:

            resp[gen] = {}
            for resource in req[gen]:

                have_priority = request[resource] <= req[gen][resource]
                have_access = resource not in holder or holder[resource] is gen
                was_holder = resource in holder and holder[resource] is gen
                
                r = None

                global release
                
                if was_holder and not have_priority: r = release
                if have_priority and have_access: r = response[resource]

                resp[gen][resource] = r 

# sequence
def sequence(*args):
    for gen in args:
        response = None
        try:
            while True:
                request = gen.send( response )
                response = yield request
                response = adapt(response, request)
        except StopIteration:
            pass
