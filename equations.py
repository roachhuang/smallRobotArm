
def eq1(t, v0, a0):
    '''
    in 0, 0.5s. col[1~3]: x, y, z
    '''
    dt = t - 0
    # col+1 coz time on the 1st col. parabloic (二次多項式)
    # d(t) = d0+v0t+1/2*at^2, here we replace d with theta
    return v0 * dt + 0.5 * a0 * dt**2


def eq2(t, v1):
    ''' in 0.5, 1.75 '''
    dt = t - 0.25
    #v1 = v[1, col]
    # linear segment.
    return v1 * dt


def eq3(t, ts1, v1, acc1):
    '''in 1.75, 2.25'''
    # v1 = v[1, col]
    # acc1 = a[1, col]
    dt1 = t - 0.25
    dt2 = t - (ts1 - 0.25)
    return v1 * dt1 + 0.5 * acc1 * dt2**2


def eq4(dt, v2):
    ''' in 2.25, 5.75'''
    #dt = t - ts[1]
    # v2 = v[2, col]
    return v2 * dt


def eq5(t, ts, v2, acc2):
    ''' 5.75, 6.25'''
    dt1 = t - ts[1]
    dt2 = t - (ts[2] - 0.25)
    # v2 = v[2, col]
    # acc2 = a[2, col]
    return v2 * dt1 + 1 / 2 * acc2 * dt2**2


def eq6(dt: float, v3) -> float:
    ''' 6.25, 8.5 '''
    #dt = t - ts[2]
    #v3 = v[3, col]
    return v3 * dt

# col - 0:x, 1:y, 2:theta


def eq7(t, ts, v3, acc3, totalPoints):
    '''# 8.5 ~9s'''
    dt1 = t - ts[2]
    dt2 = t - (ts[totalPoints - 1] - 0.5)
    #v3 = v[3, col]
    #acc3 = a[3, col]
    return v3 * dt1 + 1 / 2 * acc3 * dt2**2
