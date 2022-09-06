# Import math Library
import math

# Return the sine value of 30 degrees
print(round(math.cos(math.radians(-90))))

# Return the sine value of 90 degrees
print(math.sin(math.radians(-90)))


def get_ti2i_1(i, t=NULL):
    alp, ai, di, ti = symbols('alp, ai, di, ti')
    (alp, ai, di) = dh_tbl[i - 1, :]
    #alp = dh_tbl[i - 1][0]
    #ai='a'+str(i)
    #di='d'+str(i)
    if (t==NULL):
        ti='t'+str(i)
    tm = np.array([
        [cos(ti), -sin(ti), 0, ai],
        [
        sin(ti) * round(cos(alp)),
        cos(ti) * round(cos(alp)),
        -sin(alp),
        round(-sin(alp)) * di],
        [
        sin(ti) * sin(alp),
        cos(ti) * sin(alp),
        round(cos(alp)),
        round(cos(alp)) * di
        ],
        [0, 0, 0, 1]
        ])
    print(f't{i}-{i-1}:', tm)
    return (tm)
