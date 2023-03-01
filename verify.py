def verify(t, a0, a1, a2, a3):
    return a0 + a1 * t + a2 * (t**2) + a3 * (t**3)


print(verify(2, 5, 1.67, -2.07, 0.37))