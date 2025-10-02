# input vectors should be (x, y) tuples with magnitude <= 1

def __sum(inputs):
    total = 0
    for i in inputs:
        total += i
    return total

def __avg(inputs):
    return __sum(inputs) / len(inputs)

def __clamp(v):
    mag = v[0]**2 + v[1]**2
    if mag > 1:
        mag = mag**0.5
        return (v[0]/mag, v[1]/mag)
    return v

def __normalize(v):
    mag = (v[0]**2 + v[1]**2)**0.5
    if mag > 0:
        return (v[0]/mag, v[1]/mag)
    return (0, 0)

#possible methods to use
def sum_and_clamp(inputs):
    return __clamp(__sum(inputs))

def sum_and_normalize(inputs):
    return __normalize(__sum(inputs))

def avg(inputs):
    return __avg(inputs)

# a little more complicated: basically its more sensitive, i think. it should be!
def sum_norm_avg(inputs, number_moving):

    return (number_moving / len(inputs)) * __avg(inputs)


