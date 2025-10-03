from pyglet.math import Vec2
# input vectors should be Vec2s

def __avg(inputs: list[Vec2]) -> Vec2:
    if len(inputs) == 0:
        return Vec2(0, 0)
    return sum(inputs) / len(inputs)

def __clamp(v: Vec2) -> Vec2:
    mag = v.length_squared()
    if mag > 1:
        mag = mag**0.5
        return v / mag
    return v

#possible methods to use
def sum_and_clamp(inputs: list[Vec2]) -> Vec2:
    return __clamp(sum(inputs))

def sum_and_normalize(inputs: list[Vec2]) -> Vec2:
    return sum(inputs).normalize()

def avg(inputs: list[Vec2]) -> Vec2:
    return __avg(inputs)

# a little more complicated: basically its more sensitive, i think. it should be!
def sum_norm_avg(inputs: list[Vec2], number_moving: int) -> Vec2:
    return (number_moving / len(inputs)) * __avg(inputs)


