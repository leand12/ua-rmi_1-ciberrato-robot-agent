import pytest

@pytest.mark.parametrize("dir,expected", [
    (0, [(0, +1), (+1, 0), (-1, 0)]), # y, x
    (1, [(+1, 0), (0, -1), (0, +1)]),
    (2, [(0, -1), (-1, 0), (+1, 0)]),
    (3, [(-1, 0), (0, +1), (0, -1)]),
])
def test_visited(dir, expected):
    m = 1 if dir < 2 else -1
    dir %= 2
    dy, dx = -dir * m, abs(dir - 1) * m
    assert expected == [
        (-dy, +dx),    # dyf = -1 * dir, dxf = abs(dir - 1)
        (+dx, +dy),    # dyl = dxf, dxl = -dyf
        (-dx, -dy),    # dyr = -dxf, dxr = dyf 
    ]


'''
EAST = (0, 0)
NORTH = (1, 90)
WEST = (2, 180)
SOUTH = (3, -90)

'''