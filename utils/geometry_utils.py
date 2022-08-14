def calc_linear_eq(coor1, coor2):
    # line parallel to y axis
    if coor2[0] - coor1[0] == 0:
        return None, coor2[0]

    a = (coor2[1] - coor1[1]) / (coor2[0] - coor1[0])
    b = coor2[1] - a * coor2[0]
    return a, b
