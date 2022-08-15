def calc_linear_eq(point1, point2):
    """
    Calculates the coefficients of the equation of a linear line between 2 inputted points.
    :param point1: the first point for the line.
    :param point2: the second point for the line.
    :return: the coefficients of the line between point1 and point2.
    """
    # line parallel to y axis
    if point2[0] - point1[0] == 0:
        return None, point2[0]

    a = (point2[1] - point1[1]) / (point2[0] - point1[0])
    b = point2[1] - a * point2[0]
    return a, b
