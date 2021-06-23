from math import cos, sin, acos, sqrt, atan2, degrees, radians


def rpy2rv(roll, pitch, yaw):
    alpha = radians(yaw)
    beta = radians(pitch)
    gamma = radians(roll)

    ca = cos(alpha)
    cb = cos(beta)
    cg = cos(gamma)
    sa = sin(alpha)
    sb = sin(beta)
    sg = sin(gamma)

    r11 = ca * cb
    r12 = ca * sb * sg - sa * cg
    r13 = ca * sb * cg + sa * sg
    r21 = sa * cb
    r22 = sa * sb * sg + ca * cg
    r23 = sa * sb * cg - ca * sg
    r31 = -sb
    r32 = cb * sg
    r33 = cb * cg

    theta = acos((r11 + r22 + r33 - 1) / 2)
    sth = sin(theta)
    kx = (r32 - r23) / (2 * sth)
    ky = (r13 - r31) / (2 * sth)
    kz = (r21 - r12) / (2 * sth)

    return [(theta * kx), (theta * ky), (theta * kz)]


def rv2rpy(rx, ry, rz):
    theta = sqrt(rx * rx + ry * ry + rz * rz)
    kx = rx / theta
    ky = ry / theta
    kz = rz / theta
    cth = cos(theta)
    sth = sin(theta)
    vth = 1 - cos(theta)

    r11 = kx * kx * vth + cth
    r12 = kx * ky * vth - kz * sth
    r13 = kx * kz * vth + ky * sth
    r21 = kx * ky * vth + kz * sth
    r22 = ky * ky * vth + cth
    r23 = ky * kz * vth - kx * sth
    r31 = kx * kz * vth - ky * sth
    r32 = ky * kz * vth + kx * sth
    r33 = kz * kz * vth + cth

    beta = atan2(-r31, sqrt(r11 * r11 + r21 * r21))

    if beta > radians(89.99):
        beta = radians(89.99)
        alpha = 0
        gamma = atan2(r12, r22)
    elif beta < -radians(89.99):
        beta = -radians(89.99)
        alpha = 0
        gamma = -atan2(r12, r22)
    else:
        cb = cos(beta)
        alpha = atan2(r21 / cb, r11 / cb)
        gamma = atan2(r32 / cb, r33 / cb)

    return [degrees(gamma), degrees(beta), degrees(alpha)]
